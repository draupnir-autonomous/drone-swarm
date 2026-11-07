/**
 * Transforms the Gazebo lidar's point cloud (published in the sensor/body
 * frame, stamped with Gazebo sim time) into the world/ENU frame that
 * plan_env's SDFMap::inputPointCloud expects (it treats incoming points as
 * already-world-frame coordinates, mirroring how the original MARSIM
 * pcl_render_node published world-frame clouds directly), then republishes
 * on /map_ros/cloud.
 *
 * Deliberately does NOT use a message_filters time-sync against /odom_world:
 * the Gazebo-bridged cloud is stamped with Gazebo sim time while this
 * process's own clock (used to stamp /odom_world in odometry_bridge) is
 * wall-clock, and with the simulation's real_time_factor < 1 those two
 * diverge continuously - an ApproximateTime sync between them would never
 * match. Instead this node just caches the latest odometry sample and uses
 * it whenever a new cloud arrives (odometry publishes far faster than the
 * 10 Hz lidar, so the staleness is negligible), and republishes with this
 * node's own wall-clock stamp - the same time domain odometry_bridge uses -
 * so that plan_env's own internal /map_ros/cloud + /odom_world sync (both
 * now wall-clock-stamped) lines up correctly downstream.
 */
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <deque>
#include <map>
#include <string>
#include <vector>

// POSA GIUSTA PER OGNI NUVOLA (aggiunta di ottobre 2026).
//
// La versione precedente trasformava ogni nuvola con l'ULTIMA odometria
// ricevuta. Il commento qui sopra la dava per trascurabile ("odometry publishes
// far faster than the 10 Hz lidar"), ed e' vero col GPS, dove PX4 pubblica a
// ~100 Hz. Senza GPS no: l'odometria viene da FAST-LIO, a 10 Hz, ed e' calcolata
// SULLA STESSA scansione, quindi arriva dopo la nuvola. Ogni nuvola veniva
// posata con la posa della scansione PRECEDENTE, vecchia di 100 ms: a 1 rad/s di
// imbardata un punto a 5 m finisce mezzo metro fuori posto. Erano le
// "strisciate" a ventaglio nelle mappe delle run senza GPS (precisione 70-85%
// contro 74-92% col GPS).
//  - senza GPS (lio_topic impostato): la nuvola attende la posa di FAST-LIO con
//    lo STESSO istante (FAST-LIO marca l'odometria con lidar_end_time, cioe' il
//    tempo della scansione) e viene trasformata con quella;
//  - col GPS: l'odometria di PX4 e' conservata in uno storico, e la posa viene
//    interpolata all'istante della scansione, ricavato dall'eta' della nuvola
//    misurata sull'orologio della simulazione (/clock).

class CloudTransformBridge : public rclcpp::Node {
public:
  CloudTransformBridge() : Node("c2x_cloud_transform_bridge"), have_odom_(false) {
    // Mount offset of the lidar in the body frame. c2x_description's
    // x500_mid360 puts lidar_sensor_link at (0, 0, 0.25) relative to base_link,
    // and the cloud arrives expressed in that sensor frame - applying only the
    // base_link pose (as this bridge originally did) shifted the whole map
    // 0.25m down, i.e. 2.5 voxels at sdf_map.resolution=0.1. Keep in sync with
    // the model's <pose relative_to="base_link">.
    const std::vector<double> default_offset = { 0.0, 0.0, 0.25 };
    const auto offset = this->declare_parameter<std::vector<double>>(
        "lidar_offset_body", default_offset);
    if (offset.size() == 3) {
      lidar_offset_body_ = Eigen::Vector3d(offset[0], offset[1], offset[2]);
    } else {
      RCLCPP_WARN(this->get_logger(),
          "lidar_offset_body must have 3 elements, got %zu - using the x500_mid360 default",
          offset.size());
      lidar_offset_body_ = Eigen::Vector3d(0.0, 0.0, 0.25);
    }

    // Self-hit rejection. The mid360's wide vertical FOV sees the vehicle's own
    // arms and legs: the nearest returns in a real cloud from this model sit at
    // 0.23-0.26m, i.e. inside the airframe. Mapped as obstacles and then grown
    // by sdf_map.obstacles_inflation (0.199m) they enclose the drone in its own
    // position, and every A* to a viewpoint fails at the first expansion with
    // "No path to next viewpoint" - the drone can never leave. Matches the
    // self_hit_radius the reactive proximity check in position_command_bridge
    // already applies to the same cloud for the same reason. Measured against
    // the sensor origin, before the world transform below.
    min_range_ = this->declare_parameter<double>("min_range", 0.5);

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_ros/cloud", 50);
    // Stessa nuvola senza alcun compagno, per la mappa 3D (vedi trasformaEPubblica).
    cloud_mappa3d_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("map_ros/cloud_mappa3d", 50);
    // Modalita' dell'ambiente, dedotta dai ritorni verso l'alto (vedi onCloud).
    env_pub_ = this->create_publisher<std_msgs::msg::Bool>("environment/closed", rclcpp::QoS(1).transient_local());
    // Soglie con isteresi: si passa a CHIUSO sopra la prima, ad APERTO sotto la
    // seconda. Misurato: ufficio 100%, campo aperto 0-7%, quindi qualunque
    // valore in mezzo separa - le soglie larghe servono solo a non oscillare
    // durante le transizioni (per esempio un drone che esce da un capannone).
    env_closed_enter_ = this->declare_parameter<double>("env_closed_enter", 0.60);
    env_open_enter_ = this->declare_parameter<double>("env_open_enter", 0.40);

    // Trasformazione di spawn, la stessa di odometry_bridge: serve per portare
    // in coordinate mondo la posa grezza di FAST-LIO.
    const double syaw = this->declare_parameter<double>("spawn_yaw", 0.0);
    spawn_rot_ = Eigen::Quaterniond(Eigen::AngleAxisd(syaw, Eigen::Vector3d::UnitZ()));
    spawn_trans_ = Eigen::Vector3d(this->declare_parameter<double>("spawn_x", 0.0),
        this->declare_parameter<double>("spawn_y", 0.0),
        this->declare_parameter<double>("spawn_z", 0.0));
    // Senza GPS: topic dell'odometria GREZZA di FAST-LIO (marcata col tempo della
    // scansione). Vuoto = modalita' GPS.
    lio_topic_ = this->declare_parameter<std::string>("lio_topic", "");
    // Col GPS: compensazione dell'eta' della nuvola (vedi la nota in testa).
    latency_comp_ = this->declare_parameter<bool>("latency_compensation", true);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world", 25,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          last_odom_ = *msg;
          have_odom_ = true;
          // Storico per l'interpolazione, marcato con l'orologio di questo nodo.
          storico_.push_back({ this->now(), posizione(*msg), orientamento(*msg) });
          while (storico_.size() > 300) storico_.pop_front();
        });

    if (!lio_topic_.empty()) {
      lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          lio_topic_, rclcpp::QoS(rclcpp::KeepLast(20)),
          [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            lio_.push_back({ rclcpp::Time(msg->header.stamp, RCL_ROS_TIME), posizione(*msg),
                orientamento(*msg) });
            while (lio_.size() > 50) lio_.pop_front();
            elaboraInAttesa();
          });
      RCLCPP_WARN(this->get_logger(),
          "nuvole posate con la posa di FAST-LIO dello stesso istante (%s)", lio_topic_.c_str());
    } else {
      clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
          [this](const rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
            ultimo_clock_ = rclcpp::Time(msg->clock, RCL_ROS_TIME);
            have_clock_ = true;
          });
    }

    // I COMPAGNI NON SONO OSTACOLI (sciame, ottobre 2026). Ogni lidar vede anche gli
    // altri droni, e i loro ritorni entravano nella mappa del pianificatore (ostacoli
    // inesistenti che bloccano i corridoi finche' un'altra scansione non li libera) e
    // nella mappa 3D, dove restano per sempre: nella prima run a tre droni su office_small
    // 38 mila celle da 2 cm nello spazio libero lungo le traiettorie, a piu' di 0.3 m da
    // ogni ostacolo vero, e precisione 3D scesa al 93.8% (drone singolo ~100%). Si tolgono
    // dalla nuvola della mappa 3D (map_ros/cloud_mappa3d) i punti dentro un cilindro attorno
    // a ogni compagno, e da quella del pianificatore (map_ros/cloud) solo quelli attorno ai
    // compagni IN MOTO (vedi trasformaEPubblica) (odometria in coordinate mondo,
    // portata all'istante della scansione con la sua velocita'). Il cilindro (0.6 m di
    // raggio, da 0.45 m sotto a 0.55 m sopra il centro) contiene l'x500 col lidar e le
    // eliche (0.37 m dal centro) piu' l'errore di posa; i compagni volano ad almeno 0.5 m
    // dalle pareti, quindi quello che si perde di muro e' un attimo e lo rivedono le
    // scansioni successive. Con un drone solo non c'e' nessuno da togliere.
    drone_id_ = this->declare_parameter<int>("drone_id", 1);
    const int drone_num = this->declare_parameter<int>("drone_num", 1);
    raggio_compagni_ = this->declare_parameter<double>("peer_filter_radius", 0.6);
    compagni_giu_ = this->declare_parameter<double>("peer_filter_below", 0.45);
    compagni_su_ = this->declare_parameter<double>("peer_filter_above", 0.55);
    velocita_compagni_ = this->declare_parameter<double>("peer_filter_min_speed", 0.3);
    for (int k = 1; k <= drone_num; ++k) {
      if (k == drone_id_) continue;
      compagni_[k] = Compagno();
      compagni_subs_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
          "/drone_" + std::to_string(k) + "/odom_world", rclcpp::QoS(rclcpp::KeepLast(5)),
          [this, k](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            Compagno& c = compagni_[k];
            c.t = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
            c.p = posizione(*msg);
            c.v = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                msg->twist.twist.linear.z);
            c.valido = true;
          }));
    }

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(10));
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/points", cloud_qos,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg); });
  }

private:
  struct Posa {
    rclcpp::Time t;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
  };

  static Eigen::Vector3d posizione(const nav_msgs::msg::Odometry& o) {
    return Eigen::Vector3d(o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z);
  }
  static Eigen::Quaterniond orientamento(const nav_msgs::msg::Odometry& o) {
    return Eigen::Quaterniond(o.pose.pose.orientation.w, o.pose.pose.orientation.x,
        o.pose.pose.orientation.y, o.pose.pose.orientation.z).normalized();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
    if (!lio_topic_.empty()) {
      // Senza GPS: la posa di questa scansione arrivera' fra qualche ms.
      in_attesa_.push_back(cloud_msg);
      while (in_attesa_.size() > 10) in_attesa_.pop_front();
      elaboraInAttesa();
      return;
    }
    if (!have_odom_) return;

    Eigen::Vector3d t = posizione(last_odom_);
    Eigen::Quaterniond q = orientamento(last_odom_);
    // Col GPS: posa all'istante della scansione. L'eta' della nuvola si misura
    // sull'orologio della simulazione (la nuvola e' marcata in tempo simulato) e
    // si riporta sull'orologio di questo nodo, a cui e' marcato lo storico.
    if (latency_comp_ && have_clock_ && storico_.size() >= 2) {
      const double eta = std::clamp((ultimo_clock_ - rclcpp::Time(cloud_msg->header.stamp,
          RCL_ROS_TIME)).seconds(), 0.0, 0.3);
      eta_media_ = (eta_media_ < 0.0) ? eta : 0.95 * eta_media_ + 0.05 * eta;
      interpola(this->now() - rclcpp::Duration::from_seconds(eta), t, q);
    }
    // Sensor origin in world = body origin + R_body * mount offset. The sensor
    // shares the body's orientation (the model's <pose> has zero rotation), so
    // only the translation needs correcting.
    t += q * lidar_offset_body_;
    trasformaEPubblica(cloud_msg, t, q);
  }

  // Interpolazione lineare (posizione) e sferica (orientamento) nello storico.
  void interpola(const rclcpp::Time& tq, Eigen::Vector3d& p, Eigen::Quaterniond& q) const {
    if (tq >= storico_.back().t) { p = storico_.back().p; q = storico_.back().q; return; }
    if (tq <= storico_.front().t) { p = storico_.front().p; q = storico_.front().q; return; }
    for (size_t i = storico_.size() - 1; i > 0; --i) {
      const Posa& a = storico_[i - 1];
      const Posa& b = storico_[i];
      if (a.t <= tq && tq <= b.t) {
        const double span = (b.t - a.t).seconds();
        const double s = span > 1e-6 ? (tq - a.t).seconds() / span : 0.0;
        p = a.p + s * (b.p - a.p);
        q = a.q.slerp(s, b.q);
        return;
      }
    }
  }

  // Senza GPS: si posano le nuvole per cui e' arrivata la posa di FAST-LIO con
  // lo stesso istante (entro 20 ms). Quelle rimaste indietro di oltre 0.5 s
  // rispetto all'ultima posa non avranno mai la loro: si scartano.
  void elaboraInAttesa() {
    while (!in_attesa_.empty() && !lio_.empty()) {
      const auto& c = in_attesa_.front();
      const rclcpp::Time tc(c->header.stamp, RCL_ROS_TIME);
      const Posa* trovata = nullptr;
      for (const auto& l : lio_)
        if (std::abs((l.t - tc).seconds()) < 0.02) { trovata = &l; break; }
      // Posa ESATTA della scansione, oppure INTERPOLATA fra le due pose di
      // FAST-LIO che la racchiudono. Sotto carico FAST-LIO non elabora tutte le
      // scansioni - nella run V9_LIO 1 ne saltava circa il 30% - e per quelle la
      // posa non arrivera' mai: scartarle lasciava la mappa rada (10 mila punti
      // di parete invece di ~45 mila) e l'esplorazione ferma al 41%. Fra due
      // pose distanti 100-300 ms l'interpolazione e' precisa al centimetro.
      Posa interpolata;
      if (!trovata && lio_.back().t >= tc) {
        for (size_t i = lio_.size() - 1; i > 0; --i) {
          const Posa& a = lio_[i - 1];
          const Posa& b = lio_[i];
          if (a.t <= tc && tc <= b.t && (b.t - a.t).seconds() < 0.5) {
            const double span = (b.t - a.t).seconds();
            const double s = span > 1e-6 ? (tc - a.t).seconds() / span : 0.0;
            interpolata = { tc, a.p + s * (b.p - a.p), a.q.slerp(s, b.q) };
            trovata = &interpolata;
            ++interpolate_;
            break;
          }
        }
      }
      if (trovata) {
        // Posa del lidar in coordinate mondo. FAST-LIO stima la posa della
        // terna lidar/IMU rispetto a quella iniziale; la terna iniziale sta
        // sopra il punto di spawn di lidar_offset_body (velivolo a terra, in
        // piano).
        const Eigen::Quaterniond q = (spawn_rot_ * trovata->q).normalized();
        const Eigen::Vector3d t =
            spawn_rot_ * trovata->p + spawn_trans_ + spawn_rot_ * lidar_offset_body_;
        trasformaEPubblica(c, t, q);
        in_attesa_.pop_front();
        continue;
      }
      if ((lio_.back().t - tc).seconds() > 0.5) {
        ++scartate_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "nuvola senza posa di FAST-LIO corrispondente: scartata (%lu finora)", scartate_);
        in_attesa_.pop_front();
        continue;
      }
      break;  // la sua posa non e' ancora arrivata
    }
  }

  // t, q: posizione e orientamento del SENSORE in coordinate mondo.
  void trasformaEPubblica(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
      const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
    {
      static rclcpp::Clock orologio(RCL_STEADY_TIME);
      RCLCPP_INFO_THROTTLE(this->get_logger(), orologio, 10000,
          "[Nuvola] %s, eta' media della nuvola %.0f ms, interpolate %lu, scartate %lu, "
          "punti sui compagni tolti %lu",
          lio_topic_.empty() ? "posa di PX4 interpolata" : "posa di FAST-LIO della stessa scansione",
          1000.0 * std::max(0.0, eta_media_), interpolate_, scartate_, tolti_compagni_);
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_in, cloud_out, cloud_mappa3d;
    pcl::fromROSMsg(*cloud_msg, cloud_in);
    cloud_out.reserve(cloud_in.size());
    if (!compagni_.empty()) cloud_mappa3d.reserve(cloud_in.size());

    // Compagni all'istante della scansione (vedi compagni_ nel costruttore): solo quelli
    // con un'odometria recente, portati avanti con la loro velocita' (al massimo 0.3 s).
    // in_moto: oltre peer_filter_min_speed. Un compagno FERMO (in attesa del via, a fine
    // missione, in sosta) e' un ostacolo vero per pianificatore e filtro di sicurezza e
    // resta nella loro nuvola; quelli in moto li gestiscono lo scambio delle traiettorie e la
    // separazione fra droni del ponte dei comandi. Seconda run a tre droni su office_small:
    // col filtro anche sui fermi, il drone 2 e' andato addosso al drone 3 fermo in attesa del
    // via (invisibile alla sua mappa). Dalla mappa 3D si tolgono tutti.
    struct Centro { Eigen::Vector3d p; bool in_moto; };
    std::vector<Centro> centri_compagni;
    {
      const rclcpp::Time tc(cloud_msg->header.stamp, RCL_ROS_TIME);
      for (const auto& kv : compagni_) {
        const Compagno& c = kv.second;
        if (!c.valido) continue;
        const double dt = (tc - c.t).seconds();
        if (std::abs(dt) > 1.0) continue;
        centri_compagni.push_back({ c.p + c.v * std::clamp(dt, -0.3, 0.3),
            c.v.norm() > velocita_compagni_ });
      }
    }
    const double r2_compagni = raggio_compagni_ * raggio_compagni_;

    const double min_range_sq = min_range_ * min_range_;
    size_t non_finite = 0;
    for (const auto& p : cloud_in.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        ++non_finite;
        continue;
      }
      const double range_sq = double(p.x) * p.x + double(p.y) * p.y + double(p.z) * p.z;
      if (range_sq < min_range_sq) continue;  // self-hit, see min_range_
      Eigen::Vector3d p_world = q * Eigen::Vector3d(p.x, p.y, p.z) + t;
      // 0 = su nessun compagno, 1 = su un compagno fermo, 2 = su un compagno in moto.
      int su_un_compagno = 0;
      for (const auto& c : centri_compagni) {
        const double dz = p_world.z() - c.p.z();
        if (dz < -compagni_giu_ || dz > compagni_su_) continue;
        if ((p_world - c.p).head<2>().squaredNorm() < r2_compagni) {
          su_un_compagno = std::max(su_un_compagno, c.in_moto ? 2 : 1);
          if (su_un_compagno == 2) break;
        }
      }
      pcl::PointXYZ pw;
      pw.x = p_world.x();
      pw.y = p_world.y();
      pw.z = p_world.z();
      if (su_un_compagno > 0) ++tolti_compagni_;
      if (su_un_compagno < 2) cloud_out.push_back(pw);
      if (!compagni_.empty() && su_un_compagno == 0) cloud_mappa3d.push_back(pw);
    }

    // Ritorni per FASCIA DI ELEVAZIONE.
    //
    // Il rapporto aggregato non distingue chiuso da aperto: misurato, da' 66-69%
    // nell'ufficio e 67-72% nel campo, cioe' lo stesso valore. Quel ~32% di
    // non-ritorni costante in entrambe le scene e' un artefatto di come Gazebo
    // riempie la nuvola, non una proprieta' dell'ambiente.
    // Cio' che separa davvero i due casi e' se tornano i raggi diretti VERSO
    // L'ALTO: un soffitto li restituisce, il cielo no. Ed e' anche la condizione
    // che conta per l'algoritmo, perche' decide se il volume sopra il velivolo
    // e' osservabile o restera' ignoto per sempre.
    // Un punto non finito non ha coordinate, quindi la direzione si ricava
    // dall'INDICE: la nuvola di gz e' organizzata (larghezza x altezza =
    // raggi orizzontali x verticali) e la riga codifica l'elevazione.
    if (cloud_in.height > 1) {
      const int rows = static_cast<int>(cloud_in.height);
      const int cols = static_cast<int>(cloud_in.width);
      int up_tot = 0, up_ok = 0, mid_tot = 0, mid_ok = 0, dn_tot = 0, dn_ok = 0;
      for (int r = 0; r < rows; ++r) {
        // Elevazione del fascio r, dagli angoli del sensore (+-50 gradi).
        const double elev = -50.0 + 100.0 * double(r) / double(rows - 1);
        for (int c = 0; c < cols; ++c) {
          const auto& p = cloud_in.points[size_t(r) * cols + c];
          const bool ok = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
          if (elev > 30.0) { ++up_tot; up_ok += ok; }
          else if (elev < -30.0) { ++dn_tot; dn_ok += ok; }
          else { ++mid_tot; mid_ok += ok; }
        }
      }
      const double up_ratio = double(up_ok) / std::max(1, up_tot);
      // Media esponenziale: una singola scansione puo' essere ingannata da un
      // ostacolo alto che passa sopra il velivolo.
      up_ratio_avg_ = (up_ratio_avg_ < 0.0) ? up_ratio : (0.9 * up_ratio_avg_ + 0.1 * up_ratio);

      const bool was_closed = env_closed_;
      if (up_ratio_avg_ > env_closed_enter_) env_closed_ = true;
      else if (up_ratio_avg_ < env_open_enter_) env_closed_ = false;

      if (env_closed_ != was_closed || !env_published_) {
        std_msgs::msg::Bool m;
        m.data = env_closed_;
        env_pub_->publish(m);
        env_published_ = true;
        RCLCPP_WARN(this->get_logger(), "[EnvSense] ambiente %s (ritorni verso l'alto %.0f%%)",
            env_closed_ ? "CHIUSO" : "APERTO", 100.0 * up_ratio_avg_);
      }

      static rclcpp::Clock throttle(RCL_ROS_TIME);
      RCLCPP_INFO_THROTTLE(this->get_logger(), throttle, 2000,
          "[EnvSense] ritorni per elevazione: ALTO(>30) %.1f%%  MEDIO %.1f%%  BASSO(<-30) %.1f%% "
          "-> %s",
          100.0 * up_ok / std::max(1, up_tot), 100.0 * mid_ok / std::max(1, mid_tot),
          100.0 * dn_ok / std::max(1, dn_tot), env_closed_ ? "CHIUSO" : "APERTO");
    }

    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
    cloud_out.is_dense = true;
    cloud_out.header.frame_id = "world";

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(cloud_out, out_msg);
    out_msg.header.stamp = this->now();
    cloud_pub_->publish(out_msg);
    // Nuvola per la MAPPA 3D (registra_mappa.py): senza nessun compagno, fermo o in moto.
    // Con un drone solo e' la stessa.
    if (compagni_.empty()) {
      cloud_mappa3d_pub_->publish(out_msg);
    } else {
      cloud_mappa3d.width = cloud_mappa3d.points.size();
      cloud_mappa3d.height = 1;
      cloud_mappa3d.is_dense = true;
      cloud_mappa3d.header.frame_id = "world";
      sensor_msgs::msg::PointCloud2 m3d;
      pcl::toROSMsg(cloud_mappa3d, m3d);
      m3d.header.stamp = out_msg.header.stamp;
      cloud_mappa3d_pub_->publish(m3d);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, cloud_mappa3d_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  std::string lio_topic_;
  bool latency_comp_ = true;
  Eigen::Quaterniond spawn_rot_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d spawn_trans_ = Eigen::Vector3d::Zero();
  std::deque<Posa> storico_;
  std::deque<Posa> lio_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> in_attesa_;
  rclcpp::Time ultimo_clock_{ 0, 0, RCL_ROS_TIME };
  bool have_clock_ = false;
  double eta_media_ = -1.0;
  uint64_t scartate_ = 0;
  uint64_t interpolate_ = 0;
  nav_msgs::msg::Odometry last_odom_;
  bool have_odom_;
  Eigen::Vector3d lidar_offset_body_ = Eigen::Vector3d(0.0, 0.0, 0.25);
  double min_range_ = 0.5;
  // Compagni di sciame (vedi il costruttore): ultima odometria in coordinate mondo.
  struct Compagno {
    rclcpp::Time t{ 0, 0, RCL_ROS_TIME };
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    bool valido = false;
  };
  int drone_id_ = 1;
  std::map<int, Compagno> compagni_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> compagni_subs_;
  double raggio_compagni_ = 0.6, compagni_giu_ = 0.45, compagni_su_ = 0.55;
  double velocita_compagni_ = 0.3;
  uint64_t tolti_compagni_ = 0;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr env_pub_;
  double env_closed_enter_ = 0.60, env_open_enter_ = 0.40;
  double up_ratio_avg_ = -1.0;
  bool env_closed_ = true;   // si parte assumendo chiuso: e' il caso conservativo
  bool env_published_ = false;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudTransformBridge>());
  rclcpp::shutdown();
  return 0;
}
