/**
 * Adatta la nuvola del lidar simulato al formato che FAST-LIO si aspetta.
 *
 * Il gpu_lidar di Gazebo pubblica una nuvola ORGANIZZATA 64x300, vicina alla
 * struttura velodyne_ros::Point che FAST-LIO usa nel percorso VELO16 ma non
 * identica: mancano "time", cioe' l'istante di ciascun punto dentro la
 * scansione, e - a seconda della versione di gz-sensors - anche "ring".
 *
 * Il campo deve esserci, altrimenti pcl::fromROSMsg lascia quel membro non
 * inizializzato e il confronto "> 0" decide su spazzatura. Ma il VALORE conta
 * quanto la presenza, e per una ragione che e' costata cara.
 *
 * In preprocess.cpp FAST-LIO sceglie cosi':
 *
 *     if (pl_orig.points[plsize-1].time > 0) given_offset_time = true;
 *     else                                   given_offset_time = false;
 *
 * Con "time" a zero si finisce nel ramo false, dove FAST-LIO SI INVENTA gli
 * scostamenti temporali dall'angolo di imbardata, distribuendoli su un intero
 * periodo di scansione (~100 ms). Ha senso per un Velodyne vero, che ruota e
 * campiona i punti uno dopo l'altro. NON ha senso qui: il gpu_lidar di Gazebo
 * genera tutti i 300x64 punti in un UNICO istante, con una singola passata di
 * rendering. Gli scostamenti veri sono tutti nulli.
 *
 * La conseguenza e' che FAST-LIO chiede alla IMU di correggere 100 ms di moto
 * che non e' mai avvenuto durante la scansione. Da fermi o in salita verticale
 * l'effetto e' trascurabile; appena il velivolo imbarda a ~1 rad/s la nuvola
 * viene ruotata di qualche grado a sproposito, i residui esplodono e il filtro
 * diverge. E' esattamente quello che si e' misurato: stima perfetta fino a
 * 14.9 s, poi posizione in fuga lineare a 15 m/s con il drone quasi fermo.
 *
 * Quindi si scrive un valore POSITIVO ma trascurabile, uguale per tutti i
 * punti: positivo perche' attiva il ramo given_offset_time = true e disinnesca
 * l'invenzione; uguale per tutti perche' la de-distorsione diventa una
 * trasformazione unica, cioe' l'identita' relativa - che e' la fisica giusta
 * per una cattura istantanea.
 *
 * Quindi qui si copia la nuvola aggiungendo "time". E' l'intervento
 * minimo: nessun ricampionamento, nessuna trasformazione di sistema di
 * riferimento. La nuvola resta nel frame del sensore, che e' quello che una
 * odometria lidar-inerziale deve ricevere - a differenza di map_ros/cloud, gia'
 * portata in sistema mondo usando l'odometria di PX4, cioe' proprio la stima
 * che vogliamo sostituire.
 */
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstring>
#include <cmath>

#include <map>
#include <string>
#include <vector>

class LidarLioAdapter : public rclcpp::Node {
public:
  LidarLioAdapter() : Node("lidar_lio_adapter") {
    ingresso_ = this->declare_parameter<std::string>("cloud_in", "lidar/points");
    uscita_ = this->declare_parameter<std::string>("cloud_out", "lidar/points_lio");

    // Best effort come il resto della catena sensori: una nuvola persa non va
    // ritrasmessa, ne arriva un'altra fra 100 ms.
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(uscita_, qos);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        ingresso_, qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { onCloud(*msg); });

    RCLCPP_INFO(this->get_logger(), "adattatore LIO: %s -> %s (aggiunge il campo 'time')",
        ingresso_.c_str(), uscita_.c_str());

    // I COMPAGNI FUORI DA FAST-LIO (sciame, ottobre 2026). Gli altri droni nelle scansioni
    // sono oggetti in moto: FAST-LIO li inserisce nella sua mappa interna e ci si aggancia.
    // Misurato a tre droni su office_small: errore di posizione medio 0.20-0.33 m con un
    // compagno entro 2 m, contro 0.11-0.14 m altrimenti. Si tolgono i punti dentro lo stesso
    // cilindro di cloud_transform_bridge attorno a ogni compagno, portato in terna sensore con
    // la posa di questo drone (odom_world, istanti dei messaggi, nessun orologio). Con un
    // drone solo non si iscrive a nulla.
    drone_id_ = this->declare_parameter<int>("drone_id", 1);
    const int drone_num = this->declare_parameter<int>("drone_num", 1);
    raggio_ = this->declare_parameter<double>("peer_filter_radius", 0.6);
    giu_ = this->declare_parameter<double>("peer_filter_below", 0.45);
    su_ = this->declare_parameter<double>("peer_filter_above", 0.55);
    const auto off = this->declare_parameter<std::vector<double>>(
        "lidar_offset_body", std::vector<double>{ 0.0, 0.0, 0.25 });
    if (off.size() == 3) offset_ = Eigen::Vector3d(off[0], off[1], off[2]);
    auto iscrivi = [this](const std::string& topic, Stato& st) {
      return this->create_subscription<nav_msgs::msg::Odometry>(topic,
          rclcpp::QoS(rclcpp::KeepLast(5)), [&st](const nav_msgs::msg::Odometry::ConstSharedPtr m) {
            st.t = rclcpp::Time(m->header.stamp, RCL_ROS_TIME);
            st.p = Eigen::Vector3d(m->pose.pose.position.x, m->pose.pose.position.y,
                m->pose.pose.position.z);
            st.v = Eigen::Vector3d(m->twist.twist.linear.x, m->twist.twist.linear.y,
                m->twist.twist.linear.z);
            st.q = Eigen::Quaterniond(m->pose.pose.orientation.w, m->pose.pose.orientation.x,
                m->pose.pose.orientation.y, m->pose.pose.orientation.z).normalized();
            st.valido = true;
          });
    };
    if (drone_num > 1) {
      odom_subs_.push_back(iscrivi("odom_world", io_));
      for (int k = 1; k <= drone_num; ++k)
        if (k != drone_id_) compagni_[k] = Stato();
      for (auto& kv : compagni_)
        odom_subs_.push_back(iscrivi("/drone_" + std::to_string(kv.first) + "/odom_world", kv.second));
    }
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2& in) {
    if (!verificato_) verificaCampi(in);

    sensor_msgs::msg::PointCloud2 out;
    out.header = in.header;
    out.is_bigendian = in.is_bigendian;

    // I campi originali restano ai loro offset: si accodano in fondo quelli che
    // mancano, cosi' chi legge per nome (pcl::fromROSMsg lo fa) trova tutto dove
    // si aspetta.
    out.fields = in.fields;
    uint32_t passo = in.point_step;

    sensor_msgs::msg::PointField campo_tempo;
    campo_tempo.name = "time";
    campo_tempo.offset = passo;
    campo_tempo.datatype = sensor_msgs::msg::PointField::FLOAT32;
    campo_tempo.count = 1;
    out.fields.push_back(campo_tempo);
    const uint32_t off_tempo = passo;
    passo += sizeof(float);

    // "ring" ha lo stesso identico problema di "time", e conseguenze peggiori.
    // velodyne_ros::Point lo dichiara, quindi se la nuvola non lo porta
    // pcl::fromROSMsg lascia quel membro NON INIZIALIZZATO. FAST-LIO lo usa poi
    // come INDICE nei vettori per anello del velodyne_handler (yaw_fp[layer],
    // time_last[layer]): un indice casuale non solo sbaglia la de-distorsione,
    // legge fuori dagli estremi. E l'errore di de-distorsione e' proporzionale
    // alla velocita', quindi il difetto e' invisibile in stazionario e cresce
    // col moto - esattamente il comportamento osservato in anello chiuso.
    // Se la sorgente lo fornisce si lascia il suo; se manca lo si SINTETIZZA,
    // cosa che qui e' esatta e non un ripiego: la nuvola e' organizzata, e in
    // una nuvola organizzata l'anello e' per definizione l'indice di riga.
    const uint32_t off_anello = passo;
    if (!ring_presente_) {
      sensor_msgs::msg::PointField campo_anello;
      campo_anello.name = "ring";
      campo_anello.offset = off_anello;
      campo_anello.datatype = sensor_msgs::msg::PointField::UINT16;
      campo_anello.count = 1;
      out.fields.push_back(campo_anello);
      passo += sizeof(uint16_t);
    }

    out.point_step = passo;
    const size_t n = static_cast<size_t>(in.width) * in.height;
    out.data.resize(n * out.point_step);
    // Un microsecondo: positivo quanto basta a superare il test "> 0" di
    // preprocess.cpp, e cinque ordini di grandezza sotto il periodo di
    // scansione, quindi ininfluente sulla de-distorsione. Vedi la nota in testa
    // al file: il valore non e' arbitrario, e' un sentinella.
    const float tempo_punto = 1e-6f;
    const uint32_t larghezza = in.width > 0 ? in.width : 1;
    // SOLO PUNTI FINITI (18 ottobre 2026). I raggi che non colpiscono nulla - il
    // cielo, all'aperto - il gpu_lidar li restituisce a coordinate INFINITE (2850
    // su 19200 nel parco). Il preprocessore di FAST-LIO li lascia passare (controlla
    // solo la distanza cieca, e "infinito > 0.5 m" e' vero) e poi il filtro a voxel
    // di PCL calcola il riquadro della nuvola con dentro gli infiniti: i voxel
    // collassano e da ~6200 punti ne uscivano ~273, ciascuno la MEDIA di superfici
    // diverse. FAST-LIO si agganciava a una geometria inventata e derivava entro
    // 20-60 s (3 urti su 3 nel parco). In ufficio il soffitto restituisce ogni
    // raggio, non ci sono infiniti, ed e' per questo che li' funzionava. Si scartano
    // qui, prima di FAST-LIO; l'anello si calcola dalla riga ORIGINALE, quindi la
    // nuvola puo' uscire non organizzata (FAST-LIO usa il campo ring, non la forma).
    const auto off_campo = [&in](const char* nome) -> int {
      for (const auto& f : in.fields)
        if (f.name == nome && f.datatype == sensor_msgs::msg::PointField::FLOAT32)
          return static_cast<int>(f.offset);
      return -1;
    };
    const int ox = off_campo("x"), oy = off_campo("y"), oz = off_campo("z");
    // Compagni in terna sensore all'istante della scansione (vedi il costruttore).
    std::vector<Eigen::Vector3d> compagni_s;
    Eigen::Matrix3d r_ws = Eigen::Matrix3d::Identity();
    if (io_.valido && !compagni_.empty()) {
      const rclcpp::Time tc(in.header.stamp, RCL_ROS_TIME);
      const double dt_io = std::clamp((tc - io_.t).seconds(), -0.3, 0.3);
      r_ws = io_.q.toRotationMatrix();
      const Eigen::Vector3d t_sensore = io_.p + io_.v * dt_io + r_ws * offset_;
      for (const auto& kv : compagni_) {
        const Stato& c = kv.second;
        if (!c.valido) continue;
        const double dt = (tc - c.t).seconds();
        if (std::abs(dt) > 1.0) continue;
        const Eigen::Vector3d p = c.p + c.v * std::clamp(dt, -0.3, 0.3);
        compagni_s.push_back(r_ws.transpose() * (p - t_sensore));
      }
    }
    const double r2 = raggio_ * raggio_;
    size_t validi = 0, tolti = 0;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t* src = &in.data[i * in.point_step];
      if (ox >= 0 && oy >= 0 && oz >= 0) {
        float x, y, z;
        std::memcpy(&x, src + ox, sizeof(float));
        std::memcpy(&y, src + oy, sizeof(float));
        std::memcpy(&z, src + oz, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        bool su_un_compagno = false;
        for (const auto& c : compagni_s) {
          const Eigen::Vector3d d = r_ws * (Eigen::Vector3d(x, y, z) - c);   // in terna mondo
          if (d.z() < -giu_ || d.z() > su_) continue;
          if (d.head<2>().squaredNorm() < r2) {
            su_un_compagno = true;
            break;
          }
        }
        if (su_un_compagno) {
          ++tolti;
          continue;
        }
      }
      uint8_t* dst = &out.data[validi * out.point_step];
      std::memcpy(dst, src, in.point_step);
      std::memcpy(dst + off_tempo, &tempo_punto, sizeof(float));
      if (!ring_presente_) {
        const uint16_t anello = static_cast<uint16_t>(i / larghezza);
        std::memcpy(dst + off_anello, &anello, sizeof(uint16_t));
      }
      ++validi;
    }
    out.data.resize(validi * out.point_step);
    out.height = 1;
    out.width = static_cast<uint32_t>(validi);
    out.row_step = out.point_step * out.width;
    out.is_dense = true;
    scartati_ += n - validi - tolti;
    tolti_compagni_ += tolti;
    if (++scansioni_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(),
          "punti non finiti scartati: %.0f per scansione (ultime 100); punti sui compagni tolti %.1f",
          static_cast<double>(scartati_) / 100.0, static_cast<double>(tolti_compagni_) / 100.0);
      scartati_ = 0;
      tolti_compagni_ = 0;
    }
    pub_->publish(out);
  }

  // Si controlla UNA volta che la nuvola in ingresso sia quella attesa, e si
  // dice esattamente cosa manca. Senza questo, un cambio nel modello del sensore
  // si manifesterebbe come una stima di posa che diverge, e la causa andrebbe
  // cercata a valle di tutta la catena.
  void verificaCampi(const sensor_msgs::msg::PointCloud2& in) {
    verificato_ = true;
    bool intensity = false, tempo_gia_presente = false;
    for (const auto& f : in.fields) {
      if (f.name == "ring") ring_presente_ = true;
      if (f.name == "intensity") intensity = true;
      if (f.name == "time") tempo_gia_presente = true;
    }
    RCLCPP_INFO(this->get_logger(), "nuvola in ingresso: %ux%u punti, passo %u B, campi:%s%s%s",
        in.width, in.height, in.point_step, intensity ? " intensity" : "",
        ring_presente_ ? " ring" : "", tempo_gia_presente ? " time" : "");
    if (!ring_presente_) {
      RCLCPP_WARN(this->get_logger(),
          "la sorgente non fornisce 'ring': viene sintetizzato dall'indice di riga "
          "(nuvola organizzata, %u anelli da %u punti). Senza, FAST-LIO userebbe un "
          "indice non inizializzato per la de-distorsione.", in.height, in.width);
    }
    if (tempo_gia_presente) {
      RCLCPP_WARN(this->get_logger(),
          "la nuvola ha gia' un campo 'time': questo adattatore ne aggiunge un secondo. "
          "Se la sorgente e' cambiata, l'adattatore non serve piu'.");
    }
    if (in.height <= 1) {
      RCLCPP_WARN(this->get_logger(),
          "nuvola non organizzata (height=%u): la deduzione dei tempi per anello e' meno "
          "affidabile.", in.height);
    }
  }

  struct Stato {
    rclcpp::Time t{ 0, 0, RCL_ROS_TIME };
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    bool valido = false;
  };
  int drone_id_ = 1;
  double raggio_ = 0.6, giu_ = 0.45, su_ = 0.55;
  Eigen::Vector3d offset_ = Eigen::Vector3d(0.0, 0.0, 0.25);
  Stato io_;
  std::map<int, Stato> compagni_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  size_t tolti_compagni_ = 0;
  std::string ingresso_, uscita_;
  bool verificato_ = false;
  size_t scartati_ = 0, scansioni_ = 0;
  bool ring_presente_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLioAdapter>());
  rclcpp::shutdown();
  return 0;
}
