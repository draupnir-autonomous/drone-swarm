/**
 * Odometry direction of the PX4<->C2-Explorer bridge: subscribes PX4's
 * px4_msgs/VehicleOdometry (NED/FRD, published on /fmu/out/vehicle_odometry)
 * and republishes it as nav_msgs/Odometry (ENU/FLU, on /odom_world) using
 * px4_ros_com's frame_transforms helpers, matching the topic name
 * plan_env/map_ros.cpp subscribes to (odom_world, synced with the lidar
 * point cloud).
 *
 * Multi-vehicle: PX4 namespaces instance i>0 as /px4_<i>/fmu/... (see
 * ROMFS/px4fmu_common/init.d-posix/rcS), so the PX4 side is prefixed with the
 * px4_ns parameter while the ROS side stays relative and picks up this node's
 * own /drone_N namespace.
 */
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class OdometryBridge : public rclcpp::Node {
public:
  OdometryBridge() : Node("c2x_odometry_bridge") {
    const std::string px4_ns = this->declare_parameter<std::string>("px4_ns", "");

    // PX4's local position is expressed in an EKF frame whose origin sits at
    // the vehicle's own spawn point, so with several vehicles every one of them
    // reports itself near (0,0,0) no matter where it was actually spawned.
    // Adding back the spawn pose puts all drones into the one shared world
    // frame the exploration stack assumes (map voxels, task centers, peer
    // trajectories and the r_comm distance check are all expressed in it).
    // These must match the PX4_GZ_MODEL_POSE the vehicle was spawned with.
    spawn_x_ = this->declare_parameter<double>("spawn_x", 0.0);
    spawn_y_ = this->declare_parameter<double>("spawn_y", 0.0);
    spawn_z_ = this->declare_parameter<double>("spawn_z", 0.0);
    spawn_yaw_ = this->declare_parameter<double>("spawn_yaw", 0.0);
    spawn_rot_ = Eigen::AngleAxisd(spawn_yaw_, Eigen::Vector3d::UnitZ());

    // SORGENTE DELL'ODOMETRIA: autopilota oppure odometria lidar-inerziale.
    //
    // "px4" prende la stima dell'EKF di bordo, che in simulazione riceve un GPS
    // quasi perfetto - comodo per lo sviluppo, ma e' proprio cio' che in un
    // edificio senza GPS non esiste. "lio" prende la stima di FAST-LIO, che
    // ricava la posa da lidar e IMU e quindi funziona anche al chiuso.
    // La trasformazione da applicare e' la STESSA nei due casi: entrambe le
    // stime nascono nella posa iniziale del velivolo, quindi diventano
    // coordinate di mondo con la stessa rototraslazione di spawn. E' il motivo
    // per cui questo ponte puo' ospitare tutte e due senza duplicare la logica.
    spawn_trans_ = Eigen::Vector3d(spawn_x_, spawn_y_, spawn_z_);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_world", 25);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    px4_topic_ = px4_ns + "/fmu/out/vehicle_odometry";
    sorgente_ = this->declare_parameter<std::string>("odom_source", "px4");
    const std::string lio_topic = this->declare_parameter<std::string>("lio_topic", "/Odometry");

    if (sorgente_ == "lio") {
      // Coda profonda e affidabile: FAST-LIO pubblica a 10 Hz e ogni messaggio
      // conta, a differenza della raffica di PX4 a 100 Hz dove perderne uno e'
      // irrilevante.
      lio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          lio_topic, rclcpp::QoS(rclcpp::KeepLast(20)),
          [this](const nav_msgs::msg::Odometry::UniquePtr msg) { lioCallback(*msg); });
      RCLCPP_WARN(this->get_logger(),
          "odometria da FAST-LIO (%s): l'autopilota NON e' la sorgente di posa",
          lio_topic.c_str());
    } else {
      vehicle_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
          px4_topic_, qos,
          [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) { odomCallback(*msg); });
    }

    // Diagnostica del collegamento con l'autopilota.
    //
    // Circa una run su tre parte con un drone che ripete "no odom" per sempre,
    // mentre gli altri due volano. Le prove raccolte: le tre istanze PX4 sono
    // identiche (tutte creano il writer dell'odometria, tutte si armano, quella
    // "guasta" decolla pure) e tutti e tre i ponti sono avviati. Il guasto sta
    // quindi nell'ACCOPPIAMENTO fra publisher e subscriber, che in ROS2 avviene
    // per scoperta dinamica e dipende dai tempi: sotto carico qualche coppia non
    // si forma. Questo nodo non lo diceva perche' in funzionamento normale non
    // stampa nulla - un silenzio che vale sia per il caso sano sia per quello
    // rotto, e che mi ha quasi portato a incolpare il ponte sbagliato.
    // Qui si distingue il caso "nessuno pubblica sul topic" da "pubblica ma non
    // arriva": il primo e' un problema a monte, il secondo e' la scoperta.
    link_timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
      if (odom_count_ > 0) {
        if (!link_ok_) {
          link_ok_ = true;
          RCLCPP_INFO(this->get_logger(), "[Odom] collegamento con l'autopilota attivo (%s)",
              px4_topic_.c_str());
        }
        return;
      }
      const size_t publishers = this->count_publishers(px4_topic_);
      RCLCPP_WARN(this->get_logger(),
          "[Odom] nessun dato dall'autopilota dopo %.0fs: %zu publisher visti su %s - %s",
          (this->now() - start_time_).seconds(), publishers, px4_topic_.c_str(),
          publishers == 0 ? "nessuno pubblica (problema a monte: PX4 o agente XRCE)"
                          : "qualcuno pubblica ma non arriva (accoppiamento DDS non riuscito)");
    });
  }

private:
  // Odometria lidar-inerziale -> stesso argomento e stessa convenzione del ramo
  // PX4, cosi' nulla a valle si accorge del cambio di sorgente.
  //
  // Si rimarca con this->now(), esattamente come fa il ramo PX4 poco sotto e per
  // la stessa ragione - tenere un solo dominio temporale a valle. Con tempo_sim
  // (di serie in simulazione dall'ottobre 2026, vedi il launch) this->now() e' gia'
  // il tempo di Gazebo come quello di FAST-LIO; con tempo_sim:=false e' l'orologio
  // di parete del resto dello stack, e questa era l'unica conversione di dominio.
  void lioCallback(const nav_msgs::msg::Odometry& lio) {
    ++odom_count_;
    const auto& p = lio.pose.pose.position;
    const auto& q = lio.pose.pose.orientation;
    const auto& v = lio.twist.twist.linear;

    Eigen::Vector3d pos(p.x, p.y, p.z);
    Eigen::Vector3d vel(v.x, v.y, v.z);
    Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);

    // VELOCITA' DERIVATA DALLE POSIZIONI: FAST-LIO non la pubblica.
    //
    // Il campo twist esiste nel messaggio ma resta a zero - misurato: modulo
    // mediano 0.0000 m/s mentre il velivolo si muoveva davvero a 0.23 m/s
    // (0.84 di picco), dedotto dalle posizioni successive.
    // Non e' un dettaglio cosmetico: l'FSM usa odom_vel_ come STATO INIZIALE
    // della pianificazione, quindi con velocita' nulla ogni traiettoria verrebbe
    // pianificata come se il drone fosse fermo, mentre e' in movimento. Il piano
    // partirebbe sistematicamente da uno stato sbagliato.
    // Si deriva quindi per differenze finite sulle pose consecutive, con un
    // filtro esponenziale leggero perche' derivare amplifica il rumore: a 10 Hz
    // un centimetro di incertezza sulla posa diventa 0.1 m/s di velocita'.
    // Il dt si prende dai TEMPI DEI MESSAGGI, non da this->now().
    //
    // FAST-LIO gira a tempo di simulazione e questo nodo sull'orologio di
    // parete: misurando l'intervallo con l'orologio sbagliato si ottengono
    // velocita' fuori scala di ordini di grandezza - misurate 146-203 m/s per un
    // velivolo che si muove a 0.2 m/s, con l'errore che cresceva a ogni ciclo.
    // I due tempi consecutivi dello stesso flusso stanno invece sempre nello
    // stesso dominio, qualunque esso sia, e la loro differenza e' corretta per
    // costruzione.
    const rclcpp::Time t_msg(lio.header.stamp, RCL_ROS_TIME);
    if (lio_prec_valida_) {
      const double dt = (t_msg - lio_prec_t_).seconds();
      if (dt > 0.01 && dt < 1.0) {
        const Eigen::Vector3d grezza = (pos - lio_prec_pos_) / dt;
        // Rete di sicurezza: una velocita' non fisica indica che qualcosa a
        // monte e' rotto, e passarla al pianificatore come stato iniziale
        // sarebbe peggio che non averla.
        //
        // La soglia era 20 m/s, che non escludeva niente di utile: questo
        // velivolo pianifica sotto 1.5 m/s, quindi 20 lasciava passare valori
        // dieci volte fuori scala - ne e' stato osservato uno di 17.9 m/s
        // arrivare intatto al pianificatore. Il limite ora e' fisico, non
        // simbolico.
        //
        // E quando si scarta NON si tiene per sempre l'ultima buona: se la
        // stima a monte e' andata, quel valore invecchia e resta comunque
        // sbagliato, con l'aggravante di sembrare valido. Lo si fa decadere
        // verso zero, cioe' verso "fermo", che e' l'ipotesi meno dannosa per
        // chi pianifica.
        constexpr double kVelMaxFisica = 3.0;   // m/s
        if (grezza.norm() < kVelMaxFisica) {
          vel = lio_vel_filtrata_ = 0.6 * lio_vel_filtrata_ + 0.4 * grezza;
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "velocita' derivata non fisica (%.1f m/s su dt=%.3fs): scartata. "
              "Se persiste, la stima di FAST-LIO sta divergendo.",
              grezza.norm(), dt);
          vel = lio_vel_filtrata_ = 0.7 * lio_vel_filtrata_;
        }
      } else {
        vel = lio_vel_filtrata_;
      }
    }
    lio_prec_pos_ = pos;
    lio_prec_t_ = t_msg;
    lio_prec_valida_ = true;

    // FAST-LIO stima gia' in ENU nel frame iniziale del velivolo: non serve la
    // conversione NED->ENU del ramo PX4, ma serve la stessa rototraslazione di
    // spawn per passare al mondo condiviso.
    pos = spawn_rot_ * pos + spawn_trans_;
    vel = spawn_rot_ * vel;
    rot = spawn_rot_ * rot;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    odom.pose.pose.orientation.w = rot.w();
    odom.pose.pose.orientation.x = rot.x();
    odom.pose.pose.orientation.y = rot.y();
    odom.pose.pose.orientation.z = rot.z();
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();
    odom_pub_->publish(odom);
  }

  void odomCallback(const px4_msgs::msg::VehicleOdometry& px4_odom) {
    ++odom_count_;
    Eigen::Vector3d pos_ned(px4_odom.position[0], px4_odom.position[1], px4_odom.position[2]);
    Eigen::Vector3d vel_ned(px4_odom.velocity[0], px4_odom.velocity[1], px4_odom.velocity[2]);
    Eigen::Quaterniond q_px4(
        px4_odom.q[0], px4_odom.q[1], px4_odom.q[2], px4_odom.q[3]);  // w, x, y, z

    Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);
    Eigen::Vector3d vel_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_ned);
    Eigen::Quaterniond q_ros = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_px4);

    // Vehicle-local ENU -> shared world ENU (identity when spawn_* are 0, i.e.
    // the single-drone-at-the-origin case).
    pos_enu = spawn_rot_ * pos_enu + spawn_trans_;
    vel_enu = spawn_rot_ * vel_enu;
    q_ros = spawn_rot_ * q_ros;

    nav_msgs::msg::Odometry odom;
    // Wall-clock stamp (this->now()), not PX4's own timestamp field: it
    // turns out not to be a clean Gazebo-sim-time value we can compare
    // against Gazebo-bridged topics either. Every node in this bridge layer
    // stamps with wall-clock consistently instead, so any of our own
    // cross-topic sync stays within one time domain.
    odom.header.stamp = this->now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = pos_enu.x();
    odom.pose.pose.position.y = pos_enu.y();
    odom.pose.pose.position.z = pos_enu.z();
    odom.pose.pose.orientation.w = q_ros.w();
    odom.pose.pose.orientation.x = q_ros.x();
    odom.pose.pose.orientation.y = q_ros.y();
    odom.pose.pose.orientation.z = q_ros.z();

    odom.twist.twist.linear.x = vel_enu.x();
    odom.twist.twist.linear.y = vel_enu.y();
    odom.twist.twist.linear.z = vel_enu.z();

    odom_pub_->publish(odom);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;
  rclcpp::TimerBase::SharedPtr link_timer_;
  std::string px4_topic_;
  uint64_t odom_count_ = 0;
  bool link_ok_ = false;
  rclcpp::Time start_time_{ this->now() };

  std::string sorgente_ = "px4";
  // Stato per derivare la velocita' dalle pose di FAST-LIO (vedi lioCallback).
  bool lio_prec_valida_ = false;
  Eigen::Vector3d lio_prec_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d lio_vel_filtrata_ = Eigen::Vector3d::Zero();
  rclcpp::Time lio_prec_t_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
  double spawn_x_ = 0.0, spawn_y_ = 0.0, spawn_z_ = 0.0, spawn_yaw_ = 0.0;
  Eigen::Quaterniond spawn_rot_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d spawn_trans_ = Eigen::Vector3d::Zero();
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
