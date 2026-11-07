/**
 * Porta la stima di FAST-LIO dentro l'autopilota come VISIONE ESTERNA.
 *
 * E' il pezzo che rende il volo davvero indipendente dal GPS. Finora FAST-LIO
 * alimentava solo lo STACK DI ESPLORAZIONE - mappa, frontiere, pianificazione -
 * mentre PX4 continuava a stimare la propria posizione con il GPS simulato. In
 * un edificio quel GPS non esiste, quindi l'autopilota resterebbe senza
 * riferimento e non potrebbe nemmeno tenere la quota.
 *
 * Qui l'odometria stimata viene pubblicata su /fmu/in/vehicle_visual_odometry,
 * che EKF2 fonde come misura di visione esterna. Con EKF2_GPS_CTRL a zero e
 * EKF2_EV_CTRL configurato, la posizione dell'autopilota viene interamente da
 * lidar e IMU.
 *
 * Due scelte che vale la pena spiegare:
 *
 * 1. SI MANDA ANCHE LA VELOCITA', derivata dalle pose. Prima si mandava NaN
 *    ("non disponibile") col ragionamento che una velocita' imperfetta fosse
 *    peggio del silenzio, tanto EKF2 la ricava integrando l'IMU fra una misura
 *    di posizione e l'altra. La misura ha smentito quel ragionamento.
 *
 *    Confronto diretto sulla stessa mappa, stesso velivolo, stesso controllore:
 *
 *      col GPS (EKF2_GPS_CTRL=7)  rollio max 10.6 gradi, motori mai in
 *                                 saturazione, 20.8 m percorsi, nessuna caduta
 *      con la visione, senza      rollio fino a 42 gradi, motori in saturazione
 *      velocita'                  a 14.2 s, ribaltamento a 15.0 s
 *
 *    L'unica differenza sostanziale fra le due configurazioni e' che il GPS
 *    porta ANCHE la velocita'. Senza, EKF2 deve integrare l'IMU per 100 ms fra
 *    un aggiornamento e l'altro, con 150 ms di latenza dichiarata: la velocita'
 *    stimata arriva al controllore di posizione in ritardo e a scatti, e il
 *    termine di smorzamento che ne dipende manda l'anello in oscillazione
 *    divergente. "Tacere" non e' neutro: lascia il controllore senza
 *    smorzamento.
 *
 *    La derivata delle pose e' ora molto piu' affidabile di quando si scrisse
 *    quella nota, perche' nel frattempo e' stato corretto il difetto di
 *    de-distorsione nell'adattatore della nuvola (vedi lidar_lio_adapter.cpp).
 *
 * 2. SI USA L'ODOMETRIA GREZZA di FAST-LIO, non quella trasformata in
 *    coordinate di mondo. L'origine di FAST-LIO coincide con la posa del drone
 *    all'avvio, che e' esattamente l'origine locale di EKF2: le due convenzioni
 *    combaciano gia'. Applicare lo scostamento di spawn qui introdurrebbe un
 *    salto pari alla posizione di partenza.
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

class LioToPx4Bridge : public rclcpp::Node {
public:
  LioToPx4Bridge() : Node("lio_to_px4_bridge") {
    const std::string px4_ns = this->declare_parameter<std::string>("px4_ns", "");
    const std::string lio_topic = this->declare_parameter<std::string>("lio_topic", "/Odometry");
    // Incertezza dichiarata a EKF2: 0.05 m, coerente con la precisione VERA di
    // FAST-LIO misurata contro la verita' di Gazebo (2-9 cm in volo).
    //
    // Questo numero ha una storia che vale la pena non ripetere. Era 0.10; l'ho
    // portato a 0.25 per far passare il test di consistenza (sbagliato: il
    // filtro accettava le misure e poi le ignorava), poi di nuovo a 0.10. Ma a
    // 0.10 il filtro restava comunque lento, e il motivo si vede solo misurando
    // il guadagno di Kalman nel registro di volo:
    //
    //   P (fiducia del filtro in se') = 0.0034 m^2  -> sigma 5.8 cm
    //   R (fiducia nella misura)      = 0.0100 m^2  -> sigma 10 cm
    //   guadagno = P/(P+R)            = 0.255
    //
    // Con un guadagno di 0.25 e misure a 10 Hz, la costante di tempo con cui il
    // filtro recupera un errore vale 1/(10*0.255) = 0.39 s: sono esattamente i
    // 0.4 s di "ritardo" misurati contro la verita'. Non era latenza di
    // trasporto - quella e' 13 ms - era il filtro che si fidava piu' di se'
    // stesso che del lidar, pur avendo in mano una misura tre volte migliore di
    // come la dichiaravamo.
    //
    // ATTENZIONE: da solo questo valore non basta. EKF2 impone un pavimento con
    // EKF2_EVP_NOISE, quindi finche' quello resta a 0.10 dichiarare meno qui non
    // ha alcun effetto. I due vanno cambiati INSIEME (vedi l'airframe 22000).
    pos_std_ = this->declare_parameter<double>("position_std", 0.05);
    att_std_ = this->declare_parameter<double>("orientation_std", 0.05);
    // Incertezza della velocita' DERIVATA: 0.30 m/s.
    //
    // PROVATO E SCARTATO 0.10 (insieme a un filtro piu' reattivo, 0.3/0.7).
    // L'idea era la stessa che sulla posizione aveva funzionato: dichiarare la
    // misura per quello che vale invece di sottostimarla. Sulla velocita' pero'
    // ha peggiorato tutto, su 4 run per configurazione:
    //
    //                      errore_pos   latenza   guadagno
    //   vel_std 0.30         0.470 m    0.309 s    0.281
    //   vel_std 0.10         0.996 m    0.516 s    0.165
    //
    // Il guadagno di Kalman sulla POSIZIONE e' crollato, ed e' la grandezza piu'
    // pulita che misuriamo (escursione 0.003 fra run identiche, quindi la
    // differenza e' certa). Fondendo la velocita' con piu' forza il filtro
    // diventa piu' sicuro di se' NEL COMPLESSO: P scende, e con essa il
    // guadagno sulla posizione. Le due misure non sono indipendenti, e stringere
    // l'una allenta l'altra.
    vel_std_ = this->declare_parameter<double>("velocity_std", 0.30);
    // RILEVATORE DI DIVERGENZA. Il controllo di plausibilita' qui sotto
    // escludeva i campioni anomali dalla media ma continuava a mandare a PX4
    // POSIZIONI che, a stima divergente, sono spazzatura: nella run BASE_LIO 3,
    // dopo un urto, la stima e' partita (errore di quota fino a 256 m) e il
    // velivolo ha inseguito riferimenti a 5.8 m di quota. Con divergence_samples
    // campioni di fila oltre divergence_speed (ben sopra cio' che PX4 consente:
    // MPC_XY_VEL_MAX 1.5) la stima si considera persa: si smette di mandarla,
    // si chiede l'atterraggio e si annuncia la fine del volo all'FSM.
    divergence_speed_ = this->declare_parameter<double>("divergence_speed", 5.0);
    divergence_samples_ = this->declare_parameter<int>("divergence_samples", 3);
    land_pub_ = this->create_publisher<std_msgs::msg::Empty>("land_request", 1);
    flight_terminated_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "flight_terminated", rclcpp::QoS(1).transient_local());

    // SALUTE DELLA STIMA: il rilevatore qui sopra vede la divergenza solo quando
    // la stima e' gia' partita, cioe' di solito DOPO l'urto. Nel parco (17-18
    // ottobre 2026) la causa era a monte e misurabile da subito: i raggi verso
    // il cielo (+inf) facevano collassare il filtro a voxel e FAST-LIO agganciava
    // a un piano della mappa 0-239 punti per scansione (mediana 141) invece di
    // ~2800 (vedi lidar_lio_adapter.cpp); la stima derivava piano, a velocita'
    // plausibili, fino a portare il drone contro un ostacolo o a terra. Qui si
    // legge a ogni scansione quanti punti sono stati agganciati: se restano sotto
    // salute_min_agganciati per salute_scansioni di fila si chiede l'atterraggio
    // sul posto e si chiude la missione. La stima si continua a mandare a PX4:
    // e' degradata, non persa, e per scendere in verticale e' meglio di nessuna.
    // A terra la stessa regola impedisce il decollo (nel parco col difetto
    // scattava a 0.4 s dall'avvio di FAST-LIO).
    //
    // Soglia 500: sotto il massimo misurato col difetto (239) c'e' un fattore 2,
    // sopra il minimo in volo sano un fattore 3.6 (ufficio 1795, parco 1972).
    // NON si usa l'autovalore minimo l1, che pure FAST-LIO pubblica: in ufficio,
    // dove la stima e' buona, l1 sta a 82-90 per minuti (una direzione vincolata
    // da pochi piani), troppo vicino al 28 mediano del parco guasto; la prima
    // versione con soglia l1 100 ha fatto atterrare un volo sano al decollo.
    // Numeri completi nel LEGGIMI ("Scenario parco"). 0 disattiva la regola.
    salute_min_agganciati_ = this->declare_parameter<double>("salute_min_agganciati", 500.0);
    salute_scansioni_ = this->declare_parameter<int>("salute_scansioni", 5);
    const std::string salute_topic = this->declare_parameter<std::string>("salute_topic", "");
    if (!salute_topic.empty()) {
      salute_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
          salute_topic, rclcpp::QoS(rclcpp::KeepLast(20)),
          [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr m) { onSalute(*m); });
    }

    ev_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_ns + "/fmu/in/vehicle_visual_odometry", 10);

    lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_topic, rclcpp::QoS(rclcpp::KeepLast(20)),
        [this](const nav_msgs::msg::Odometry::UniquePtr msg) { onLio(*msg); });

    diag_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
      if (inviati_ == 0) {
        RCLCPP_WARN(this->get_logger(),
            "nessuna posa inviata all'autopilota: FAST-LIO non sta pubblicando su %s. "
            "Senza queste misure e con il GPS spento, EKF2 resta senza riferimento.",
            lio_sub_->get_topic_name());
      } else {
        RCLCPP_INFO(this->get_logger(), "visione esterna verso PX4: %lu pose inviate", inviati_);
      }
      if (salute_n_ > 0) {
        RCLCPP_INFO(this->get_logger(),
            "salute FAST-LIO (ultimi 5 s, %d scansioni): agganciati min %.0f (soglia %.0f), "
            "l1 min %.0f", salute_n_, salute_agg_min_, salute_min_agganciati_, salute_l1_min_);
        salute_n_ = 0;
        salute_l1_min_ = salute_agg_min_ = std::numeric_limits<double>::infinity();
      }
    });
  }

private:
  void onSalute(const std_msgs::msg::Float32MultiArray& m) {
    if (m.data.size() < 3) return;
    const double agganciati = m.data[1], l1 = m.data[2];
    ++salute_n_;
    salute_l1_min_ = std::min(salute_l1_min_, l1);
    salute_agg_min_ = std::min(salute_agg_min_, agganciati);
    if (degradata_ || divergenza_) return;
    if (agganciati < salute_min_agganciati_) {
      if (++salute_basse_ >= salute_scansioni_) {
        degradata_ = true;
        RCLCPP_ERROR(this->get_logger(),
            "STIMA FAST-LIO DEGRADATA: %d scansioni di fila con meno di %.0f punti agganciati "
            "(ultima: %.0f, l1 %.1f) - chiedo l'atterraggio sul posto e chiudo la missione",
            salute_basse_, salute_min_agganciati_, agganciati, l1);
        land_pub_->publish(std_msgs::msg::Empty());
        flight_terminated_pub_->publish(std_msgs::msg::Empty());
      }
    } else {
      salute_basse_ = 0;
    }
  }

  void onLio(const nav_msgs::msg::Odometry& lio) {
    if (divergenza_) return;  // stima persa: non si manda piu' nulla a PX4
    px4_msgs::msg::VehicleOdometry ev{};

    // PX4 vuole microsecondi dal proprio avvio. Con uXRCE-DDS la convenzione e'
    // passare il tempo ROS in microsecondi: l'agente si occupa dello scarto fra
    // i due orologi.
    const uint64_t ora_us = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
    ev.timestamp = ora_us;
    ev.timestamp_sample = ora_us;

    // ENU (convenzione ROS) -> NED (convenzione PX4): per la POSIZIONE si
    // scambiano x e y e si inverte z. Per l'assetto serve altro, vedi sotto.
    const auto& p = lio.pose.pose.position;
    ev.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    ev.position[0] = static_cast<float>(p.y);
    ev.position[1] = static_cast<float>(p.x);
    ev.position[2] = static_cast<float>(-p.z);

    // ASSETTO: qui NON vale la regola della posizione.
    //
    // Per un VETTORE, passare da ENU a NED significa scambiare x e y e invertire
    // z, ed e' quello che si fa sopra per la posizione. Per un QUATERNIONE DI
    // ASSETTO no: descrive una rotazione fra due terne, e vanno convertite
    // entrambe - il mondo (ENU->NED) e il corpo (FLU->FRD).
    //
    // Avevo applicato al quaternione la regola dei vettori, (w, y, x, -z),
    // scrivendo pure nel commento che valeva per tutti e due. Il risultato e'
    // un errore di imbardata di ESATTAMENTE -90 gradi a qualunque angolo,
    // verificato numericamente contro la composizione corretta.
    //
    // Non e' un difetto cosmetico: il controllore di posizione calcola
    // l'accelerazione in coordinate mondo e la converte in assetto usando
    // l'imbardata stimata. Con 90 gradi di errore il velivolo accelera
    // perpendicolarmente a dove dovrebbe, l'anello di posizione si chiude in
    // positivo e oscilla - misurato: rollio con deviazione standard 12 gradi,
    // picchi a 37, 4.2 inversioni al secondo. Ed e' anche il motivo per cui
    // andava contro i muri: la sua nozione di "avanti" era ruotata.
    //
    // Composizione corretta: q_ned = Q_mondo * q_enu * Q_corpo, con
    //   Q_mondo = 180 gradi attorno a (1,1,0)/sqrt(2)   ENU -> NED
    //   Q_corpo = 180 gradi attorno a X                 FLU -> FRD
    static const double kS = 0.70710678118654752;
    static const Eigen::Quaterniond kQmondo(0.0, kS, kS, 0.0);   // (w,x,y,z)
    static const Eigen::Quaterniond kQcorpo(0.0, 1.0, 0.0, 0.0);
    const auto& q = lio.pose.pose.orientation;
    const Eigen::Quaterniond q_enu(q.w, q.x, q.y, q.z);
    const Eigen::Quaterniond q_ned = kQmondo * q_enu * kQcorpo;
    ev.q[0] = static_cast<float>(q_ned.w());
    ev.q[1] = static_cast<float>(q_ned.x());
    ev.q[2] = static_cast<float>(q_ned.y());
    ev.q[3] = static_cast<float>(q_ned.z());

    // VELOCITA' DERIVATA dalle pose consecutive: vedi la nota 1 in testa al
    // file. Il dt si prende dai tempi dei MESSAGGI, mai dall'orologio locale:
    // FAST-LIO marca al tempo di simulazione e questo nodo potrebbe girare su
    // un orologio diverso, mentre due tempi dello stesso flusso stanno sempre
    // nello stesso dominio.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // UN CAMPIONE SPORCO NON DEVE SPEGNERE LA SORGENTE.
    //
    // Prima, quando la derivata grezza superava la soglia, si mandava NaN. Ma
    // in controlEvVelFusion la condizione di permanenza include
    // ev._sample.vel.isAllFinite(): un solo NaN fa scattare stopEvVelFusion() e
    // BUTTA GIU' tutta la sorgente di aiuto, che poi deve ripartire da zero.
    // Misurato: cs_ev_vel si accendeva a drone fermo e restava attivo l'1.1%
    // del tempo, spegnendosi appena il velivolo si muoveva - perche' in volo la
    // derivata a 10 Hz ha picchi di rumore che sforavano la soglia.
    //
    // Senza aiuto sulla velocita' EKF2 deve dedurla dalla posizione a 10 Hz con
    // 150 ms di ritardo, e sbaglia in grande: misurata una stima di 0.44 m/s
    // mentre il drone viaggiava davvero a 2.02 m/s. Il controllore smorza su
    // quel numero, quindi continua ad accelerare, e quando il filtro recupera
    // di colpo (4.48 m/s) la correzione e' violenta.
    //
    // E' la stessa lezione della nota 1: tacere non e' neutro. Quindi il
    // controllo di plausibilita' ora protegge SOLO l'aggiornamento del filtro -
    // il campione anomalo non entra nella media - mentre in uscita si pubblica
    // sempre la velocita' filtrata, che e' continua e finita per costruzione.
    // La soglia sale a 5 m/s: il velivolo ha toccato 2.3 m/s veri, quindi 3 era
    // troppo vicino al normale, mentre una divergenza vera fa decine di m/s.
    const rclcpp::Time t_msg(lio.header.stamp, RCL_ROS_TIME);
    Eigen::Vector3d p_enu(p.x, p.y, p.z);
    if (prec_valida_) {
      const double dt = (t_msg - prec_t_).seconds();
      if (dt > 0.01 && dt < 1.0) {
        const Eigen::Vector3d grezza = (p_enu - prec_pos_) / dt;
        if (grezza.norm() < divergence_speed_) {
          vel_filtrata_ = 0.6 * vel_filtrata_ + 0.4 * grezza;
          vel_mai_valida_ = false;
          anomalie_consecutive_ = 0;
        } else if (++anomalie_consecutive_ >= divergence_samples_) {
          divergenza_ = true;
          RCLCPP_ERROR(this->get_logger(),
              "DIVERGENZA FAST-LIO: %d campioni di fila oltre %.1f m/s (ultimo %.1f m/s) - "
              "smetto di mandare la stima a PX4, chiedo l'atterraggio",
              anomalie_consecutive_, divergence_speed_, grezza.norm());
          land_pub_->publish(std_msgs::msg::Empty());
          flight_terminated_pub_->publish(std_msgs::msg::Empty());
          return;
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "velocita' derivata non fisica (%.1f m/s): escludo il campione dal filtro, "
              "continuo a pubblicare l'ultima buona", grezza.norm());
        }
      }
    }
    prec_pos_ = p_enu;
    prec_t_ = t_msg;
    prec_valida_ = true;
    const bool vel_valida = !vel_mai_valida_;

    if (vel_valida) {
      // Stessa conversione ENU->NED della posizione.
      ev.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
      ev.velocity = { static_cast<float>(vel_filtrata_.y()),
                      static_cast<float>(vel_filtrata_.x()),
                      static_cast<float>(-vel_filtrata_.z()) };
      const float vvar = static_cast<float>(vel_std_ * vel_std_);
      ev.velocity_variance = { vvar, vvar, vvar };
    } else {
      ev.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
      ev.velocity = { nan, nan, nan };
      ev.velocity_variance = { nan, nan, nan };
    }
    // La velocita' angolare resta non dichiarata: EKF2 la ha dai giroscopi, che
    // sono a 200 Hz e non hanno bisogno di aiuto.
    ev.angular_velocity = { nan, nan, nan };

    const float pvar = static_cast<float>(pos_std_ * pos_std_);
    const float avar = static_cast<float>(att_std_ * att_std_);
    ev.position_variance = { pvar, pvar, pvar };
    ev.orientation_variance = { avar, avar, avar };
    ev.reset_counter = 0;
    ev.quality = 0;

    ev_pub_->publish(ev);
    ++inviati_;
  }

  double pos_std_ = 0.10, att_std_ = 0.05, vel_std_ = 0.30;
  double divergence_speed_ = 5.0;
  int divergence_samples_ = 3;
  int anomalie_consecutive_ = 0;
  bool divergenza_ = false;
  double salute_min_agganciati_ = 500.0;
  int salute_scansioni_ = 5;
  int salute_basse_ = 0;
  bool degradata_ = false;
  int salute_n_ = 0;
  double salute_l1_min_ = std::numeric_limits<double>::infinity();
  double salute_agg_min_ = std::numeric_limits<double>::infinity();
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr salute_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr flight_terminated_pub_;
  bool prec_valida_ = false;
  bool vel_mai_valida_ = true;
  Eigen::Vector3d prec_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_filtrata_ = Eigen::Vector3d::Zero();
  rclcpp::Time prec_t_{ 0, 0, RCL_ROS_TIME };
  uint64_t inviati_ = 0;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr ev_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioToPx4Bridge>());
  rclcpp::shutdown();
  return 0;
}
