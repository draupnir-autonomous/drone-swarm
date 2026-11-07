/**
 * Consumes the exploration stack's c2x_msgs/PositionCommand (published by
 * traj_server on /position_cmd, ENU/FLU) and drives PX4 via offboard control:
 * translates to px4_msgs/TrajectorySetpoint (NED/FRD) using px4_ros_com's
 * frame_transforms, publishes the required OffboardControlMode heartbeat,
 * and owns arming/offboard-mode switching. Honors PositionCommand's
 * trajectory_flag: holds the last valid setpoint instead of forwarding
 * garbage while status is EMPTY, and triggers a landing on ABORT.
 *
 * Also runs an independent reactive safety layer (see onCloud()):
 * the exploration planner's own collision avoidance (checkTrajCollision)
 * only asks whether the *planned* trajectory still avoids the probabilistic
 * occupancy map - it can't see real-time raw sensor data, and a real crash
 * this session happened despite that check because the planner's own
 * callback group can stall for seconds on a cluttered map. This node is a
 * separate process with its own 100Hz timer, so it can't be stalled by
 * exploration_node's planning load; it checks the latest raw lidar cloud
 * directly (not the processed map) every tick and overrides the outgoing
 * setpoint with a hold-in-place if anything gets too close, regardless of
 * what the planner is doing or whether it's even responsive.
 */
#include <c2x_msgs/msg/drone_state.hpp>
#include <c2x_msgs/msg/position_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_set>
#include <vector>

using namespace std::chrono_literals;

class PositionCommandBridge : public rclcpp::Node {
public:
  PositionCommandBridge() : Node("c2x_position_command_bridge") {
    // PX4 SITL instance i is reachable on /px4_<i>/fmu/... for i>0 (bare /fmu
    // for instance 0) and answers vehicle commands addressed to MAV_SYS_ID
    // i+1. Both come from the launch file, which maps drone_id N to instance
    // N-1 / system id N.
    const std::string px4_ns = this->declare_parameter<std::string>("px4_ns", "");
    target_system_ = static_cast<uint8_t>(this->declare_parameter<int>("target_system", 1));

    // Inverse of the spawn offset odometry_bridge adds: the planner speaks the
    // shared world frame, PX4 wants setpoints in its own spawn-anchored local
    // frame. Must match odometry_bridge's spawn_* for this vehicle.
    spawn_yaw_ = this->declare_parameter<double>("spawn_yaw", 0.0);
    spawn_trans_ = Eigen::Vector3d(this->declare_parameter<double>("spawn_x", 0.0),
        this->declare_parameter<double>("spawn_y", 0.0),
        this->declare_parameter<double>("spawn_z", 0.0));
    spawn_rot_inv_ =
        Eigen::Quaterniond(Eigen::AngleAxisd(-spawn_yaw_, Eigen::Vector3d::UnitZ()));

    // Automatic takeoff before exploration starts. The planner cannot get the
    // vehicle off the ground on its own: sitting at z~0.1 the drone is inside
    // the ground's own inflation radius (sdf_map.obstacles_inflation = 0.199),
    // so the A* to the first viewpoint has no free start cell and every plan
    // fails ("No path to next viewpoint"). The reference MARSIM simulation
    // sidesteps this by spawning its drones already hovering at init_z = 1;
    // with PX4 they really do start on the ground, so this node flies them up
    // to takeoff_height and holds there until the first planner setpoint
    // arrives. Set to 0 to disable.
    takeoff_height_ = this->declare_parameter<double>("takeoff_height", 1.0);
    takeoff_done_tol_ = this->declare_parameter<double>("takeoff_done_tol", 0.15);
    // GIRO INIZIALE: raggiunta la quota di decollo il drone resta fermo e fa un
    // giro completo su se stesso prima che l'esplorazione parta. Il lidar vede
    // gia' a 360 gradi, quindi il giro non aggiunge geometria; serve alla
    // telecamera frontale (90 gradi), che cosi' fotografa tutto l'intorno della
    // partenza per colorare la mappa 3D, e a FAST-LIO, che parte con una mappa
    // e una stima stabilizzate mentre il drone e' fermo invece che gia' in moto.
    // Velocita' in rad/s; 0 = nessun giro. 0.5 rad/s = un giro in 12.6 s, ben
    // sotto il limite di imbardata di PX4 e lento per le foto (mosso).
    initial_spin_rate_ = this->declare_parameter<double>("initial_spin_rate", 0.5);

    // Blind radius of the reactive proximity check below. This used to be a
    // hardcoded 0.45m to discard the vehicle's own airframe returns - but
    // cloud_transform_bridge now drops those at the source (min_range, measured
    // from the sensor), so keeping a radius that large only blinded this last
    // line of defence to anything that got really close, another drone
    // included: with the stop threshold at 0.55m the window in which a peer was
    // visible at all was 10cm wide. Kept small but non-zero because the two
    // radii are measured from different origins - the filter from the sensor,
    // this from base_link 0.25m below it.
    self_hit_radius_ = this->declare_parameter<double>("self_hit_radius", 0.25);
    max_hold_time_ = this->declare_parameter<double>("max_proximity_hold_time", 5.0);
    // PROVATO E SCARTATO (campagna BOUND, 8 run): stringere questo limite a
    // 0.40 m insieme a setpoint_slew_speed 0.6 ha PEGGIORATO tutto - 2
    // ribaltamenti su 8 contro 1 su 6, e soprattutto il limitatore passava da
    // 0-7 interventi per run a 23-67, cioe' restava permanentemente attivo. Un
    // riferimento tenuto sempre a 40 cm fa volare il velivolo cronicamente
    // indietro rispetto alla propria traiettoria, e quel ritardo invalida la
    // separazione fra traiettorie calcolata dall'ottimizzatore.
    // L'errore di lettura a monte: il numero alto di interventi nelle run
    // finite male e' un SINTOMO (il riferimento scappa perche' il velivolo e'
    // gia' in difficolta'), non la causa. Stringere la vite che lo produce non
    // toglie la difficolta'.
    max_setpoint_jump_ = this->declare_parameter<double>("max_setpoint_jump", 1.0);

    // Both reactive layers freeze the vehicle in place, and neither exists
    // upstream: C2-Explorer resolves conflicts purely by replanning, which by
    // construction restarts from where the drone actually is. Freezing here
    // does not stop traj_server, which keeps evaluating the B-spline against
    // wall time - so every hold leaves the vehicle permanently behind its own
    // trajectory, and the setpoint clamp was firing 19-56 times per run
    // because of it. A drone that trails its trajectory by more than a metre
    // also invalidates the separation the optimizer computed between
    // trajectories, which is the margin that is supposed to keep drones apart.
    // Quel presupposto vale ancora per enable_lidar_hold, che resta spento.
    // NON vale piu' per enable_peer_hold: traj_server riscala il tempo della
    // traiettoria sull'errore di inseguimento e lo ferma del tutto a 1 m di
    // scarto, quindi una fermata non fa piu' scappare il piano - il piano
    // aspetta. Il launch multi-drone lo attiva (enable_peer_hold=True) perche'
    // nei corridoi il solo termine di sciame dell'ottimizzatore non impedisce
    // le collisioni fra droni. Il default qui resta false per non cambiare il
    // comportamento di chi avvia il nodo da solo.
    enable_lidar_hold_ = this->declare_parameter<bool>("enable_lidar_hold", false);
    // FILTRO DI SICUREZZA CONTINUO, alternativa alla fermata su lidar.
    //
    // La fermata (enable_lidar_hold) congela il riferimento e lo rilascia a
    // soglie di distanza: vicino a un muro la distanza misurata oscilla, e la
    // fermata si accende e si spegne piu' volte al secondo. Misurato sulla run
    // BASE_GPS 2 (office_small, drone singolo): fermata/ripresa 4 volte in 4 s
    // a 0.46-0.79 m dalla parete cw106, riferimento a onda quadra, deriva di
    // 0.43 m verso il muro, urto, inclinazione 76 gradi e 8 m/s. Dopo l'urto
    // la stessa fermata ha tenuto il drone in un ciclo "ferma 5 s, rilascia,
    // ferma" per oltre un minuto.
    // Il filtro invece non ferma niente: a ogni ciclo toglie al MOVIMENTO del
    // riferimento la sola componente diretta verso l'ostacolo piu' vicino,
    // limitandola in proporzione alla distanza (zero sotto safety_dist). Il
    // riferimento puo' sempre scorrere lungo la parete o allontanarsene, quindi
    // non c'e' nessuna soglia di rilascio, nessun salto e nessun blocco.
    enable_safety_filter_ = this->declare_parameter<bool>("enable_safety_filter", false);
    // Distanza centro-ostacolo sotto la quale il riferimento non si avvicina
    // piu': raggio d'ingombro dell'x500 con le eliche (0.35 m) piu' 0.2 m.
    safety_dist_ = this->declare_parameter<double>("safety_dist", 0.55);
    // Da qui in giu' la velocita' di avvicinamento consentita cala
    // linearmente fino a zero a safety_dist.
    safety_slow_dist_ = this->declare_parameter<double>("safety_slow_dist", 1.0);
    // Velocita' di avvicinamento massima nella fascia di rallentamento: quella
    // di crociera del pianificatore.
    safety_approach_speed_ = this->declare_parameter<double>("safety_approach_speed", 0.6);
    // LIMITE DI FRENATA (17 ottobre 2026, Jetson, cubicle_office senza GPS a 1.0 m/s).
    //
    // Il filtro qui sopra vincola il RIFERIMENTO, e solo sotto safety_slow_dist
    // (0.9 m). Il velivolo pero' arrivava alle pareti piu' veloce del riferimento:
    // inseguendolo con 0.4-0.6 m di ritardo, PX4 aggiungeva la correzione di
    // posizione all'anticipo e chiedeva 1.4-1.5 m/s col piano a 1.0-1.1; la
    // velocita' STIMATA restava indietro di quella vera fino a 0.4 m/s, e il drone
    // toccava 1.5-1.8 m/s veri. In tutti i contatti e quasi-contatti misurati (7
    // run, margine vero 0.27-0.50 m) un secondo prima il drone puntava l'ostacolo a
    // 0.8-1.3 m/s mentre il riferimento ne stava a 0.6-1.0 m: andava lungo in
    // curva per inerzia.
    // Qui il vincolo e' sul VELIVOLO e sui punti lidar che stanno SUL SUO
    // PERCORSO: dentro un tubo di raggio brake_gov_tube (ingombro con le eliche
    // 0.35 m + 0.10) lungo la direzione della velocita', entro brake_gov_range. La
    // velocita' non deve superare quella da cui ci si ferma prima che il tubo tocchi
    // il primo di quei punti, con decelerazione brake_gov_accel dopo un ritardo
    // brake_gov_latency (stima + controllo). Se la supera, il riferimento non resta
    // davanti al velivolo in quella direzione e l'anticipo di velocita' scende al
    // valore ammesso: PX4 frena col proprio anello di velocita'. Un tubo e non la
    // componente verso ogni punto: con quella, volare parallelo a una parete a
    // 0.7 m (i punti del muro davanti risultano "in avvicinamento") limitava a
    // ~0.9 m/s anche dove non c'e' niente da evitare.
    enable_brake_governor_ = this->declare_parameter<bool>("enable_brake_governor", false);
    brake_gov_accel_ = this->declare_parameter<double>("brake_gov_accel", 1.5);
    brake_gov_latency_ = this->declare_parameter<double>("brake_gov_latency", 0.3);
    brake_gov_tube_ = this->declare_parameter<double>("brake_gov_tube", 0.45);
    brake_gov_range_ = this->declare_parameter<double>("brake_gov_range", 2.5);
    // FRENATA ATTIVA. Togliere l'anticipo non basta: l'anello di velocita' di PX4
    // (MPC_XY_VEL_P_ACC 1.8) frena in modo esponenziale, con costante ~0.55 s,
    // cioe' ~1 m/s2 misurato: da 1.3 m/s consumava 0.75 m (run del 17 ottobre ore
    // 17:30, fermo a 0.40 m dallo spigolo col limite intervenuto a 1.15 m). Durante
    // un intervento si aggiunge all'anticipo un'accelerazione CONTRARIA al moto,
    // brake_gov_acc_gain per m/s di eccesso, al massimo brake_gov_acc_max (con
    // MPC_TILTMAX_AIR 20 PX4 ne ha ~3.6 m/s2): da 1.3 m/s l'arresto scende a ~0.45 m.
    // PROVATO E SCARTATO (stesso giorno, 5 run a 4.0 m/s2 per m/s, tetto 2.5): DUE
    // URTI su 5 (a 64 e 57 s, nei passaggi stretti della partenza), contro 0 su 8
    // senza. Nei passaggi stretti la velocita' cambia verso ogni 0.25-0.5 s e la
    // frenata, sempre contraria alla velocita' stimata, la insegue: accelerazione
    // comandata a +-2 m/s2 in versi opposti, inclinazione fino a 50 gradi. Di serie 0.
    brake_gov_acc_gain_ = this->declare_parameter<double>("brake_gov_acc_gain", 0.0);
    brake_gov_acc_max_ = this->declare_parameter<double>("brake_gov_acc_max", 2.5);
    // VELOCITA' LIMITATA VICINO AGLI OSTACOLI, in qualunque direzione (17 ottobre
    // 2026). Il limite di frenata qui sopra guarda solo gli ostacoli SUL percorso;
    // nei due quasi-contatti rimasti (margine vero 0.40-0.43 m) il drone girava
    // attorno a uno spigolo a 1.3-1.5 m/s veri, lo spigolo e' entrato "davanti"
    // all'improvviso e la frenata, pur partita (decelerazione ~2.4 m/s2), ha
    // consumato 0.45 m. Qui la velocita' del riferimento e il suo anticipo si
    // limitano in base all'ostacolo piu' vicino al VELIVOLO: prox_v_min a
    // prox_near, crescente fino a prox_v_far a prox_far, nessun limite oltre.
    // Sotto 0.7 m il drone passa ~1% del volo (5 run su cubicle_office), quindi il
    // costo in tempo stimato era sotto l'1%.
    // PROVATO E SCARTATO (stesso giorno, 5 run per parte): con il limite margine
    // minimo vero 0.35-0.55 m (mediana 0.40) e missione 284-364 s (mediana 333);
    // senza, 0.40-0.69 m (0.54) e 281-311 s (304). Rallentando il riferimento
    // vicino ai muri il drone resta piu' indietro rispetto alla traiettoria, e le
    // ripianificazioni dallo stato misurato si moltiplicano proprio li'. Di serie
    // spento, il launch non lo accende.
    enable_proximity_speed_ = this->declare_parameter<bool>("enable_proximity_speed", false);
    prox_near_ = this->declare_parameter<double>("prox_near", 0.6);
    prox_far_ = this->declare_parameter<double>("prox_far", 0.9);
    prox_v_min_ = this->declare_parameter<double>("prox_v_min", 0.5);
    prox_v_far_ = this->declare_parameter<double>("prox_v_far", 1.2);
    enable_peer_hold_ = this->declare_parameter<bool>("enable_peer_hold", false);
    use_velocity_feedforward_ = this->declare_parameter<bool>("use_velocity_feedforward", false);
    // Ben sopra la velocita' di crociera (0.35-0.6 m/s) perche' non deve
    // rallentare il volo normale: serve solo ad appianare i salti. Portarla a
    // 0.6 e' stato provato e scartato insieme a max_setpoint_jump 0.40 - vedi la
    // nota li' sopra: rende il vincolo permanente invece che occasionale.
    // Alzata a 1.6 insieme alla crociera (manager.max_vel 0.6 -> 1.0): deve
    // restare SOPRA la velocita' pianificata, altrimenti da limite occasionale
    // sui salti diventa un vincolo permanente - il difetto che ha fatto scartare
    // la campagna BOUND.
    setpoint_slew_speed_ = this->declare_parameter<double>("setpoint_slew_speed", 1.6);
    // Oltre questo silenzio l'odometria e' considerata scaduta (vedi odomFresh).
    odom_timeout_ = this->declare_parameter<double>("odom_timeout", 0.5);
    // Quota minima che il riferimento puo' chiedere, in coordinate mondo.
    //
    // Il drone incrociava a 0.90 m. Inclinarsi per accelerare in orizzontale
    // toglie spinta verticale, quindi un cedimento di quota e' fisiologico: con
    // 90 cm di margine finisce a terra. Sul registro di volo della run BAN 2 la
    // quota comandata scendeva a 0.72 e poi 0.50 m mentre quella reale crollava
    // a 0.38 e poi 0.05 m - il velivolo ha strisciato sul pavimento e si e'
    // ribaltato in 0.4 s (inclinazione da 21 a 158 gradi). Con il riferimento
    // tenuto a 1.0 m lo stesso cedimento lascia 65 cm di margine.
    // Le frontiere che richiedono davvero di scendere piu' in basso diventano
    // inosservabili, ed e' accettabile: il guardiano di stallo ora le archivia.
    min_flight_z_ = this->declare_parameter<double>("min_flight_z", 1.0);
    // Quota sotto cui il riferimento NON scende mai, nemmeno spinto dal filtro di
    // sicurezza (ostacolo sovrastante) o dal limite di frenata: min_flight_z vale
    // solo per il bersaglio del pianificatore, e quei due spostano la rampa dopo.
    // Misurato nel parco (18 ottobre 2026): sotto la piattaforma del parco giochi
    // (fondo a ~1.34 m) e sotto le chiome il filtro ha portato il riferimento a
    // 0.75 m mondo (0.52 m quota PX4) e il drone a 0.38-0.43 m. 0.8 m mondo lascia
    // 0.54 m dalla piattaforma (safety_dist 0.5) e ~0.57 m di quota PX4 al drone.
    min_flight_z_hard_ = this->declare_parameter<double>("min_flight_z_hard", 0.8);
    // TETTO di quota del riferimento, in coordinate mondo (0 = nessun tetto).
    // C'era solo il pavimento: nella run BASE_LIO 3, dopo un urto e la
    // divergenza della stima, il riferimento e' salito a 5.8 m con il soffitto
    // di office_small a 2.3 m.
    max_flight_z_ = this->declare_parameter<double>("max_flight_z", 0.0);
    // Semiampiezza della fascia verticale in cui lo strato reattivo cerca gli
    // ostacoli (vedi onCloud). Abbastanza da coprire l'ingombro del velivolo e
    // quello che gli sta davvero davanti, non abbastanza da includere pavimento
    // e soffitto.
    // Allargata da 0.45 a 0.70 m: con 45 cm restavano fuori dalla ricerca gli
    // ostacoli poco sopra o poco sotto la quota di volo - uno spigolo di tavolo,
    // una mensola - che il velivolo puo' comunque colpire con le eliche.
    // Il pavimento non rientra: e' escluso a monte dal filtro sui punti sotto
    // 0.15 m, quindi allargare la fascia non lo fa tornare a mascherare le
    // pareti (era il difetto originale).
    // Motivazione: i due ribaltamenti residui sono eventi IMPROVVISI durante volo
    // regolare - quota corretta fino all'istante prima, poi 6-7 m/s che sono
    // successivi all'impatto. Quindi si colpisce qualcosa che non era stato
    // visto.
    proximity_band_z_ = this->declare_parameter<double>("proximity_band_z", 0.70);
    // Soglie dello stop reattivo su lidar. Devono stare sopra il min_range
    // della nuvola (0.5 m) con un margine utile, altrimenti lo strato non ha
    // nessuna finestra in cui vedere qualcosa.
    // DISTANZA A CUI SI COMINCIA A GUARDARE, non a cui ci si ferma: dentro
    // questa soglia interviene il criterio sul tempo al contatto, che ferma solo
    // se ci si sta davvero andando addosso.
    //
    // Alzata da 0.80 a 1.80 m. Il motivo e' che 0.80 non era un limite del
    // sensore ma una scelta, e misurata sul campo era troppo stretta: fra 0.80 e
    // 0.45 m (la soglia dura) restano 35 cm, che a 0.6 m/s valgono 0.58 s e a
    // 1.0 m/s appena 0.35 s. In quella finestra il 40-51% degli arresti scattava
    // ormai sulla soglia dura, cioe' all'ultimo momento.
    // Il sensore vede molto piu' lontano: misurati su 1965 campioni, il 69% dei
    // punti piu' vicini sta fra 0.80 e 1.50 m, con massimi a 2.90 m. La
    // limitazione era autoimposta.
    // A 1.80 m restano 1.35 m prima della soglia dura: 2.2 s a 0.6 m/s, contro
    // una distanza di frenata di ~9 cm. Margine abbondante.
    obstacle_stop_dist_ = this->declare_parameter<double>("obstacle_stop_dist", 1.80);
    // Deve restare sopra stop_dist, altrimenti la fermata non si revoca mai.
    obstacle_resume_dist_ = this->declare_parameter<double>("obstacle_resume_dist", 2.10);
    // Decelerazione con cui si calcola un punto di arresto ESEGUIBILE (vedi
    // brakingHoldPoint). Sotto MPC_ACC_HOR di PX4 per restare nel dominio in
    // cui il controllore non satura.
    brake_accel_ = this->declare_parameter<double>("brake_accel", 2.0);
    // Durata minima di una fermata prima che possa essere revocata. Le distanze
    // misurate oscillano (1.26 -> 0.50 -> 1.24 m in 0.9 s sulla run UAF 3):
    // senza questo la fermata si accende e si spegne piu' volte al secondo e il
    // riferimento diventa un'onda quadra.
    min_hold_time_ = this->declare_parameter<double>("min_hold_time", 0.6);
    // Arretramento alla scadenza della fermata (vedi onCloud). Default 0 =
    // disattivato: provato su 4 run, l'arretramento e' stato eseguito 109
    // volte ma NON ha ridotto i crash (3 su 4 contro 1 su 4 senza), e il
    // margine minimo misurato sulla verita' del simulatore non e' migliorato.
    // Nessuna correlazione temporale fra arretramenti e ribaltamenti (l'ultimo
    // cadeva 14-117 s prima), quindi non fa danno: semplicemente non serve, e
    // una modifica che non sposta il numero per cui e' stata fatta non resta
    // attiva. Codice e motivazione conservati perche' il problema che aggredisce
    // e' reale: alla scadenza il controllo torna a un pianificatore che la
    // geometria non ancora mappata non la vede.
    max_retreats_ = this->declare_parameter<int>("max_obstacle_retreats", 0);
    // Tempo al contatto sotto il quale ci si ferma. 1.2 s a 0.6 m/s significa
    // intervenire ~0.7 m prima dell'impatto, con la fermata che avviene in
    // frazioni di secondo.
    obstacle_ttc_ = this->declare_parameter<double>("obstacle_time_to_contact", 1.2);

    // Dedicated inter-drone separation layer, independent of the lidar check
    // above. The lidar cannot tell a peer from a wall, so its threshold has to
    // stay below the clearance the planner legitimately keeps from mapped
    // obstacles (optimization.dist0 = 0.7) or it would trip on every wall - and
    // 0.55m at 1 m/s closing speed is under half a second from contact. That is
    // how drone 2 ended up inverted: it detected an obstacle at 0.35m and hit
    // 0.7s later. Peer positions are known exactly at 20Hz from DroneState, so
    // this layer can use a threshold several times larger with no risk of
    // false-triggering on geometry, and it extrapolates the relative velocity
    // to react before the gap closes rather than after.
    drone_id_ = this->declare_parameter<int>("drone_id", 1);
    // PROVATO E SCARTATO - stringere i raggi a 1.0 / 1.5 / 0.7 contando sul
    // ponte passato da 10 a 50 Hz per recuperare il margine.
    //
    // Il ragionamento era che a 10 Hz fra un comando e l'altro passavano 100 ms
    // e la fermata doveva partire da lontano; a 50 Hz quel margine e' un quinto.
    // Non regge alla misura (campagna V5, base V3): i contatti sotto 0.5 m fra
    // droni passano da ~54 a 261, cinque volte tanto. La prontezza piu' alta non
    // compensa lo spazio perso.
    // In piu' c'e' un accoppiamento che non avevo previsto: la manovra di
    // sblocco usa la FERMATA PER COMPAGNO come segnale di stallo. Stringendo la
    // soglia la fermata scatta piu' tardi e meno spesso, si rilevano meno
    // stalli, e la separazione verticale al passaggio crolla dal 31% al 7% del
    // tempo sopra 0.40 m. I due meccanismi non sono indipendenti.
    peer_stop_dist_ = this->declare_parameter<double>("peer_stop_dist", 1.5);
    peer_resume_dist_ = this->declare_parameter<double>("peer_resume_dist", 2.0);
    // Below this nobody has right of way any more: both drones stop, whatever
    // their ids. Priority is an optimisation, not something to bet a vehicle on.
    peer_hard_stop_dist_ = this->declare_parameter<double>("peer_hard_stop_dist", 1.0);
    peer_lookahead_ = this->declare_parameter<double>("peer_lookahead", 1.0);
    peer_state_timeout_ = this->declare_parameter<double>("peer_state_timeout", 1.0);
    // Incrocio in quota (vedi updateCrossing). Si innesca PRIMA della fermata
    // per compagno (peer_stop_dist 1.5 m): l'obiettivo e' scansarsi mentre si
    // ha ancora spazio, non rimediare quando si e' gia' addosso.
    // TERZA VERSIONE - a corsie di quota assolute (0.50 m sotto, 1.50 m sopra).
    //
    // Le prime due sono state provate e sono peggiorative, ed e' documentato in
    // updateCrossing perche': lo scarto RELATIVO veniva mangiato dalla quota
    // minima di volo, quindi chi doveva scendere non scendeva. Le corsie
    // assolute tolgono quel vincolo ed e' la ragione per riprovare.
    //
    //   RIPIEGO2 (senza manovra): 4/4 complete, 0 crash, mediana 282 s, contatti
    //                             sotto 0.5 m: 28
    //   INCROCIO  (uno sale uno scende): 2/4, 2 crash, sotto 0.5 m: 112
    //   INCROCIO2 (sale solo l'id piu' basso, fermata tolta solo a separazione
    //             verticale ottenuta): 3/4, 2 crash, durate 207/297/507/677
    //             (mediana ~402 contro 282), sotto 0.5 m: 103
    //
    // La seconda versione ha corretto quello che la prima sbagliava - le manovre
    // sono 95 e tutte in salita, l'asimmetria 44/85 e' sparita - e il risultato
    // NON e' migliorato. Quindi il difetto non era l'implementazione: e' che qui
    // la manovra non ha spazio. La scena e' alta 2.5 m, la quota minima di volo
    // e' 1.0 m e si vola a ~0.95 m: la fascia utile e' meno di un metro, e la
    // salita passa dalla rampa a 1.6 m/s, quindi per tutto il tempo in cui il
    // drone sta salendo i due continuano ad avvicinarsi. Si arriva separati
    // troppo tardi, e intanto si e' volato l'uno verso l'altro invece di
    // fermarsi.
    // In un edificio vero, con soffitti da 3 m in su, le due corsie possono
    // stare piu' larghe e la manovra rende molto di piu': qui il metro netto fra
    // 0.50 e 1.50 e' il massimo che una scena alta 2.5 m concede.
    enable_crossing_ = this->declare_parameter<bool>("enable_crossing", true);
    // MARGINE DI ANTICIPO, non distanza di sicurezza.
    //
    // A 2.5 m la manovra si innescava troppo tardi: due droni che si avvicinano
    // a ~1.2 m/s complessivi hanno 2 secondi prima di incontrarsi, e in due
    // secondi un multirotore non scende di mezzo metro e non risale. Misurato
    // sui registri di volo con le corsie gia' assegnate correttamente
    // (0.50 contro 1.50, coppie opposte verificate): separazione verticale
    // effettiva 0.11 m di mediana, massimo 0.38 m su un metro richiesto, mai
    // sopra 0.40 m. Le quote erano giuste, mancava il tempo per raggiungerle.
    // A 5.0 m il margine raddoppia a ~4 s.
    cross_trigger_dist_ = this->declare_parameter<double>("cross_trigger_dist", 3.0);
    // Le due corsie dell'incrocio, in quota assoluta. Un metro netto fra loro,
    // contro ~0.5 m di larghezza del velivolo. La corsia bassa sta SOTTO la
    // quota minima di volo (1.0 m) ed e' voluto: vedi la nota in updateCrossing.
    cross_low_z_ = this->declare_parameter<double>("cross_low_z", 0.50);
    cross_high_z_ = this->declare_parameter<double>("cross_high_z", 1.50);
    cross_z_band_ = this->declare_parameter<double>("cross_z_band", 0.6);
    cross_min_speed_ = this->declare_parameter<double>("cross_min_speed", 0.15);
    // Coseno di soglia per dire "frontale": 0.3 corrisponde a ~107 gradi fra le
    // due rotte. Piu' stretto lascerebbe fuori gli incroci obliqui, che in un
    // corridoio sono la norma perche' nessuno vola perfettamente in asse.
    cross_head_on_cos_ = this->declare_parameter<double>("cross_head_on_cos", 0.3);
    cross_timeout_ = this->declare_parameter<double>("cross_timeout", 8.0);
    // Da quanto tempo lo stesso compagno deve bloccare prima di considerarlo
    // uno stallo. Sotto i due secondi si scambierebbero per blocchi i normali
    // rallentamenti di quando ci si incrocia.
    cross_stall_time_ = this->declare_parameter<double>("cross_stall_time", 2.0);
    // Velocita' della sola verticale durante l'incrocio. 0.8 m/s copre i ~0.45 m
    // che separano la crociera da una corsia in poco piu' di mezzo secondo, ben
    // dentro la finestra di un incontro (misurata fra 1.3 e 15 s).
    // NON ANCORA PROVATO DA SOLO. Un tentativo a 1.8 m/s e' fallito (10% contro
    // 31%, 4 ribaltamenti contro 2) ma era stato fatto INSIEME al passaggio a
    // 100 Hz, quindi non era attribuibile. La campagna V6, con i soli 100 Hz, ha
    // poi riprodotto lo stesso crollo di separazione al 10%: il colpevole era la
    // frequenza. Alzare questa velocita' resta quindi una prova da fare, non una
    // strada gia' esclusa.
    cross_z_speed_ = this->declare_parameter<double>("cross_z_speed", 0.8);
    // Tetto di sicurezza per la salita: la scena e' alta 2.5 m e non si vuole
    // andare a strusciare il soffitto per scansare un compagno.
    // Dislivello oltre il quale i due si considerano su piani distinti e la
    // fermata per compagno puo' essere tolta. Il velivolo x500 e' largo ~0.5 m:
    // 0.45 m di scarto verticale li fa gia' passare senza toccarsi.
    cross_clear_dz_ = this->declare_parameter<double>("cross_clear_dz", 0.45);

    peer_state_sub_ = this->create_subscription<c2x_msgs::msg::DroneState>(
        "swarm_expl/drone_state_recv", 20,
        [this](const c2x_msgs::msg::DroneState::ConstSharedPtr msg) { onPeerState(*msg); });

    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        px4_ns + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        px4_ns + "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ =
        this->create_publisher<px4_msgs::msg::VehicleCommand>(px4_ns + "/fmu/in/vehicle_command", 10);

    rclcpp::QoS status_qos(rclcpp::KeepLast(10));
    status_qos.best_effort();
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        px4_ns + "/fmu/out/vehicle_status_v1", status_qos,
        [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
          armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
          // Modalita' attiva, per accorgersi di quando PX4 esce da solo
          // dall'offboard (vedi controllaModalitaPx4).
          nav_state_ = msg->nav_state;
          // Annotato qui, a ogni messaggio, e non nel ciclo del ponte: e' la
          // prova che l'armamento in offboard e' riuscito, da li' in poi
          // un'altra modalita' ad armato e' PX4 che si riprende il volo.
          if (armed_ && nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            visto_offboard_armato_ = true;
          }
        });

    position_cmd_sub_ = this->create_subscription<c2x_msgs::msg::PositionCommand>(
        "position_cmd", 50,
        [this](const c2x_msgs::msg::PositionCommand::ConstSharedPtr msg) { onPositionCommand(*msg); });

    // Richiesta di atterraggio dall'FSM: arriva quando il drone e' rientrato
    // alla partenza e ha concluso. Un canale a se' invece di riusare
    // trajectory_flag=ABORT, che nonostante il commento in cima al file NON
    // atterrava - si limitava a tenere l'ultimo comando valido.
    // Annuncio all'FSM che il volo e' finito senza che lo avesse chiesto (vedi
    // il controllo sul disarmo in tick()). transient_local: deve arrivare anche
    // a un iscritto tardivo.
    flight_terminated_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "flight_terminated", rclcpp::QoS(1).transient_local());
    // Decollo e giro iniziale conclusi: l'FSM si dichiara pronta al via solo
    // dopo, cosi' il pianificatore non comincia a mandare traiettorie a un drone
    // ancora in salita o in rotazione. transient_local per l'iscritto tardivo.
    volo_pronto_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "volo_pronto", rclcpp::QoS(1).transient_local());

    land_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "land_request", 1, [this](const std_msgs::msg::Empty::ConstSharedPtr) {
          if (landing_) return;
          landing_ = true;
          RCLCPP_WARN(this->get_logger(), "[Atterraggio] richiesto: passo in modalita' LAND");
          if (!ceduto_a_px4_) riepilogoQuota();  // gia' scritto cedendo il comando
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        });

    // ATTERRAGGIO DI PRECISIONE SULLA PARTENZA (fine missione normale).
    //
    // Il pianificatore riporta il drone entro 1.5 m dalla partenza ma non sul
    // punto esatto: li' lo spazio e' stretto e la ricerca di percorso col
    // gonfiaggio non trova celle libere. Prima si atterrava dove si arrivava,
    // a 1.2-1.5 m dal punto di decollo (campagna V13). Qui il ponte fa l'ultimo
    // tratto da solo: rampa lenta in linea retta verso la verticale del punto
    // di decollo, alla quota attuale, con il filtro di sicurezza attivo come in
    // ogni altro movimento; quando il drone e' fermo li' sopra, atterraggio
    // verticale di PX4. Resta separato da land_request, che e' l'atterraggio
    // "qui e subito" delle emergenze (crash, tempo scaduto).
    home_land_tol_ = this->declare_parameter<double>("home_land_tol", 0.10);
    home_approach_speed_ = this->declare_parameter<double>("home_approach_speed", 0.4);
    home_approach_timeout_ = this->declare_parameter<double>("home_approach_timeout", 30.0);
    home_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "land_at_home", 1, [this](const std_msgs::msg::Empty::ConstSharedPtr) {
          // Comando ceduto a PX4 (controllaModalitaPx4): l'avvicinamento lo
          // volerebbe il ponte, che non pubblica piu'. L'FSM e' gia' stata
          // avvisata con flight_terminated; un land_request invece vale ancora.
          if (landing_ || rientro_preciso_ || ceduto_a_px4_) return;
          if (!have_odom_ || !takeoff_done_) {
            // Mai decollato davvero: non c'e' una "casa" sensata, si atterra qui.
            landing_ = true;
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            return;
          }
          rientro_preciso_ = true;
          rientro_inizio_ = this->now();
          rientro_fermo_dal_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          home_target_world_ = Eigen::Vector3d(takeoff_target_world_.x(),
              takeoff_target_world_.y(), std::max(cur_pos_world_.z(), min_flight_z_));
          riepilogoQuota();
          RCLCPP_WARN(this->get_logger(), "[Casa] avvicinamento finale: %.2f m dalla verticale "
              "del decollo", std::hypot(cur_pos_world_.x() - home_target_world_.x(),
                                        cur_pos_world_.y() - home_target_world_.y()));
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world", 25, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          cur_pos_world_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
              msg->pose.pose.position.z);
          cur_vel_world_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
              msg->twist.twist.linear.z);
          if (!have_odom_) {
            // Hover straight above wherever the vehicle actually started.
            takeoff_target_world_ =
                Eigen::Vector3d(cur_pos_world_.x(), cur_pos_world_.y(), takeoff_height_);
            // ...e con la prua che ha gia'. last_yaw_ned_ partiva da 0 (nord),
            // quindi il decollo comandava anche una rotazione di 90 gradi per un
            // drone nato rivolto a est. Stessa conversione dei comandi del
            // pianificatore (ENU mondo -> NED locale).
            const auto& q = msg->pose.pose.orientation;
            const double yaw_enu =
                std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            last_yaw_ned_ = static_cast<float>(std::remainder(M_PI_2 - (yaw_enu - spawn_yaw_), 2.0 * M_PI));
          }
          have_odom_ = true;
          last_odom_time_ = this->now();
        });

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(5));
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "map_ros/cloud", cloud_qos,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(*msg); });

    // Heartbeat + setpoint publish loop: PX4 requires OffboardControlMode +
    // TrajectorySetpoint at >2Hz to remain in / enter offboard mode.
    //
    // 50 Hz, non 10. A 100 ms il minimo di PX4 era ampiamente rispettato, ma il
    // minimo non e' il criterio giusto: dentro questo tick vive TUTTO lo strato
    // reattivo - la fermata per ostacolo, quella per compagno, la manovra di
    // sblocco - quindi a 10 Hz il velivolo reagiva a scatti da un decimo di
    // secondo, e nel registro di volo i comandi effettivi risultavano a 5 Hz.
    // Misurato: assetto a 250 Hz, posizione a 125 Hz, comando di posizione a 5.
    // Fra un comando e il successivo un drone a 1.5 m/s percorre 30 cm senza
    // riferimento aggiornato - ed e' il regime in cui avvengono gli avvicinamenti
    // fra droni che precedono i ribaltamenti.
    // Il riferimento c'era gia': traj_server pubblica a 100 Hz e questo ponte ne
    // buttava via il 90%. Non serve produrre piu' informazione, solo smettere di
    // scartarla.
    // Tutto cio' che dipende dal tempo qui dentro e' gia' scritto in secondi o
    // in metri al secondo moltiplicati per il dt misurato (rampAndClamp), quindi
    // si adatta da solo alla nuova cadenza.
    //
    // 100 Hz PROVATO E SCARTATO (campagna V6, unica variabile rispetto a V3):
    // la separazione verticale al passaggio crolla dal 31% al 10% del tempo
    // sopra 0.40 m e i contatti sotto 0.5 m fra droni triplicano, da ~54 a 175.
    // I ribaltamenti restano 2 su 12 voli. Piu' veloce non e' meglio: 50 Hz e'
    // gia' oltre il punto in cui il tempo di reazione era il vincolo, e
    // raddoppiare aggiunge solo traffico sul collegamento verso PX4 - con tre
    // droni sono 6 flussi invece di 3 sullo stesso agente DDS.
    timer_ = this->create_wall_timer(20ms, [this]() { tick(); });
  }

private:
  // Independent reactive safety check: recompute the nearest raw lidar point to the
  // drone's current position, ignoring near-floor returns (z < 0.15m) so low-altitude
  // flight near the ground doesn't trip it - the floor being "close" vertically isn't the
  // same hazard as a wall being close horizontally, and the exploration map/planner already
  // handles ground clearance on its own terms. Also ignores points closer than
  // self_hit_radius: the lidar is mounted 0.25m above base_link on the x500, and its wide
  // vertical FOV can see the vehicle's own arms/legs at ~0.25-0.4m - confirmed empirically
  // (first version of this check fired 43 times in 4 minutes, always at 0.25-0.4m,
  // resolving instantly once the vehicle's attitude shifted slightly - a real external
  // obstacle wouldn't behave that way). That's an inherent blind zone for any body-mounted
  // sensor with this FOV, not something a software filter can fully recover; the tradeoff is
  // accepted since obstacles_inflation already keeps planned trajectories well clear of that
  // range anyway. Hysteresis (stop below 0.55m, only resume past 0.75m) avoids flapping.
  void onCloud(const sensor_msgs::msg::PointCloud2& msg) {
    if (enable_safety_filter_ && odomFresh()) aggiornaOstacoli(msg);
    if (!odomFresh() || !enable_lidar_hold_) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(msg, cloud);
    const double self_hit_radius = self_hit_radius_;
    double min_d2 = std::numeric_limits<double>::infinity();
    for (const auto& p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.z < 0.15) continue;
      const double dx = p.x - cur_pos_world_.x();
      const double dy = p.y - cur_pos_world_.y();
      const double dz = p.z - cur_pos_world_.z();
      // Solo la fascia orizzontale attorno al velivolo.
      //
      // Senza questo filtro il punto piu' vicino era quasi sempre il PAVIMENTO:
      // misurato su 1965 campioni, distanza mediana 0.91 m, cioe' esattamente la
      // quota di crociera. E siccome qui si guarda il solo punto piu' vicino, una
      // parete davanti diventava "la piu' vicina" solo quando scendeva sotto la
      // distanza del pavimento - sotto i 0.9 m. Da qui il fatto che TUTTE le
      // fermate osservate scattassero fra 0.41 e 0.49 m, cioe' sempre dalla
      // soglia dura, e mai dal criterio sul tempo al contatto che dovrebbe
      // intervenire prima: il pavimento mascherava gli ostacoli frontali.
      // A 1.59 m/s quei 45 cm valgono 0.3 s, e infatti il velivolo ci e' andato
      // dentro (impulso da urto misurato: da 1.59 a 5.14 m/s in 0.12 s).
      // Il margine dal suolo non serve qui: lo garantisce min_flight_z.
      if (std::abs(dz) > proximity_band_z_) continue;
      const double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < self_hit_radius * self_hit_radius) continue;
      if (d2 < min_d2) {
        min_d2 = d2;
        // Serve anche la DIREZIONE, non solo la distanza: senza sapere da che
        // parte sta l'ostacolo non si puo' arretrare (vedi la scadenza della
        // fermata piu' sotto).
        nearest_obstacle_pt_ = Eigen::Vector3d(p.x, p.y, p.z);
      }
    }
    if (!std::isfinite(min_d2)) return;
    nearest_obstacle_dist_ = std::sqrt(min_d2);
    nearest_obstacle_valid_ = true;

    // Erano fissi a 0.55/0.75, e con quei valori lo strato era praticamente
    // cieco: cloud_transform_bridge scarta i punti sotto min_range (0.5 m dal
    // sensore) per togliere gli auto-colpi, quindi il punto piu' vicino che puo'
    // esistere nella nuvola sta a 0.5 m e la finestra utile di rilevamento era
    // 0.50-0.55 m, cinque centimetri - sotto un decimo di secondo di preavviso
    // alla velocita' di crociera. Resi parametrici e allargati dal launch.
    const double stop_dist = obstacle_stop_dist_;
    const double resume_dist = obstacle_resume_dist_;
    // Criterio di intervento: non la sola distanza, ma il TEMPO AL CONTATTO.
    // Con la sola distanza il drone si fermava anche passando parallelo a una
    // parete senza avvicinarsi di un centimetro - 146 fermate in 3 run, ed e' il
    // motivo per cui i droni sembravano non muoversi mai. Quello che conta e' se
    // ci stai andando addosso: si proietta la velocita' sulla direzione
    // dell'ostacolo e si guarda quanto manca all'impatto. Sotto hard_stop si
    // ferma comunque, perche' a quella distanza non si scommette sulla stima
    // della velocita'.
    // SOGLIA DURA DIMENSIONATA, non scelta a occhio.
    //
    // Era 0.45 m fisso, e con quel valore la fermata parte gia' persa: il
    // velivolo ha toccato 1.65 m/s, e con brake_accel 2.0 m/s^2 la distanza di
    // frenata vale v^2/2a = 0.68 m. Sommando la semiapertura dei rotori (0.25 m)
    // servono 0.93 m per fermarsi senza toccare. Con 0.45 la fermata scattava
    // avendone a disposizione 0.33: meta' del necessario.
    // Misurato sull'urto contro il muro cw6 di cubicle_office: fermata annunciata
    // a 0.78 m, impatto a 148 g un decimo di secondo dopo.
    //
    // PROVATO E SCARTATO: soglia dura calcolata da raggio rotori + distanza di
    // frenata, invece dei 0.45 m storici.
    //
    // L'idea era giusta in astratto - una soglia di sicurezza si dimensiona, non
    // si sceglie - ma non ha funzionato e ha fatto danni:
    //
    //  - dimensionata sulla velocita' STIMATA non cambiava nulla (0.75 m contro
    //    0.78), perche' la velocita' stimata e' proprio la grandezza sbagliata
    //    in volo cieco: costruiva la rete di sicurezza sul numero rotto;
    //  - dimensionata sulla velocita' di progetto (1.5 m/s, il tetto di PX4)
    //    dava 0.81 m e BLOCCAVA il drone: in office_small il velivolo e' quasi
    //    sempre entro 0.81 m da qualcosa, quindi la fermata non si revocava mai
    //    (71 fermate consecutive, velocita' 0.01 m/s).
    //
    // E soprattutto: questa riga sta FUORI dal ramo senza GPS, quindi valeva
    // anche per la configurazione con GPS, che funzionava. L'ho modificata
    // testando solo il volo cieco e ho rotto quella buona. Le modifiche per
    // FAST-LIO non devono toccare il percorso condiviso: se un giorno servira'
    // una soglia diversa in volo cieco, va passata come parametro dal launch
    // solo per quella configurazione, non cambiata qui per tutti.
    //
    // 0.45 m e' il valore storico, ed era gia' dimensionato bene per il regime
    // in cui il velivolo vola davvero: a 0.6 m/s del pianificatore la frenata
    // vale 9 cm, e restano 0.20 m di margine oltre i rotori.
    const double hard_stop = std::min(0.45, stop_dist);
    double closing = 0.0;
    if (nearest_obstacle_valid_ && nearest_obstacle_dist_ > 1e-3) {
      const Eigen::Vector3d dir = (nearest_obstacle_pt_ - cur_pos_world_) / nearest_obstacle_dist_;
      closing = cur_vel_world_.dot(dir);  // >0 = mi sto avvicinando
    }
    const double ttc = (closing > 0.05)
        ? (nearest_obstacle_dist_ - hard_stop) / closing
        : std::numeric_limits<double>::infinity();
    const bool imminent = nearest_obstacle_dist_ < hard_stop ||
        (nearest_obstacle_dist_ < stop_dist && ttc < obstacle_ttc_);

    // Diagnostica: TUTTE le fermate osservate scattano fra 0.41 e 0.49 m, cioe'
    // sempre dal ramo hard_stop (0.45 m); il ramo sul tempo al contatto, che
    // dovrebbe intervenire fra 0.45 e 0.80 m, non si vede mai nei log. Le due
    // spiegazioni possibili - la nuvola non contiene punti a quelle distanze,
    // oppure li contiene e il criterio non scatta - richiedono rimedi opposti,
    // quindi va misurato invece che dedotto. Qui si registra la distanza vista
    // a ogni ciclo, non solo quando si decide di fermarsi.
    {
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000,
          "[Prossimita] piu' vicino %.2fm  avvicinamento %.2fm/s  tempo al contatto %.2fs  "
          "velocita %.2fm/s",
          nearest_obstacle_dist_, closing, std::isfinite(ttc) ? ttc : -1.0,
          cur_vel_world_.norm());
    }

    if (!proximity_hold_ && imminent) {
      proximity_hold_ = true;
      hold_start_ = this->now();
      hold_pos_world_ = brakingHoldPoint(nearest_obstacle_dist_ - hard_stop);
      retreat_count_ = 0;
      RCLCPP_ERROR(this->get_logger(),
          "PROXIMITY STOP: obstacle at %.2fm, holding position - overriding planner setpoints "
          "until clear",
          nearest_obstacle_dist_);
    } else if (proximity_hold_ && nearest_obstacle_dist_ > resume_dist &&
               (this->now() - hold_start_).seconds() >= min_hold_time_) {
      proximity_hold_ = false;
      retreat_count_ = 0;
      RCLCPP_WARN(this->get_logger(), "Proximity clear (%.2fm) - resuming planner setpoints",
          nearest_obstacle_dist_);
    } else if (proximity_hold_ && max_hold_time_ > 0.0 &&
               (this->now() - hold_start_).seconds() > max_hold_time_) {
      // A hold can never clear itself against something that does not move.
      // Freezing in place keeps the distance exactly where it was, so the
      // resume threshold is never reached and the drone sits there forever
      // while the planner keeps replanning around it. Peers move away on their
      // own and clear within a second or two; a wall does not. Hand control
      // back and let the planner - which does see the map, and keeps its own
      // obstacles_inflation clearance - fly the drone out.
      //
      // Ma quel presupposto e' proprio quello che salta quando serve: se
      // l'ostacolo non e' ancora nella mappa (in esplorazione si vola verso il
      // confine del noto) il pianificatore continua a puntarci dentro, e
      // rilasciare qui significa consegnargli il drone a 0.8 m da un muro.
      // Misurato: lo strato interviene (146 volte in 3 run) e i droni arrivano
      // lo stesso a 0.20 m, dentro l'ingombro delle eliche. Quindi invece di
      // arrendersi si ARRETRA lungo la direzione opposta all'ostacolo, che e'
      // un movimento sicuro per costruzione (aumenta la distanza da cio' che ci
      // preoccupa). Il rilascio resta come ultima risorsa dopo qualche
      // tentativo, cosi' la protezione anti-stallo che motivava la scadenza
      // rimane.
      if (retreat_count_ < max_retreats_ && nearest_obstacle_valid_) {
        Eigen::Vector3d away = cur_pos_world_ - nearest_obstacle_pt_;
        away.z() = 0.0;  // resta in quota: salire o scendere non allontana dal muro
        if (away.norm() < 1e-3)
          away = Eigen::Vector3d(-1.0, 0.0, 0.0);
        else
          away.normalize();
        // Passo contenuto: il ramo di hold pubblica hold_pos_world_ senza
        // passare dalla rampa, quindi un arretramento grande sarebbe a sua
        // volta un gradino.
        const double step = std::min(0.45, std::max(0.30, resume_dist + 0.15 - nearest_obstacle_dist_));
        hold_pos_world_ = cur_pos_world_ + away * step;
        hold_start_ = this->now();
        ++retreat_count_;
        RCLCPP_WARN(this->get_logger(),
            "Ostacolo statico a %.2fm dopo %.1fs di fermata - arretro di %.2fm (tentativo %d/%d)",
            nearest_obstacle_dist_, max_hold_time_, step, retreat_count_, max_retreats_);
      } else {
        proximity_hold_ = false;
        retreat_count_ = 0;
        RCLCPP_WARN(this->get_logger(),
            "Proximity hold held %.1fs at %.2fm without clearing (static obstacle) - releasing to "
            "the planner",
            (this->now() - hold_start_).seconds(), nearest_obstacle_dist_);
      }
    }
  }

  // Insieme di ostacoli per il filtro di sicurezza, in coordinate mondo.
  //
  // Si tengono solo i punti che contano per i prossimi istanti: entro 3 m in
  // orizzontale e nella fascia verticale del velivolo. Il pavimento resta
  // fuori come per la fermata (z < 0.15): la quota minima la garantisce
  // min_flight_z. Un punto per voxel da 0.1 m basta per una distanza e tiene
  // la ricerca lineare a poche migliaia di punti.
  // Le coordinate sono di MONDO e la scena e' statica, quindi se la nuvola si
  // interrompe per un attimo l'ultimo insieme resta valido.
  void aggiornaOstacoli(const sensor_msgs::msg::PointCloud2& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(msg, cloud);
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(4096);
    std::unordered_set<int64_t> visti;
    visti.reserve(8192);
    const double band = proximity_band_z_ + 0.2;
    for (const auto& p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.z < 0.15) continue;
      const double dx = p.x - cur_pos_world_.x();
      const double dy = p.y - cur_pos_world_.y();
      if (dx * dx + dy * dy > 9.0) continue;
      if (std::abs(p.z - cur_pos_world_.z()) > band) continue;
      const int64_t ix = static_cast<int64_t>(std::floor(p.x * 10.0)) & 0xFFFFF;
      const int64_t iy = static_cast<int64_t>(std::floor(p.y * 10.0)) & 0xFFFFF;
      const int64_t iz = static_cast<int64_t>(std::floor(p.z * 10.0)) & 0xFFFFF;
      if (!visti.insert((ix << 40) | (iy << 20) | iz).second) continue;
      pts.emplace_back(p.x, p.y, p.z);
    }
    ostacoli_.swap(pts);
    ostacoli_validi_ = true;
  }

  // Ostacolo piu' vicino a p: distanza e direzione (versore da p all'ostacolo).
  // I punti piu' vicini di self_hit_radius al VELIVOLO sono esclusi, come
  // nella fermata: sono ritorni del telaio sfuggiti al filtro a monte.
  bool ostacoloPiuVicino(const Eigen::Vector3d& p, Eigen::Vector3d& dir, double& dist) const {
    if (!ostacoli_validi_ || ostacoli_.empty()) return false;
    double best = std::numeric_limits<double>::infinity();
    const double self2 = self_hit_radius_ * self_hit_radius_;
    for (const auto& o : ostacoli_) {
      if ((o - cur_pos_world_).squaredNorm() < self2) continue;
      const double d2 = (o - p).squaredNorm();
      if (d2 < best) {
        best = d2;
        dir = o - p;
      }
    }
    if (!std::isfinite(best) || best < 1e-8) return false;
    dist = std::sqrt(best);
    dir /= dist;
    return true;
  }

  // Velocita' di avvicinamento consentita a distanza d dall'ostacolo.
  double velocitaAvvicinamentoAmmessa(double d) const {
    if (d <= safety_dist_) return 0.0;
    if (d >= safety_slow_dist_) return std::numeric_limits<double>::infinity();
    return safety_approach_speed_ * (d - safety_dist_) /
           std::max(1e-3, safety_slow_dist_ - safety_dist_);
  }

  // Applica il filtro a uno spostamento del riferimento da 'da' di 'passo'.
  // Due iterazioni: dopo aver tolto la componente verso un ostacolo, il nuovo
  // punto puo' avvicinarsi a un altro (un angolo, un passaggio stretto).
  Eigen::Vector3d filtraPasso(const Eigen::Vector3d& da, Eigen::Vector3d passo, double dt) {
    filtro_attivo_ = false;
    if (!enable_safety_filter_ || dt <= 0.0) return passo;
    for (int it = 0; it < 2; ++it) {
      Eigen::Vector3d n;
      double d;
      if (!ostacoloPiuVicino(da + passo, n, d)) break;
      const double verso = passo.dot(n);  // > 0: ci si avvicina
      const double ammesso = velocitaAvvicinamentoAmmessa(d) * dt;
      if (verso <= ammesso) break;
      passo -= (verso - ammesso) * n;
      filtro_attivo_ = true;
      filtro_normale_ = n;
      filtro_dist_ = d;
      ++filtro_interventi_;
    }
    // Stesso limite visto dal VELIVOLO (vedi filtraAnticipo): se il drone e'
    // gia' vicino a un ostacolo, il riferimento non deve trascinarlo verso di
    // esso piu' in fretta di quanto quella distanza consenta.
    Eigen::Vector3d nd;
    double dd;
    if (ostacoloPiuVicino(cur_pos_world_, nd, dd) && dd < safety_slow_dist_) {
      const double verso = passo.dot(nd);
      const double ammesso = velocitaAvvicinamentoAmmessa(dd) * dt;
      if (verso > ammesso) {
        passo -= (verso - ammesso) * nd;
        if (!filtro_attivo_ || dd < filtro_dist_) {
          filtro_normale_ = nd;
          filtro_dist_ = dd;
        }
        filtro_attivo_ = true;
        ++filtro_interventi_;
      }
    }
    return passo;
  }

  // Tetto di velocita' dato dall'ostacolo piu' vicino al velivolo (vedi
  // enable_proximity_speed_); infinito se lontano o se il limite e' spento.
  double velocitaVicinoOstacoli() const {
    if (!enable_proximity_speed_) return std::numeric_limits<double>::infinity();
    Eigen::Vector3d n;
    double d;
    if (!ostacoloPiuVicino(cur_pos_world_, n, d) || d >= prox_far_)
      return std::numeric_limits<double>::infinity();
    if (d <= prox_near_) return prox_v_min_;
    return prox_v_min_ + (d - prox_near_) * (prox_v_far_ - prox_v_min_) /
                             std::max(1e-3, prox_far_ - prox_near_);
  }

  // Velocita' da cui ci si ferma nello spazio libero dato:
  // v*latenza + v^2/(2a) = spazio.
  double velocitaFrenataAmmessa(double spazio) const {
    if (spazio <= 0.0) return 0.0;
    const double at = brake_gov_accel_ * brake_gov_latency_;
    return -at + std::sqrt(at * at + 2.0 * brake_gov_accel_ * spazio);
  }

  // Limite di frenata (vedi enable_brake_governor_): cerca il punto lidar verso
  // cui il velivolo supera di piu' la velocita' ammessa e, se c'e', toglie al
  // riferimento l'anticipo in quella direzione. Il limite sull'anticipo di
  // velocita' lo applica poi filtraAnticipo (gov_attivo_, gov_dir_, gov_vamm_).
  void governaFrenata() {
    gov_attivo_ = false;
    if (!enable_brake_governor_ || !ostacoli_validi_ || ostacoli_.empty()) return;
    const Eigen::Vector3d v = cur_vel_world_;
    const double vel = v.norm();
    if (vel < 0.2) return;
    const Eigen::Vector3d dir = v / vel;
    const double self2 = self_hit_radius_ * self_hit_radius_;
    const double tubo2 = brake_gov_tube_ * brake_gov_tube_;
    // Spazio libero lungo il percorso: il minimo, sui punti dentro il tubo, della
    // distanza a cui il tubo li tocca.
    double spazio = std::numeric_limits<double>::infinity();
    for (const auto& o : ostacoli_) {
      const Eigen::Vector3d r = o - cur_pos_world_;
      const double d2 = r.squaredNorm();
      if (d2 < self2) continue;
      const double avanti = r.dot(dir);
      if (avanti <= 0.0 || avanti > brake_gov_range_) continue;
      const double laterale2 = std::max(0.0, d2 - avanti * avanti);
      if (laterale2 >= tubo2) continue;
      spazio = std::min(spazio, avanti - std::sqrt(tubo2 - laterale2));
    }
    if (!std::isfinite(spazio)) return;
    const double vamm = velocitaFrenataAmmessa(spazio);
    if (vel <= vamm) return;
    gov_attivo_ = true;
    gov_dir_ = dir;
    gov_vamm_ = vamm;
    gov_dist_ = spazio;
    gov_vn_ = vel;
    ++gov_interventi_;
    const double anticipo = (ramp_pos_world_ - cur_pos_world_).dot(gov_dir_);
    if (anticipo > 0.0) ramp_pos_world_ -= anticipo * gov_dir_;
    rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000,
        "[Frenata] %.2f m/s con un ostacolo sul percorso fra %.2f m (ammesso %.2f): "
        "riferimento trattenuto (%lu interventi)",
        gov_vn_, gov_dist_, gov_vamm_, gov_interventi_);
  }

  // Stessa regola applicata all'ANTICIPO di velocita' (e accelerazione) che va
  // a PX4: se il riferimento non puo' avvicinarsi all'ostacolo, nemmeno il
  // termine in avanti deve spingere il velivolo in quella direzione.
  // Vettori in coordinate mondo ENU.
  static Eigen::Vector3d limitaVerso(const Eigen::Vector3d& v, const Eigen::Vector3d& n,
      double limite) {
    const double verso = v.dot(n);
    if (verso <= limite) return v;
    return v - (verso - limite) * n;
  }

  // Filtro sull'anticipo, con DUE riferimenti: l'ostacolo piu' vicino al
  // riferimento (quello che ha limitato il passo) e l'ostacolo piu' vicino al
  // VELIVOLO. Il secondo serve quando il drone e' piu' avanti del proprio
  // riferimento: nella run BASE_LIO 2 era 0.25 m oltre, verso la parete, e un
  // controllo sul solo riferimento non lo avrebbe visto.
  void filtraAnticipo(Eigen::Vector3d& vel, Eigen::Vector3d& acc) {
    if (!enable_safety_filter_) return;
    if (filtro_attivo_) {
      const double vamm = velocitaAvvicinamentoAmmessa(filtro_dist_);
      vel = limitaVerso(vel, filtro_normale_, std::isfinite(vamm) ? vamm : 1e9);
      acc = limitaVerso(acc, filtro_normale_, 0.0);
    }
    Eigen::Vector3d n;
    double d;
    if (ostacoloPiuVicino(cur_pos_world_, n, d) && d < safety_slow_dist_) {
      vel = limitaVerso(vel, n, velocitaAvvicinamentoAmmessa(d));
      acc = limitaVerso(acc, n, 0.0);
    }
    if (gov_attivo_) {
      vel = limitaVerso(vel, gov_dir_, gov_vamm_);
      acc = limitaVerso(acc, gov_dir_, 0.0);
      const double frenata =
          std::min(brake_gov_acc_max_, brake_gov_acc_gain_ * std::max(0.0, gov_vn_ - gov_vamm_));
      acc -= frenata * gov_dir_;
    }
    if (std::isfinite(v_vicino_) && vel.norm() > v_vicino_) {
      const double k = v_vicino_ / vel.norm();
      vel *= k;
      acc *= k;
    }
  }

  // NIENTE ANTICIPO VERSO IL BASSO AL PAVIMENTO. La quota minima (min_flight_z)
  // si impone alla POSIZIONE del riferimento; velocita' e accelerazione del
  // pianificatore passavano intatte. Con il piano sotto il pavimento il ponte
  // teneva la posizione a min_flight_z ma mandava a PX4 vz < 0, e con MPC_Z_P = 1
  // PX4 si assesta a |vz| metri SOTTO il riferimento: run 2026-10-18_1248 (parco)
  // riferimento a 0.61 m (quota PX4) con vz -0.46 e az -1.05, drone a 0.13 m.
  // Le stesse componenti verso il basso le aggiungeva il filtro di sicurezza
  // sotto un ostacolo sovrastante (limitaVerso toglie la componente verso
  // l'ostacolo, cioe' aggiunge discesa): parco giochi e chiome nelle run 1320,
  // 1103, 1153, 0054, drone fra 0.38 e 0.43 m.
  // Qui: se il riferimento e' a meno di 5 cm dal pavimento, o il drone gia' sotto,
  // si tolgono le componenti verso il basso di velocita' e accelerazione. Sopra
  // il pavimento non cambia nulla.
  void anticipoAlPavimento(const Eigen::Vector3d& cmd_world, Eigen::Vector3d& vel,
      Eigen::Vector3d& acc) {
    if (crossing_ || min_flight_z_ <= 0.0) return;
    if (cmd_world.z() > min_flight_z_ + 0.05 && cur_pos_world_.z() >= min_flight_z_) return;
    if (vel.z() >= 0.0 && acc.z() >= 0.0) return;
    vel.z() = std::max(0.0, vel.z());
    acc.z() = std::max(0.0, acc.z());
    ++quota_anticipi_tolti_;
    rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 2000,
        "[Quota] riferimento a %.2f m, drone a %.2f m (pavimento %.2f): anticipo verso il "
        "basso tolto (%lu volte; piano sotto il pavimento %lu volte, minimo %.2f m; "
        "riferimento trattenuto %lu volte)",
        cmd_world.z(), cur_pos_world_.z(), min_flight_z_, quota_anticipi_tolti_,
        quota_piano_sotto_, quota_piano_min_, quota_trattenute_);
  }

  void riepilogoQuota() {
    RCLCPP_INFO(this->get_logger(),
        "[Quota] riepilogo: piano sotto il pavimento %.2f m per %.1f s (minimo %.2f m), "
        "anticipo verso il basso tolto %lu volte, riferimento trattenuto %lu volte",
        min_flight_z_, quota_piano_sotto_ * 0.01,
        std::isfinite(quota_piano_min_) ? quota_piano_min_ : min_flight_z_,
        quota_anticipi_tolti_, quota_trattenute_);
  }

  void onPeerState(const c2x_msgs::msg::DroneState& msg) {
    if (msg.drone_id == drone_id_ || msg.drone_id < 1) return;
    if (msg.pos.size() < 3 || msg.vel.size() < 3) return;
    PeerState& p = peers_[msg.drone_id];
    p.pos = Eigen::Vector3d(msg.pos[0], msg.pos[1], msg.pos[2]);
    p.vel = Eigen::Vector3d(msg.vel[0], msg.vel[1], msg.vel[2]);
    p.recv = this->now();
  }

  // Compagno APPOGGIATO A TERRA mentre questo drone vola sopra di lui (sciame, ottobre 2026).
  // Le partenze dello sciame stanno in fila sullo stesso corridoio: chi rientra per ultimo ci
  // passa sopra a quelli gia' atterrati, e la fermata per compagno (distanza 3D sotto 1.5 m)
  // con la manovra di stallo lo tenevano bloccato fino allo scadere del rientro, poi atterrava
  // fuori casa (cubicle_office, 73 s di "PEER STOP"/"STALLO" con un drone fermo a terra). Un
  // drone a terra e' un ostacolo fermo: lo vedono mappa e filtro di sicurezza
  // (cloud_transform_bridge non toglie i compagni fermi dalla nuvola).
  // Basta essere 0.4 m piu' in alto OPPURE a piu' di 0.6 m in orizzontale: con la sola
  // condizione sulla quota, chi scendeva sulla corsia bassa dell'incrocio (0.7 m) per scansare
  // un compagno che atterrava tornava a vederlo come ostacolo in volo, e restava in stallo
  // accanto a lui fino allo scadere del rientro (office_small, ottobre 2026).
  bool compagnoATerraSotto(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) const {
    if (pos.z() >= 0.6 || vel.norm() >= 0.2) return false;
    return cur_pos_world_.z() > pos.z() + 0.4 ||
        (cur_pos_world_ - pos).head<2>().norm() > 0.6;
  }

  // Closest approach to any peer over the next peer_lookahead seconds, assuming
  // both hold their current velocity. Straight-line extrapolation is crude, but
  // over a one-second horizon at these speeds it is exactly the regime that
  // matters, and it costs nothing.
  double closestPeerDistance(int& closest_id) const {
    closest_id = -1;
    double closest = std::numeric_limits<double>::infinity();
    const auto now = this->now();
    for (const auto& kv : peers_) {
      const PeerState& p = kv.second;
      if ((now - p.recv).seconds() > peer_state_timeout_) continue;  // stale, ignore
      if (compagnoATerraSotto(p.pos, p.vel)) continue;
      const Eigen::Vector3d rel_pos = p.pos - cur_pos_world_;
      const Eigen::Vector3d rel_vel = p.vel - cur_vel_world_;

      double d = rel_pos.norm();
      // Time of closest approach along the relative velocity, clamped to the
      // lookahead window (negative means they are already separating).
      const double vv = rel_vel.squaredNorm();
      if (vv > 1e-6) {
        const double t_ca = std::clamp(-rel_pos.dot(rel_vel) / vv, 0.0, peer_lookahead_);
        d = std::min(d, (rel_pos + t_ca * rel_vel).norm());
      }
      if (d < closest) {
        closest = d;
        closest_id = kv.first;
      }
    }
    return closest;
  }

  // INCROCIO IN QUOTA: invece di fermarsi, ci si scansa in verticale.
  //
  // Due droni che si vengono incontro in un corridoio oggi si fermano entrambi
  // (o si ferma quello con id piu' alto) e ripartono solo quando il
  // pianificatore instrada uno dei due altrove - il commento in fondo a
  // updatePeerSeparation dice apertamente che non c'e' via d'uscita a tempo,
  // perche' rilasciarli su un timer li rimanderebbe uno addosso all'altro.
  // In un passaggio stretto non esiste una via d'uscita ORIZZONTALE: le pareti
  // sono li'. Ma ne esiste una verticale, e non richiede negoziazione - la
  // regola e' gia' condivisa e deterministica, la stessa precedenza per id che
  // questo strato usa gia': l'id piu' basso sale, l'altro scende. Entrambi
  // calcolano lo stesso esito senza scambiarsi un solo messaggio in piu', che e'
  // esattamente lo spirito con cui e' fatta la ripartizione delle celle.
  //
  // Si attiva solo quando serve davvero:
  //   * ci si avvicina (non ci si sta gia' allontanando);
  //   * si va l'uno CONTRO l'altro, non affiancati - il prodotto scalare delle
  //     velocita' orizzontali e' negativo;
  //   * si e' alla STESSA quota entro cross_z_band, altrimenti ci si passa
  //     gia' sopra e sotto e non c'e' niente da risolvere.
  // Lo spazio verticale e' poco: la scena e' alta 3 m, la quota minima di volo
  // e' 1.0 m e si vola a ~0.95-1.0 m. Chi scende trova subito il pavimento, per
  // questo lo scarto utile lo fa soprattutto chi sale. Con 0.7 m si arriva a
  // ~1.7 m contro il soffitto a 2.5: margine sufficiente, e la separazione
  // verticale che ne esce e' comunque piu' del diametro del velivolo.
  // Il compagno con cui si e' in STALLO: vicino, alla stessa quota, e con
  // nessuno dei due che riesce ad avanzare.
  //
  // L'incrocio in quota NON e' la risposta a un incontro qualsiasi - e' la
  // risposta a un BLOCCO. Innescandolo a ogni avvicinamento si ottengono
  // centinaia di manovre inutili (misurate: 156 in quattro run, 62 delle quali
  // scadute senza sorpasso) che deviano i droni anche quando il pianificatore
  // avrebbe risolto da solo. Uno stallo invece il pianificatore non lo risolve,
  // perche' ciascuno aspetta che si sposti l'altro.
  // Lo stallo si riconosce cosi': un pari entro la distanza di innesco, alla
  // stessa quota, e questo drone fermo - o perche' la fermata per compagno lo
  // tiene, o perche' semplicemente non avanza. La condizione deve DURARE
  // cross_stall_time: un rallentamento di un istante non e' un blocco.
  // Non si richiede piu' che le rotte siano frontali. Gli incontri lunghi senza
  // sorpasso - due droni che si affiancano e si ostacolano senza mai passarsi,
  // 11.6 s misurati con 0.02 m di dislivello - sono stalli a tutti gli effetti,
  // ed erano proprio quelli che il criterio frontale lasciava fuori.
  int compagnoInStallo(double& dist_out) {
    const auto now = this->now();
    int scelto = -1;
    double piu_vicino = std::numeric_limits<double>::infinity();
    for (const auto& kv : peers_) {
      const PeerState& p = kv.second;
      if ((now - p.recv).seconds() > peer_state_timeout_) continue;
      if (compagnoATerraSotto(p.pos, p.vel)) continue;
      const Eigen::Vector3d rel_pos = p.pos - cur_pos_world_;
      const double d = rel_pos.norm();
      if (d > cross_trigger_dist_ || d >= piu_vicino) continue;

      // "GIA' SEPARATI" SI GIUDICA SULLE CORSIE, NON SUL DISLIVELLO ISTANTANEO.
      //
      // Il criterio precedente scartava il conflitto quando il dislivello
      // superava cross_z_band. Sembra sensato e invece si autoannulla: appena il
      // PRIMO drone comincia a salire, il dislivello supera la soglia e il
      // secondo conclude "siamo gia' su piani diversi" e non parte mai.
      // Misurato: 301 campioni comandati sulla corsia alta contro 23 sulla
      // bassa, cioe' si muoveva un drone solo, e la separazione al passaggio
      // restava 1.30 - 0.95 = 0.35 m invece del metro previsto.
      // Il giudizio giusto e': sono gia' NELLA MIA corsia e siamo abbastanza
      // distanti in quota? Allora non c'e' niente da fare. Altrimenti il
      // conflitto resta aperto anche se l'altro si e' gia' mosso - anzi,
      // soprattutto allora, perche' tocca a me completare la separazione.
      const double mia_corsia = (drone_id_ < kv.first) ? cross_high_z_ : cross_low_z_;
      const bool sono_in_corsia = std::abs(cur_pos_world_.z() - mia_corsia) < 0.15;
      if (sono_in_corsia && std::abs(rel_pos.z()) > cross_clear_dz_) continue;
      piu_vicino = d;
      scelto = kv.first;
    }

    // BLOCCATO DAL COMPAGNO, non semplicemente lento.
    //
    // Accettare "la mia velocita' e' sotto soglia" come prova di stallo e'
    // troppo permissivo: i droni stanno fermi moltissimo per ragioni proprie -
    // pianificano, aspettano che l'allocazione dia loro delle celle - e con un
    // raggio di innesco di 5 m in una stanza di 15 c'e' quasi sempre un
    // compagno nel raggio. Il drone 1 innescava manovre di continuo perche' era
    // LENTO, non perche' fosse bloccato, e avendo l'id piu' basso si prendeva
    // sempre la corsia alta: misurati 443 campioni comandati in alto contro 36
    // in basso, cioe' una manovra a senso unico che non separa nessuno.
    // Servono due droni fermi UNO CONTRO L'ALTRO. La fermata per compagno resta
    // il segnale piu' netto perche' dice esattamente questo; in alternativa si
    // richiede che siano lenti ENTRAMBI.
    const Eigen::Vector2d v_mia(cur_vel_world_.x(), cur_vel_world_.y());
    bool bloccato = false;
    if (scelto > 0) {
      const auto it_s = peers_.find(scelto);
      const double v_sua = (it_s == peers_.end())
          ? 0.0 : Eigen::Vector2d(it_s->second.vel.x(), it_s->second.vel.y()).norm();
      bloccato = peer_hold_ ||
          (v_mia.norm() < cross_min_speed_ && v_sua < cross_min_speed_);
    }

    if (!bloccato || scelto != stallo_peer_) {
      stallo_peer_ = bloccato ? scelto : -1;
      stallo_da_ = now;
      dist_out = piu_vicino;
      return -1;
    }
    dist_out = piu_vicino;
    // Lo stesso pari blocca da abbastanza tempo: e' uno stallo.
    return ((now - stallo_da_).seconds() >= cross_stall_time_) ? scelto : -1;
  }

  void updateCrossing(int peer_id, double d) {
    const auto now = this->now();
    if (crossing_) {
      // FINE MANOVRA: si torna alla quota di crociera.
      //
      // Non serve comandarla esplicitamente: smettendo di imporre crossing_z_,
      // il bersaglio torna a essere quello del pianificatore, che vola a ~1 m,
      // e la quota minima di volo (1.0 m) torna a valere subito - quindi chi era
      // sceso a 0.50 risale e chi era salito a 1.50 ridiscende. Il rientro passa
      // dalla stessa rampa di tutto il resto, quindi e' graduale e non produce
      // il gradino nel riferimento che ha gia' causato dei ribaltamenti.
      // L'uscita e' a peer_resume_dist (2 m), cioe' a sorpasso AVVENUTO: uscire
      // prima rimetterebbe i due alla stessa quota mentre si stanno ancora
      // passando accanto.
      // L'USCITA DEVE STARE OLTRE L'INGRESSO, ALTRIMENTI LA MANOVRA NON ESISTE.
      //
      // Usciva a peer_resume_dist (2 m) mentre l'innesco e' a cross_trigger_dist
      // (5 m): la condizione di uscita era GIA' VERA all'ingresso, quindi la
      // manovra durava un tick e si annullava. Misurato: 509 innesci e 509
      // rientri a quota di crociera nella stessa campagna, separazione verticale
      // effettiva invariata a 0.11 m. Il drone riceveva la quota di corsia per
      // una frazione di secondo e tornava subito a un metro.
      // Ora si esce quando i due si stanno DAVVERO separando - prodotto scalare
      // fra posizione e velocita' relative positivo, cioe' sorpasso avvenuto - e
      // sono oltre la distanza di innesco. Cosi' la corsia si tiene per tutto
      // l'incontro, che e' l'unico modo perche' serva a qualcosa.
      const bool scaduto = (now - crossing_start_).seconds() > cross_timeout_;
      bool sorpassato = false;
      const auto it_c = peers_.find(crossing_peer_);
      if (it_c != peers_.end()) {
        const Eigen::Vector3d rp = it_c->second.pos - cur_pos_world_;
        const Eigen::Vector3d rv = it_c->second.vel - cur_vel_world_;
        sorpassato = rp.dot(rv) > 0.0 && rp.norm() > cross_trigger_dist_;
      }
      if (sorpassato || scaduto || peer_id != crossing_peer_) {
        crossing_ = false;
        crossing_z_ = 0.0;
        RCLCPP_WARN(this->get_logger(),
            "Incrocio concluso con drone %d (%.2fm) - rientro a quota di crociera%s",
            crossing_peer_, d, scaduto ? " (tempo scaduto)" : "");
      }
      return;
    }
    if (!enable_crossing_) return;

    double d_conf = 0.0;
    const int conflitto = compagnoInStallo(d_conf);
    if (conflitto < 0) return;

    // DUE SOLE CORSIE, E SOLO PER I DUE BLOCCATI.
    //
    // Fuori dalla manovra si vola tutti a un metro; qui uno scende a 0.50 e
    // l'altro sale a 1.50, un metro netto di separazione contro ~0.5 m di
    // larghezza del velivolo. Chi non e' coinvolto nello stallo non si muove.
    // Le quote sono ASSOLUTE e la quota minima di volo (1.0 m) viene scavalcata
    // apposta per la corsia bassa: e' il motivo per cui le versioni a scarto
    // RELATIVO fallivano tutte. Con +-0.7 m chi doveva scendere veniva ripreso
    // subito dal minimo - misurato, 44 "salgo" contro 85 "scendo" - e restava
    // alla stessa quota dell'altro con la fermata per compagno gia' spenta.
    // Scendere sotto il minimo e' un rischio calcolato e limitato dal tempo
    // massimo di manovra: si accetta per l'incrocio, non come regime.
    crossing_ = true;
    crossing_peer_ = conflitto;
    crossing_start_ = now;
    crossing_z_ = (drone_id_ < conflitto) ? cross_high_z_ : cross_low_z_;
    RCLCPP_WARN(this->get_logger(),
        "STALLO con drone %d a %.2fm - sblocco: vado a quota %.2fm (%s)", conflitto,
        d_conf, crossing_z_, crossing_z_ > cur_pos_world_.z() ? "sopra" : "sotto");
  }

  void updatePeerSeparation() {
    if (!odomFresh() || !enable_peer_hold_ || peers_.empty()) return;

    int peer_id = -1;
    const double d = closestPeerDistance(peer_id);
    if (peer_id < 0) {
      peer_hold_ = false;
      crossing_ = false;
      crossing_z_ = 0.0;
      return;
    }

    updateCrossing(peer_id, d);

    // La fermata si toglie SOLO A SEPARAZIONE OTTENUTA, non appena si decide di
    // salire. E' la lezione della prima versione: dare per buona una manovra
    // solo perche' e' stata decisa significa spegnere l'unica protezione
    // disponibile mentre il drone e' ancora esattamente sulla rotta dell'altro.
    // La salita non e' istantanea - passa dalla rampa, ~0.7 m a 1.6 m/s - e per
    // tutto quel tempo i due si stanno ancora avvicinando.
    // Quindi si guarda il dislivello VERO fra i due velivoli, non l'intenzione.
    if (crossing_ && d > peer_hard_stop_dist_) {
      const auto it = peers_.find(crossing_peer_);
      const bool separati = it != peers_.end() &&
          std::abs(it->second.pos.z() - cur_pos_world_.z()) > cross_clear_dz_;
      if (separati) {
        peer_hold_ = false;
        return;
      }
    }

    if (!peer_hold_ && d < peer_stop_dist_) {
      // Right of way by drone id, exactly as the FSM's trajectory-level check
      // resolves it - but only down to peer_hard_stop_dist. Inside that, both
      // sides stop unconditionally: a priority rule only helps if the other
      // drone is actually acting on it, and this layer exists precisely for the
      // cases where something upstream did not.
      if (drone_id_ < peer_id && d > peer_hard_stop_dist_) return;
      peer_hold_ = true;
      peer_hold_start_ = this->now();
      hold_pos_world_ = brakingHoldPoint(d - peer_hard_stop_dist_);
      RCLCPP_ERROR(this->get_logger(),
          "PEER STOP: drone %d closing to %.2fm (stop %.2fm, hard %.2fm) - holding position",
          peer_id, d, peer_stop_dist_, peer_hard_stop_dist_);
    } else if (peer_hold_ && d > peer_resume_dist_ &&
               (this->now() - peer_hold_start_).seconds() >= min_hold_time_) {
      peer_hold_ = false;
      RCLCPP_WARN(this->get_logger(), "Peer clear (%.2fm from drone %d) - resuming planner", d,
          peer_id);
    }
    // Deliberately no timeout escape here, unlike the lidar hold: two drones
    // that both stopped are separated and safe, and whichever one the planner
    // routes away first breaks the symmetry. Releasing on a timer would simply
    // fly them back at each other.
  }

  void onPositionCommand(const c2x_msgs::msg::PositionCommand& cmd) {
    have_cmd_ = true;
    last_status_ = cmd.trajectory_flag;

    if (holdActive()) {
      // Ignore whatever the planner wants while something is too close - publishTrajectory-
      // Setpoint() below publishes the frozen hold position regardless of what's stored
      // here, but skip updating it too so there's nothing stale to resume into.
      return;
    }

    if (cmd.trajectory_flag == c2x_msgs::msg::PositionCommand::TRAJECTORY_STATUS_EMPTY ||
        cmd.trajectory_flag == c2x_msgs::msg::PositionCommand::TRAJECTROY_STATUS_ABORT) {
      // Hold last valid setpoint rather than forwarding a garbage/aborted one.
      return;
    }

    // World ENU -> vehicle-local ENU (identity when this drone spawned at the
    // world origin), then ENU -> NED for PX4.
    Eigen::Vector3d pos_enu = worldToLocal(
        Eigen::Vector3d(cmd.position.x, cmd.position.y, cmd.position.z));
    Eigen::Vector3d vel_enu =
        spawn_rot_inv_ * Eigen::Vector3d(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
    Eigen::Vector3d acc_enu =
        spawn_rot_inv_ * Eigen::Vector3d(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);

    last_pos_world_ = Eigen::Vector3d(cmd.position.x, cmd.position.y, cmd.position.z);
    // Misura per la diagnosi: quanto spesso e quanto in basso il PIANO scende sotto
    // il pavimento (il riferimento lo tiene comunque a min_flight_z), riportato
    // nei messaggi [Quota]. Un comando ogni 10 ms da traj_server.
    if (takeoff_done_ && !crossing_ && min_flight_z_ > 0.0 &&
        cmd.position.z < min_flight_z_ - 0.10) {
      ++quota_piano_sotto_;
      quota_piano_min_ = std::min(quota_piano_min_, static_cast<double>(cmd.position.z));
    }
    last_vel_world_ = Eigen::Vector3d(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
    last_acc_world_ = Eigen::Vector3d(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);
    last_pos_ned_ = px4_ros_com::frame_transforms::enu_to_ned_local_frame(pos_enu);
    last_vel_ned_ = px4_ros_com::frame_transforms::enu_to_ned_local_frame(vel_enu);
    last_acc_ned_ = px4_ros_com::frame_transforms::enu_to_ned_local_frame(acc_enu);
    // ENU yaw (about +Z, zero at +X/East) -> NED yaw (about +Z/Down, zero at +X/North)
    last_yaw_ned_ = static_cast<float>(M_PI_2 - (cmd.yaw - spawn_yaw_));
    have_setpoint_ = true;
  }

  // True while we are flying ourselves up to takeoff_height because the planner
  // has not produced a trajectory yet.
  bool takingOff() {
    if (takeoff_height_ <= 0.0 || !have_odom_) return false;
    // IL DECOLLO NON SI RIARMA.
    //
    // La condizione di fondo e' una DIFFERENZA DI QUOTA, quindi tornava vera da
    // sola ogni volta che il velivolo, gia' in missione da minuti, scendeva
    // sotto mezzo metro dal bersaglio di decollo. E il ramo di decollo pubblica
    // takeoff_target_world_ DI PESO, senza passare da rampAndClamp: e' l'unico
    // percorso che scavalca sia la rampa sia il limite di scostamento
    // (max_setpoint_jump, 1 m).
    // Cosa produceva, letto sui registri di volo di tre ribaltamenti su tre:
    // un drone urtato da un compagno perde quota; appena scende sotto la soglia
    // il ponte gli comanda di colpo di rientrare al punto di decollo, che a
    // quel punto e' a 4 m di distanza. Nel registro il riferimento salta a
    // (-0.05, -0.02) - l'origine locale, cioe' proprio il punto di partenza -
    // con 4.12 m di errore, e il velivolo accelera a 3-5 m/s contro i 0.6 m/s
    // del pianificatore, perde portanza inclinandosi e tocca terra. L'inclinazione
    // a 60 gradi arriva DOPO il contatto: era la conseguenza, non la causa.
    // Un velivolo gia' in volo che perde quota va stabilizzato dov'e', non
    // rispedito alla partenza scavalcando ogni vincolo di continuita'. Dopo il
    // decollo riuscito ci pensano i rami normali, che la rampa la usano.
    if (takeoff_done_) return false;
    // Comando ceduto a PX4 a decollo o giro in corso (controllaModalitaPx4): il
    // decollo non e' piu' del ponte. Falso anche per il ramo di riarmo in
    // tick(), e niente quota raggiunta o giro scritti nel log mentre PX4 atterra.
    if (ceduto_a_px4_) return false;
    // Quota raggiunta la prima volta: comincia il giro iniziale (vedi
    // initial_spin_rate). Indipendente dall'arrivo dei comandi del
    // pianificatore, che anzi aspetta la fine del giro per partire.
    if (!quota_raggiunta_ &&
        (takeoff_target_world_.z() - cur_pos_world_.z()) <= takeoff_done_tol_) {
      quota_raggiunta_ = true;
      giro_inizio_ = this->now();
      giro_yaw0_ = last_yaw_ned_;
      if (initial_spin_rate_ > 0.0) {
        RCLCPP_INFO(this->get_logger(), "[Giro] quota %.2fm raggiunta: giro iniziale di 360 gradi "
            "a %.2f rad/s", cur_pos_world_.z(), initial_spin_rate_);
      }
    }
    // Il decollo finisce quando si e' RAGGIUNTA la quota, non quando il
    // pianificatore comincia a parlare.
    //
    // Con la sola condizione !have_setpoint_ il velivolo abbandonava la salita
    // al primo comando ricevuto: si spostava in orizzontale da mezzo metro, e
    // inclinandosi per accelerare perdeva la poca spinta verticale che aveva.
    // Misurato su due ribaltamenti a 15 e 23 s dall'avvio: quota comandata 0.90,
    // quota reale 0.33-0.50 con 1 m/s di velocita' orizzontale, fino al contatto
    // col suolo. In un caso il drone era ancora a terra e non si e' mai alzato.
    // La quota si tiene finche' non si e' arrivati entro mezzo metro dal
    // bersaglio: da li' in poi il pianificatore ha un velivolo in volo vero.
    if (!have_setpoint_) return true;
    // Tolleranza ristretta da 0.5 a takeoff_done_tol (0.15 m).
    //
    // Con 0.5 m il decollo si dichiarava concluso a META' quota: nelle run
    // BASE_GPS il log riporta "Decollo concluso a 0.50m", e da li' il
    // pianificatore cominciava a spostare in orizzontale un velivolo ancora a
    // mezzo metro da terra, sotto il piano delle scrivanie (0.75 m) - proprio il
    // regime che il commento qui sopra descrive come causa di ribaltamenti.
    if ((takeoff_target_world_.z() - cur_pos_world_.z()) > takeoff_done_tol_) return true;
    // Il giro iniziale non si interrompe: se un comando arriva prima della fine
    // (via dato senza aspettare volo_pronto) si finisce comunque il giro.
    if (!giro_concluso_) return true;
    // Quota raggiunta: da qui in avanti il decollo e' concluso per sempre.
    takeoff_done_ = true;
    RCLCPP_INFO(this->get_logger(), "Decollo concluso a %.2fm - il ramo di decollo non si riarma",
        cur_pos_world_.z());
    return false;
  }

  // PX4 ESCE DA SOLO DALL'OFFBOARD A VOLO INIZIATO: il ponte cede il comando.
  //
  // /fmu/in/trajectory_setpoint non e' un canale riservato all'offboard: il
  // ponte uXRCE-DDS lo scrive nello STESSO topic uORB trajectory_setpoint in
  // cui scrivono i compiti di volo di PX4 (atterraggio, ritorno, attesa). Con
  // PX4 in LAND e il ponte che continuava a pubblicare, il controllore riceveva
  // a turno "scendi" (compito di atterraggio) e "resta a 0.77 m" (questo
  // ponte): prova generale del 19 ottobre 2026 (run 2026-10-19_1033_reale-sim,
  // NAV_LAND mandato da fuori), drone appoggiato ma atterraggio non
  // riconosciuto per 60 s, con i motori accesi, fino allo spegnimento del ponte.
  // Il ponte smetteva solo per gli atterraggi chiesti a lui (landing_); un
  // failsafe di PX4 (batteria critica, stima persa, offboard perso con
  // COM_OBL_RC_ACT = atterraggio) o un NAV_LAND di terzi restavano contesi.
  // Sul drone vero lo aggirava sorveglia_missione.py, le run simulate no.
  //
  // Regola: PX4 visto almeno una volta armato in OFFBOARD
  // (visto_offboard_armato_, cioe' armamento riuscito), velivolo armato e PX4
  // in una modalita' qualsiasi diversa da OFFBOARD = PX4 (o qualcun altro) ha
  // preso il volo. Il ponte stesso, una volta armato, cambia modalita' solo
  // con NAV_LAND, e in quel caso landing_ e' gia' vero.
  // Da li' in poi niente setpoint e niente battito, per sempre: niente ritorno
  // all'offboard nemmeno se PX4 ci tornasse (un ripiego per offboard perso si
  // annulla da solo quando il battito riprende), niente avvicinamento a casa,
  // niente riarmo (vedi tick()). Nei 116 registri .ulg archiviati (simulazioni
  // fino al 19 ottobre) PX4 non e' mai uscito da solo dall'offboard in volo:
  // la prima modalita' diversa da OFFBOARD ad armato e' sempre un LAND chiesto
  // al ponte.
  // L'FSM viene avvisata con flight_terminated (esplorazione chiusa, niente
  // land_at_home); land_request resta valido e manda NAV_LAND (stima
  // degradata, sorvegliante), cioe' atterraggio sul posto anche da un RTL.
  //
  // DA DECOLLO CONCLUSO AD ARMAMENTO RIUSCITO (19 ottobre 2026). La prima
  // versione scattava solo dopo takeoff_done_: durante la salita e il giro
  // iniziale un failsafe LAND/DESCEND o un NAV_LAND di terzi restavano contesi
  // come prima, e peggio, a velivolo disarmato da PX4 il ramo di riarmo di
  // tick() (decollo ancora "in corso") mandava ogni secondo DO_SET_MODE
  // offboard + ARM e lo rimetteva in volo. Sul drone vero lo copriva solo
  // sorveglia_missione.py (stack spento se il decollo non si conclude).
  // Cosa NON cambia: prima dell'armamento in offboard il ponte pubblica il
  // battito e i setpoint di decollo come sempre (PX4 rifiuta l'offboard senza
  // un flusso gia' in corso), e la sequenza DO_SET_MODE + ARM con i suoi
  // tentativi e' identica. Il primo stato armato e' sempre OFFBOARD (118 run
  // armate su 118 nei registri di 17_P e 18_P; in 110 PX4 e' gia' in OFFBOARD
  // da disarmato): la condizione diventa vera all'armamento, non prima, e un
  // armamento accettato in un'altra modalita' (offboard rifiutato) non la
  // rende vera.
  void controllaModalitaPx4() {
    if (ceduto_a_px4_ || landing_ || !visto_offboard_armato_ || !armed_) return;
    if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) return;
    ceduto_a_px4_ = true;
    ceduto_dal_ = this->now();
    rientro_preciso_ = false;
    const char* fase = takeoff_done_ ? "in volo"
        : (quota_raggiunta_ ? "nel giro iniziale" : "in decollo");
    RCLCPP_ERROR(this->get_logger(),
        "[PX4] uscito da solo dall'offboard %s (modalita' %s, quota %.2fm): smetto di "
        "pubblicare setpoint e battito, il volo lo comanda PX4 - missione terminata",
        fase, nomeModalita(nav_state_).c_str(), cur_pos_world_.z());
    riepilogoQuota();
    flight_terminated_pub_->publish(std_msgs::msg::Empty());
  }

  static std::string nomeModalita(uint8_t s) {
    using VS = px4_msgs::msg::VehicleStatus;
    switch (s) {
      case VS::NAVIGATION_STATE_AUTO_LAND: return "LAND";
      case VS::NAVIGATION_STATE_DESCEND: return "DESCEND";
      case VS::NAVIGATION_STATE_AUTO_RTL: return "RTL";
      case VS::NAVIGATION_STATE_AUTO_LOITER: return "HOLD";
      case VS::NAVIGATION_STATE_POSCTL: return "POSCTL";
      case VS::NAVIGATION_STATE_ALTCTL: return "ALTCTL";
      case VS::NAVIGATION_STATE_STAB: return "STAB";
      case VS::NAVIGATION_STATE_MANUAL: return "MANUAL";
      case VS::NAVIGATION_STATE_TERMINATION: return "TERMINATION";
      default: return "nav_state " + std::to_string(s);
    }
  }

  void tick() {
    controllaModalitaPx4();
    updatePeerSeparation();
    publishOffboardControlMode();

    if (have_setpoint_ || takingOff() || (holdActive() && have_odom_) || rientro_preciso_) {
      publishTrajectorySetpoint();
    }
    controllaArrivoACasa();

    // In atterraggio PX4 comanda da solo: continuare a mandare setpoint
    // offboard o a riarmare lo farebbe uscire dalla modalita' LAND.
    if (landing_) return;

    // NIENTE RIARMO DOPO IL DECOLLO.
    //
    // Il ciclo qui sotto riarma ogni secondo un velivolo disarmato, e dopo il
    // decollo e' sbagliato: PX4 disarma da solo quando rileva un atterraggio
    // (COM_DISARM_LAND), quindi un drone che si e' appoggiato su un piano o e'
    // caduto verrebbe riarmato e rimesso in volo verso l'ultimo setpoint -
    // partendo da terra, cioe' traslando prima di avere quota, lo scenario di
    // ribaltamento descritto in takingOff(). Una volta in volo, un disarmo
    // chiude la missione: lo si annuncia all'FSM e non si riarma piu'.
    // Lo stesso dopo il comando ceduto a PX4 (controllaModalitaPx4), anche a
    // decollo o giro in corso: senza ceduto_a_px4_ qui il disarmo a fine
    // atterraggio di PX4 cadeva nel ramo di riarmo qui sotto (takingOff() era
    // ancora vero) e il velivolo ripartiva dal suolo.
    if (armed_ && takeoff_done_) ha_volato_ = true;
    if ((ha_volato_ || ceduto_a_px4_) && !armed_) {
      if (!volo_terminato_) {
        volo_terminato_ = true;
        if (ceduto_a_px4_) {
          // Fine attesa del volo comandato da PX4 (controllaModalitaPx4).
          // "atterraggio" nel testo: run_multi_exploration.sh conta le righe
          // con "atterra" per sapere che il drone e' a terra.
          RCLCPP_WARN(this->get_logger(),
              "[PX4] disarmato %.1f s dopo l'uscita dall'offboard (quota %.2fm): atterraggio "
              "di PX4 concluso, non lo riarmo", (this->now() - ceduto_dal_).seconds(),
              cur_pos_world_.z());
        } else {
          RCLCPP_ERROR(this->get_logger(),
              "DISARMATO IN VOLO: PX4 ha disarmato il velivolo a missione iniziata (quota %.2fm) - "
              "non lo riarmo, missione terminata", cur_pos_world_.z());
        }
        flight_terminated_pub_->publish(std_msgs::msg::Empty());
      }
      return;
    }

    if ((have_cmd_ || takingOff()) && !armed_) {
      // Same bring-up sequence as px4_ros_com's offboard_control.cpp example:
      // a few setpoints must precede the mode switch/arm for PX4 to accept
      // offboard mode. Retried every second while still disarmed instead of
      // fired exactly once at tick 10 - PX4 rejects the arm request outright if
      // any pre-arm check is still failing at that instant (EKF not converged,
      // GPS/home position not set yet), which is easy to hit while three SITL
      // instances start up together, and the old `== 10` meant a single
      // rejection left the drone sitting disarmed forever. It also covers a
      // disarm later in the flight.
      ++setpoint_counter_;
      if (setpoint_counter_ >= 10 && setpoint_counter_ % 10 == 0) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
      }
    } else if (armed_) {
      setpoint_counter_ = 0;
    }
  }

  void publishOffboardControlMode() {
    if (landing_ || ceduto_a_px4_) return;
    px4_msgs::msg::OffboardControlMode msg{};
    // Velocity feedforward, matching what upstream C2-Explorer's controller
    // actually tracks. Its cascadePID consumes position, velocity AND
    // acceleration from every PositionCommand; sending PX4 position alone
    // leaves the vehicle with a steady-state lag of roughly v / MPC_XY_P, about
    // a metre at 1 m/s. That is the same order as optimization.swarm_safe_dist
    // (1.2m), so two drones the optimizer separated correctly can be far closer
    // in reality - the planner reasons about where the drone should be, the
    // avoidance margins are consumed by where it actually is.
    //
    // With position=true AND velocity=true PX4 treats the velocity setpoint as
    // feedforward to its position controller rather than as an independent
    // target, which is exactly the cascade upstream implements. Acceleration is
    // deliberately left off: an earlier session found position+velocity+
    // acceleration flew erratically, and the acceleration term is the one
    // numerically differentiated twice off a B-spline that gets replanned every
    // cycle. Set use_velocity_feedforward:=false to go back to position-only.
    msg.position = true;
    msg.velocity = use_velocity_feedforward_;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
  }

  void publishTrajectorySetpoint() {
    if (landing_ || ceduto_a_px4_) return;
    px4_msgs::msg::TrajectorySetpoint msg{};
    if (have_odom_ && !odomFresh() && ramp_valid_) {
      // Odometria scaduta: non si sa piu' dove sia il velivolo, quindi si tiene
      // fermo l'ultimo riferimento valido invece di calcolarne uno nuovo su una
      // posizione vecchia. Ricalcolarlo e' peggio che non muoversi: il clamp
      // misurerebbe l'errore rispetto a un punto che il drone ha gia' lasciato.
      const Eigen::Vector3d frozen_ned =
          px4_ros_com::frame_transforms::enu_to_ned_local_frame(worldToLocal(ramp_pos_world_));
      msg.position = { static_cast<float>(frozen_ned.x()), static_cast<float>(frozen_ned.y()),
        static_cast<float>(frozen_ned.z()) };
      msg.velocity = { 0.0f, 0.0f, 0.0f };
      msg.acceleration = { 0.0f, 0.0f, 0.0f };
      msg.yaw = last_yaw_ned_;
      msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      trajectory_setpoint_pub_->publish(msg);
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000,
          "Odometria scaduta da %.2fs - riferimento congelato",
          (this->now() - last_odom_time_).seconds());
      return;
    }
    if (holdActive()) {
      // Override: la fermata ha priorita' su qualunque cosa pubblichi il
      // pianificatore. Ma il punto di arresto passa dalla STESSA rampa del ramo
      // normale, invece di essere pubblicato di colpo.
      //
      // Prima questo ramo scavalcava la rampa, e li' nasceva lo scalino: la
      // rampa poteva aver portato il riferimento fino a un metro davanti al
      // velivolo, e all'ingresso in fermata il riferimento tornava di colpo
      // sulla posizione corrente - un gradino all'indietro di un metro, seguito
      // dal gradino opposto all'uscita. Col tremolio della misura di distanza i
      // due gradini si alternavano piu' volte al secondo. Sul registro di volo
      // (BRAKE run 2) il riferimento si spostava di 1.35 m in 0.4 s, cioe' 3.4
      // m/s, mentre la rampa e' tarata a 1.0 m/s: la prova che quel percorso non
      // passava di li'. Il velivolo toccava 4.42 m/s contro i 0.6 del
      // pianificatore, e l'inclinazione saliva 54 -> 73 -> 91 -> 96 gradi fino al
      // ribaltamento. Nelle run pulite la velocita' non ha mai superato 1.35 m/s.
      const Eigen::Vector3d cmd_world = rampAndClamp(hold_pos_world_);
      const Eigen::Vector3d hold_ned =
          px4_ros_com::frame_transforms::enu_to_ned_local_frame(worldToLocal(cmd_world));
      msg.position = { static_cast<float>(hold_ned.x()), static_cast<float>(hold_ned.y()),
        static_cast<float>(hold_ned.z()) };
      msg.velocity = { 0.0f, 0.0f, 0.0f };
      msg.acceleration = { 0.0f, 0.0f, 0.0f };
      msg.yaw = last_yaw_ned_;
    } else if (rientro_preciso_) {
      // Stessa rampa (e filtro di sicurezza) del volo normale, ma lenta.
      slew_limite_ = home_approach_speed_;
      const Eigen::Vector3d cmd_world = rampAndClamp(home_target_world_);
      slew_limite_ = 0.0;
      const Eigen::Vector3d ned =
          px4_ros_com::frame_transforms::enu_to_ned_local_frame(worldToLocal(cmd_world));
      msg.position = { static_cast<float>(ned.x()), static_cast<float>(ned.y()),
        static_cast<float>(ned.z()) };
      msg.velocity = { 0.0f, 0.0f, 0.0f };
      msg.acceleration = { 0.0f, 0.0f, 0.0f };
      msg.yaw = last_yaw_ned_;
    } else if (takingOff()) {
      const Eigen::Vector3d takeoff_ned =
          px4_ros_com::frame_transforms::enu_to_ned_local_frame(worldToLocal(takeoff_target_world_));
      msg.position = { static_cast<float>(takeoff_ned.x()), static_cast<float>(takeoff_ned.y()),
        static_cast<float>(takeoff_ned.z()) };
      msg.velocity = { 0.0f, 0.0f, 0.0f };
      msg.acceleration = { 0.0f, 0.0f, 0.0f };
      aggiornaGiro(msg);
      msg.yaw = last_yaw_ned_;
    } else {
      // Bound how far ahead of the vehicle a setpoint may sit.
      //
      // Holding position freezes the vehicle but NOT the trajectory clock:
      // traj_server keeps evaluating the B-spline against wall time, so after a
      // 2-3s hold the planner's setpoint is 2-3m further along the path than
      // the drone actually is. Releasing the hold then handed PX4 a multi-metre
      // position step, which its position controller answers with maximum tilt
      // - drone 1 went inverted (roll 132 deg) three seconds after a
      // "Peer clear - resuming planner". Clamping the step along the direction
      // to the target keeps the commanded error inside what the vehicle can
      // track, and the planner's next replan (fsm.thresh_replan3) re-anchors
      // the trajectory to the current position anyway.
      Eigen::Vector3d cmd_ned = last_pos_ned_;
      Eigen::Vector3d cmd_world = last_pos_world_;
      if (have_odom_) {
        // Il clamp da solo non bastava. Limitare l'errore a max_setpoint_jump_
        // lascia comunque uno SCALINO: il riferimento salta di un metro in un
        // singolo ciclo di controllo, e il transitorio che ne segue ribalta il
        // velivolo. Misurato su 4 run di cubicle_office: l'ultimo clamp cade
        // 0.42-1.62 s prima del crash, e siccome il rilevatore richiede 0.5 s di
        // inclinazione sostenuta l'inizio del ribaltamento coincide col clamp.
        // Nessuna uscita da offboard e nessun failsafe nei log PX4, e i punti di
        // impatto distano 0.59-1.23 m dall'ostacolo piu' vicino: non e' un urto
        // contro la geometria, e' il comando stesso.
        //
        // Quindi il riferimento viene fatto avanzare a velocita' limitata verso
        // il setpoint del pianificatore: una rampa continua invece di un
        // gradino. A regime la rampa non e' vincolante (setpoint_slew_speed e'
        // ben sopra la velocita' di crociera), interviene solo sui salti - cioe'
        // al rilascio di una fermata, che e' esattamente il caso documentato
        // sopra.
        cmd_world = rampAndClamp(last_pos_world_);
        cmd_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(worldToLocal(cmd_world));
      }
      msg.position = { static_cast<float>(cmd_ned.x()), static_cast<float>(cmd_ned.y()),
        static_cast<float>(cmd_ned.z()) };
      // NOTA: PX4 usa velocita' e accelerazione del trajectory_setpoint come
      // ANTICIPO ogni volta che sono finite, qualunque cosa dica
      // OffboardControlMode (PositionControl::_positionControl le somma con
      // addIfNotNanVector3f). Quindi questi due campi vanno trattati con la
      // stessa cura della posizione: se il filtro di sicurezza ha bloccato
      // l'avvicinamento del riferimento a un ostacolo, si toglie anche
      // dall'anticipo la componente verso l'ostacolo, altrimenti PX4 ce lo
      // spingerebbe lo stesso.
      Eigen::Vector3d vel_ned = last_vel_ned_;
      Eigen::Vector3d acc_ned = last_acc_ned_;
      if (have_odom_) {
        Eigen::Vector3d vw = last_vel_world_;
        Eigen::Vector3d aw = last_acc_world_;
        if (enable_safety_filter_) filtraAnticipo(vw, aw);
        anticipoAlPavimento(cmd_world, vw, aw);
        vel_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(spawn_rot_inv_ * vw);
        acc_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(spawn_rot_inv_ * aw);
      }
      msg.velocity = { static_cast<float>(vel_ned.x()), static_cast<float>(vel_ned.y()),
        static_cast<float>(vel_ned.z()) };
      msg.acceleration = { static_cast<float>(acc_ned.x()), static_cast<float>(acc_ned.y()),
        static_cast<float>(acc_ned.z()) };
      msg.yaw = last_yaw_ned_;
    }
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(msg);
  }

  // Imbardata del giro iniziale, chiamata dal ramo di decollo a ogni ciclo.
  // Prima della quota non fa nulla; durante il giro fa avanzare l'imbardata
  // comandata a velocita' costante (con l'anticipo yawspeed); alla fine torna
  // all'imbardata di partenza e annuncia volo_pronto una volta sola.
  void aggiornaGiro(px4_msgs::msg::TrajectorySetpoint& msg) {
    if (!quota_raggiunta_ || giro_concluso_) return;
    const double angolo = initial_spin_rate_ > 0.0
        ? (this->now() - giro_inizio_).seconds() * initial_spin_rate_ : 2.0 * M_PI;
    if (angolo >= 2.0 * M_PI) {
      giro_concluso_ = true;
      last_yaw_ned_ = giro_yaw0_;
      msg.yawspeed = 0.0f;
      std_msgs::msg::Bool pronto;
      pronto.data = true;
      volo_pronto_pub_->publish(pronto);
      RCLCPP_INFO(this->get_logger(), "[Giro] %s: volo pronto per l'esplorazione",
          initial_spin_rate_ > 0.0 ? "giro iniziale concluso" : "quota raggiunta");
      return;
    }
    last_yaw_ned_ = static_cast<float>(std::remainder(giro_yaw0_ + angolo, 2.0 * M_PI));
    msg.yawspeed = static_cast<float>(initial_spin_rate_);
  }

  // Arrivo sopra la partenza: entro home_land_tol in orizzontale, quasi fermo,
  // per un secondo di fila -> atterraggio verticale di PX4. Con un tetto di tempo:
  // se il filtro di sicurezza o un ostacolo impediscono di arrivare, si atterra
  // comunque dove si e', come prima.
  void controllaArrivoACasa() {
    if (!rientro_preciso_ || landing_) return;
    const double err = std::hypot(cur_pos_world_.x() - home_target_world_.x(),
                                  cur_pos_world_.y() - home_target_world_.y());
    const double vel = cur_vel_world_.norm();
    const rclcpp::Time now = this->now();
    const bool scaduto = (now - rientro_inizio_).seconds() > home_approach_timeout_;
    if (err < home_land_tol_ && vel < 0.15) {
      if (rientro_fermo_dal_.nanoseconds() == 0) rientro_fermo_dal_ = now;
    } else {
      rientro_fermo_dal_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    const bool arrivato =
        rientro_fermo_dal_.nanoseconds() > 0 && (now - rientro_fermo_dal_).seconds() > 1.0;
    if (!arrivato && !scaduto) return;
    RCLCPP_WARN(this->get_logger(), "[Casa] %s (scarto %.2f m): atterraggio verticale",
        arrivato ? "sopra il punto di decollo" : "tempo scaduto, atterro qui", err);
    rientro_preciso_ = false;
    landing_ = true;
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

  Eigen::Vector3d worldToLocal(const Eigen::Vector3d& p_world) const {
    return spawn_rot_inv_ * (p_world - spawn_trans_);
  }

  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = target_system_;
    msg.target_component = 1;
    msg.source_system = target_system_;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<c2x_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_cmd_ = false;
  bool have_setpoint_ = false;
  bool have_odom_ = false;
  rclcpp::Time last_odom_time_{ 0, 0, RCL_ROS_TIME };
  double odom_timeout_ = 0.5;
  double min_flight_z_ = 1.0;
  double min_flight_z_hard_ = 0.8;
  double max_flight_z_ = 0.0;
  double proximity_band_z_ = 0.45;
  // Contatori dei messaggi [Quota] (vedi anticipoAlPavimento e rampAndClamp).
  unsigned long quota_anticipi_tolti_ = 0, quota_piano_sotto_ = 0, quota_trattenute_ = 0;
  double quota_piano_min_ = std::numeric_limits<double>::infinity();

  // have_odom_ era un interruttore a senso unico: una volta acceso non si
  // spegneva piu'. Ma l'odometria SI INTERROMPE - nei log delle run compaiono
  // 7-478 "no odom" per drone - e durante l'interruzione cur_pos_world_ resta
  // congelato sull'ultimo valore noto. Rampa e clamp si ancorano quindi a una
  // posizione vecchia mentre il velivolo continua a muoversi: il comando che ne
  // esce non corrisponde piu' alla realta'. E' l'unica via per cui un errore
  // limitato a 1 m possa produrre i 4.42 m/s misurati sul registro di volo.
  bool odomFresh() const {
    if (!have_odom_) return false;
    if (odom_timeout_ <= 0.0) return true;
    return (this->now() - last_odom_time_).seconds() < odom_timeout_;
  }
  bool armed_ = false;
  bool ha_volato_ = false;
  bool volo_terminato_ = false;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr flight_terminated_pub_;
  bool landing_ = false;
  // PX4 uscito da solo dall'offboard dopo l'armamento (controllaModalitaPx4):
  // il ponte non pubblica piu' nulla verso PX4 e non riarma. Separato da
  // landing_ perche' land_request deve poter ancora chiedere NAV_LAND.
  uint8_t nav_state_ = 0;
  bool visto_offboard_armato_ = false;
  bool ceduto_a_px4_ = false;
  rclcpp::Time ceduto_dal_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_sub_;
  uint8_t target_system_ = 1;
  uint64_t setpoint_counter_ = 0;
  uint8_t last_status_ = c2x_msgs::msg::PositionCommand::TRAJECTORY_STATUS_EMPTY;
  Eigen::Vector3d last_pos_ned_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_pos_world_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_vel_ned_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_acc_ned_ = Eigen::Vector3d::Zero();
  float last_yaw_ned_ = 0.0f;

  // Reactive proximity-stop state (see onCloud()).
  Eigen::Vector3d cur_pos_world_ = Eigen::Vector3d::Zero();
  // Punto di arresto che il velivolo puo' davvero raggiungere.
  //
  // Congelare il riferimento nella posizione corrente (era hold_pos_world_ =
  // cur_pos_world_) ordina di fermarsi in un punto che il drone, a velocita' di
  // crociera, ha gia' superato. Sul registro di volo della run UAF 3 il
  // riferimento e' rimasto fermo a (2.88, 0.65) mentre il drone proseguiva fino
  // a (7.34, 3.03): 5.06 m di errore, seguiti dall'ordine di invertire la marcia
  // a piena velocita'. Il rollio ha iniziato a oscillare (-22, +22, -49, -62,
  // -72, +40, -51 gradi) fino al ribaltamento a -180.
  //
  // Quindi il punto viene proiettato in avanti della distanza di frenata, mai
  // oltre il margine disponibile verso l'ostacolo: si chiede una decelerazione,
  // non un'inversione.
  // PROVATO E SCARTATO (campagna BRAKE, 4 run): proiettare il punto in avanti
  // della distanza di frenata NON ha ridotto i ribaltamenti (2 su 4, contro 1
  // su 3 senza). Il motivo e' che a 2.5 m/s la proiezione vale oltre un metro
  // NELLA DIREZIONE DI MARCIA, cioe' verso l'ostacolo: invece di far decelerare
  // il drone gli si sposta il bersaglio piu' avanti. Si torna a fermare nella
  // posizione corrente; il vero segnale e' altrove (vedi max_setpoint_jump_).
  Eigen::Vector3d brakingHoldPoint(double /*clearance*/) const {
    return cur_pos_world_;
  }

  // Rampa + clamp, usata da OGNI ramo che comanda una posizione: e' quello che
  // garantisce la continuita' del riferimento, quindi nessun ramo puo'
  // scavalcarla senza reintrodurre gli scalini.
  Eigen::Vector3d rampAndClamp(const Eigen::Vector3d& target_in) {
    // La quota minima si impone sul BERSAGLIO, non sull'uscita: applicarla dopo
    // la rampa lascerebbe ramp_pos_world_ a inseguire un punto piu' basso, e il
    // riferimento resterebbe incollato al limite invece di risalire.
    Eigen::Vector3d target_world = target_in;
    // Lo scarto di quota dell'incrocio si somma QUI, sul bersaglio, prima della
    // quota minima e prima della rampa: cosi' passa dalla stessa rampa di tutto
    // il resto e non produce il gradino nel riferimento che ha gia' causato dei
    // ribaltamenti (vedi la nota sul ramo di fermata). Chi scende viene fermato
    // dalla quota minima subito sotto, ed e' voluto: e' chi sale a creare la
    // separazione.
    // Il tetto vale SOLO durante la manovra: e' il limite di quanto si puo'
    // salire per scansare un compagno, non un limite di volo. Lasciarlo fuori
    // dalla guardia lo applicava a ogni comando anche a manovra spenta - un
    // cambiamento di comportamento non voluto, entrato di soppiatto insieme
    // alla funzione.
    if (crossing_) {
      // Quota assegnata dalla manovra, IMPOSTA e non sommata: e' l'unico punto
      // del programma dove la quota minima di volo viene deliberatamente
      // scavalcata, perche' la corsia bassa a 0.50 m sta sotto di essa ed e'
      // proprio quella a rendere possibile l'incrocio. Fuori dalla manovra il
      // minimo torna a valere subito (ramo else).
      target_world.z() = crossing_z_;
    } else if (min_flight_z_ > 0.0) {
      target_world.z() = std::max(target_world.z(), min_flight_z_);
    }
    if (max_flight_z_ > 0.0) target_world.z() = std::min(target_world.z(), max_flight_z_);
    const rclcpp::Time now = this->now();
    double dt = 0.0;
    if (last_ramp_time_.nanoseconds() > 0) dt = (now - last_ramp_time_).seconds();
    last_ramp_time_ = now;
    if (!ramp_valid_ || dt <= 0.0 || dt > 0.5) {
      // Primo comando, oppure un buco temporale: riparti dal veicolo invece di
      // rampare da uno stato vecchio.
      ramp_pos_world_ = cur_pos_world_;
      ramp_valid_ = true;
    }
    const Eigen::Vector3d to_target = target_world - ramp_pos_world_;
    double vel_rampa =
        slew_limite_ > 0.0 ? std::min(setpoint_slew_speed_, slew_limite_) : setpoint_slew_speed_;
    // Vicino agli ostacoli il riferimento avanza al massimo a v_vicino (vedi
    // enable_proximity_speed_); l'anticipo lo limita filtraAnticipo.
    v_vicino_ = velocitaVicinoOstacoli();
    vel_rampa = std::min(vel_rampa, v_vicino_);
    const double max_step = vel_rampa * dt;
    Eigen::Vector3d passo = to_target;
    if (max_step > 0.0 && to_target.norm() > max_step) passo = to_target * (max_step / to_target.norm());
    // Filtro di sicurezza sul MOVIMENTO del riferimento (vedi
    // enable_safety_filter_): toglie solo la componente che lo avvicinerebbe
    // all'ostacolo oltre il consentito. Applicato al passo e non alla
    // posizione, quindi il riferimento resta continuo per costruzione.
    const double z_prima = ramp_pos_world_.z();
    passo = filtraPasso(ramp_pos_world_, passo, dt);
    ramp_pos_world_ += passo;
    governaFrenata();
    // Filtro e frenata possono abbassare la rampa (ostacolo sovrastante, drone in
    // salita): mai sotto min_flight_z_hard (vedi la dichiarazione), ne' sotto dov'era
    // se ci si trovava gia' (dopo il decollo, o rientrando da un cedimento).
    if (!crossing_ && min_flight_z_hard_ > 0.0 && ramp_pos_world_.z() < z_prima &&
        ramp_pos_world_.z() < min_flight_z_hard_) {
      ramp_pos_world_.z() = std::min(z_prima, min_flight_z_hard_);
      ++quota_trattenute_;
    }
    if (filtro_attivo_) {
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 2000,
          "[Filtro] ostacolo a %.2fm: avvicinamento del riferimento limitato (%lu interventi)",
          filtro_dist_, filtro_interventi_);
    }

    // Il clamp resta come seconda barriera: la rampa garantisce continuita',
    // questo garantisce che il riferimento non si allontani mai troppo dal
    // veicolo (se il drone non riesce a seguire, la rampa da sola gli
    // scapperebbe comunque avanti).
    Eigen::Vector3d cmd_world = ramp_pos_world_;
    if (max_setpoint_jump_ > 0.0) {
      const Eigen::Vector3d err = cmd_world - cur_pos_world_;
      const double dist = err.norm();
      if (dist > max_setpoint_jump_) {
        const double z_rampa = cmd_world.z();
        cmd_world = cur_pos_world_ + err * (max_setpoint_jump_ / dist);
        // Il limite avvicina il riferimento al DRONE, anche in verticale: con un
        // drone che cede di quota lo trascinava giu' con se'. Run
        // 2026-10-18_1248 (parco): scarto 0.65-0.68 m per 2 s, riferimento da 0.93
        // a 0.61 m (quota PX4) mentre il drone scendeva da 0.86 a 0.13, sempre
        // 0.47-0.51 m sopra di lui. Il gradino che il limite evita e' orizzontale
        // (inclinazione); uno scarto in verticale verso l'ALTO PX4 lo recupera di
        // spinta, a MPC_Z_VEL_MAX_UP. Quindi sotto il pavimento il limite non
        // abbassa il riferimento piu' di dove la rampa lo aveva gia' portato.
        if (!crossing_ && min_flight_z_ > 0.0) {
          const double z_min = std::min(z_rampa, min_flight_z_);
          if (cmd_world.z() < z_min) {
            cmd_world.z() = z_min;
            ++quota_trattenute_;
          }
        }
        ramp_pos_world_ = cmd_world;
        rclcpp::Clock clock(RCL_ROS_TIME);
        RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 2000,
            "Setpoint %.2fm ahead of the vehicle - clamped to %.2fm", dist, max_setpoint_jump_);
      }
    }

    // LA VERTICALE DELL'INCROCIO HA UNA RAMPA PROPRIA.
    //
    // Imporre la quota di corsia al BERSAGLIO non funzionava, ed e' misurato: su
    // 95 manovre innescate, nei comandi effettivi non compare MAI ne' 0.50 ne'
    // 1.50 durante la crociera. Il motivo e' che la rampa qui sopra avanza di
    // setpoint_slew_speed lungo la direzione 3D complessiva: con un errore
    // orizzontale di qualche metro, la componente verticale del passo e'
    // minuscola, e il limite di scostamento accorcia poi l'intero vettore
    // schiacciandola ancora. Il comando di quota veniva diluito dall'inseguimento
    // orizzontale e non arrivava mai a destinazione.
    // Scorporandola, la verticale si muove alla propria velocita' a prescindere
    // da quanto e' lontano il bersaglio orizzontale - che e' il comportamento
    // giusto, perche' scansarsi in quota e proseguire lungo il corridoio sono
    // due cose indipendenti.
    if (crossing_) {
      if (!cross_z_valid_) {
        cross_z_ramp_ = cur_pos_world_.z();
        cross_z_valid_ = true;
      }
      const double passo = cross_z_speed_ * dt;
      const double diff = crossing_z_ - cross_z_ramp_;
      cross_z_ramp_ += std::clamp(diff, -passo, passo);
      cmd_world.z() = cross_z_ramp_;
      ramp_pos_world_.z() = cross_z_ramp_;  // la rampa 3D non deve combatterla
    } else {
      cross_z_valid_ = false;
    }
    return cmd_world;
  }

  Eigen::Vector3d hold_pos_world_ = Eigen::Vector3d::Zero();
  double brake_accel_ = 2.0;
  double min_hold_time_ = 0.6;
  rclcpp::Time peer_hold_start_{ 0, 0, RCL_ROS_TIME };
  double spawn_yaw_ = 0.0;
  Eigen::Vector3d spawn_trans_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond spawn_rot_inv_ = Eigen::Quaterniond::Identity();

  double self_hit_radius_ = 0.25;
  double max_hold_time_ = 5.0;
  double max_setpoint_jump_ = 1.0;
  bool enable_proximity_speed_ = false;
  double prox_near_ = 0.6, prox_far_ = 0.9, prox_v_min_ = 0.5, prox_v_far_ = 1.2;
  double v_vicino_ = std::numeric_limits<double>::infinity();
  bool enable_brake_governor_ = false;
  double brake_gov_accel_ = 1.5, brake_gov_latency_ = 0.3, brake_gov_tube_ = 0.45,
         brake_gov_range_ = 2.5, brake_gov_acc_gain_ = 0.0, brake_gov_acc_max_ = 2.5;
  bool gov_attivo_ = false;
  Eigen::Vector3d gov_dir_ = Eigen::Vector3d::UnitX();
  double gov_vamm_ = 0.0, gov_dist_ = 0.0, gov_vn_ = 0.0;
  unsigned long gov_interventi_ = 0;
  bool enable_lidar_hold_ = false;
  // Filtro di sicurezza continuo (vedi enable_safety_filter nel costruttore).
  bool enable_safety_filter_ = false;
  double safety_dist_ = 0.55;
  double safety_slow_dist_ = 1.0;
  double safety_approach_speed_ = 0.6;
  std::vector<Eigen::Vector3d> ostacoli_;
  bool ostacoli_validi_ = false;
  bool filtro_attivo_ = false;
  Eigen::Vector3d filtro_normale_ = Eigen::Vector3d::Zero();
  double filtro_dist_ = 0.0;
  uint64_t filtro_interventi_ = 0;
  Eigen::Vector3d last_vel_world_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_acc_world_ = Eigen::Vector3d::Zero();
  bool enable_peer_hold_ = false;
  bool use_velocity_feedforward_ = false;
  // Riferimento rampato verso il setpoint del pianificatore (vedi il commento
  // nel publisher): evita che il riferimento si muova a gradini.
  Eigen::Vector3d ramp_pos_world_ = Eigen::Vector3d::Zero();
  rclcpp::Time last_ramp_time_{ 0, 0, RCL_ROS_TIME };
  bool ramp_valid_ = false;
  double setpoint_slew_speed_ = 1.0;
  Eigen::Vector3d nearest_obstacle_pt_ = Eigen::Vector3d::Zero();
  bool nearest_obstacle_valid_ = false;
  int retreat_count_ = 0;
  int max_retreats_ = 3;
  double obstacle_ttc_ = 1.2;
  double obstacle_stop_dist_ = 0.80;
  double obstacle_resume_dist_ = 1.05;
  rclcpp::Time hold_start_;
  // Pre-exploration takeoff (see takeoff_height_ in the constructor).
  double takeoff_height_ = 1.0;
  double takeoff_done_tol_ = 0.15;
  Eigen::Vector3d takeoff_target_world_ = Eigen::Vector3d::Zero();
  // Il decollo si conclude una volta sola: vedi la nota in takingOff().
  bool takeoff_done_ = false;
  // Giro iniziale (vedi initial_spin_rate nel costruttore).
  double initial_spin_rate_ = 0.5;
  bool quota_raggiunta_ = false;
  bool giro_concluso_ = false;
  rclcpp::Time giro_inizio_;
  float giro_yaw0_ = 0.0f;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr volo_pronto_pub_;
  // Atterraggio di precisione sulla partenza (vedi home_sub_ nel costruttore).
  double home_land_tol_ = 0.10;
  double home_approach_speed_ = 0.4;
  double home_approach_timeout_ = 30.0;
  bool rientro_preciso_ = false;
  rclcpp::Time rientro_inizio_;
  rclcpp::Time rientro_fermo_dal_;
  Eigen::Vector3d home_target_world_ = Eigen::Vector3d::Zero();
  double slew_limite_ = 0.0;  // >0: tetto temporaneo alla velocita' della rampa
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr home_sub_;
  double nearest_obstacle_dist_ = std::numeric_limits<double>::infinity();
  bool proximity_hold_ = false;

  // Inter-drone separation (see updatePeerSeparation()).
  struct PeerState {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    rclcpp::Time recv;
  };
  std::map<int, PeerState> peers_;
  Eigen::Vector3d cur_vel_world_ = Eigen::Vector3d::Zero();
  int drone_id_ = 1;
  double peer_stop_dist_ = 1.5;
  double peer_resume_dist_ = 2.0;
  double peer_hard_stop_dist_ = 1.0;
  double peer_lookahead_ = 1.0;
  double peer_state_timeout_ = 1.0;
  bool peer_hold_ = false;
  // Stato della manovra di incrocio in quota (vedi updateCrossing).
  bool enable_crossing_ = true;
  bool crossing_ = false;
  int crossing_peer_ = -1;
  double crossing_z_ = 0.0;
  rclcpp::Time crossing_start_{ 0, 0, RCL_ROS_TIME };
  double cross_trigger_dist_ = 3.0;
  double cross_low_z_ = 0.50;
  double cross_high_z_ = 1.50;
  double cross_z_band_ = 0.6;
  double cross_min_speed_ = 0.15;
  double cross_head_on_cos_ = 0.3;
  double cross_timeout_ = 8.0;
  double cross_stall_time_ = 2.0;
  double cross_z_speed_ = 0.8;
  double cross_z_ramp_ = 0.0;
  bool cross_z_valid_ = false;
  int stallo_peer_ = -1;
  rclcpp::Time stallo_da_{ 0, 0, RCL_ROS_TIME };
  double cross_clear_dz_ = 0.45;
  rclcpp::Subscription<c2x_msgs::msg::DroneState>::SharedPtr peer_state_sub_;

  // Either safety layer freezes the vehicle the same way.
  bool holdActive() const {
    return proximity_hold_ || peer_hold_;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCommandBridge>());
  rclcpp::shutdown();
  return 0;
}
