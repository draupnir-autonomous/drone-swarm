#include <plan_manage/planner_manager.h>

#include <traj_utils/planning_visualization.h>

#include <exploration_manager/c2_exploration_manager.h>
#include <exploration_manager/c2_exploration_fsm.h>
#include <exploration_manager/expl_data.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>

#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>

#include <plan_env/communication_graph.h>
#include <active_perception/uniform_grid.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using Eigen::Vector4d;

namespace c2_expl {
C2ExplorationFSM::C2ExplorationFSM() {
}

C2ExplorationFSM::~C2ExplorationFSM() {
}

void C2ExplorationFSM::init(rclcpp::Node::SharedPtr node) {
  node_ = node;

  // Parameters
  fp_.reset(new FSMParam);
  fp_->replan_thresh1_ = node_->declare_parameter("fsm.thresh_replan1", -1.0);
  fp_->replan_thresh2_ = node_->declare_parameter("fsm.thresh_replan2", -1.0);
  fp_->replan_thresh3_ = node_->declare_parameter("fsm.thresh_replan3", -1.0);
  fp_->replan_time_ = node_->declare_parameter("fsm.replan_time", -1.0);
  // Anticipo CORTO per le ripianificazioni anti-collisione: con l'anticipo
  // normale (fsm.replan_time, ora fino a 1 s su cubicle_office) il drone
  // continuerebbe per tutto quel tempo sulla traiettoria che porta all'ostacolo.
  replan_time_collisione_ = node_->declare_parameter("fsm.replan_time_collision", 0.2);
  fp_->attempt_interval_ = node_->declare_parameter("fsm.attempt_interval", 0.2);
  fp_->meeting_opt_interval_ = node_->declare_parameter("fsm.meeting_opt_interval", 1.0);
  fp_->state_fresh_timeout_ = node_->declare_parameter("fsm.state_fresh_timeout", 1.0);
  fp_->opt_wait_timeout_ = node_->declare_parameter("fsm.opt_wait_timeout", 2.0);
  fp_->opt_retry_interval_ = node_->declare_parameter("fsm.opt_retry_interval", 0.3);
  fp_->opt_retry_max_ = node_->declare_parameter("fsm.opt_retry_max", 4);
  fp_->opt_reject_backoff_ = node_->declare_parameter("fsm.opt_reject_backoff", 0.6);
  fp_->opt_reject_backoff_max_ = node_->declare_parameter("fsm.opt_reject_backoff_max", 2.5);
  fp_->repeat_send_num_ = node_->declare_parameter("fsm.repeat_send_num", 10);
  fp_->debug_opt_output_ = node_->declare_parameter("fsm.debug_opt_output", false);
  fp_->no_progress_timeout_ = node_->declare_parameter("fsm.no_progress_timeout", 45.0);
  fp_->no_progress_min_volume_ = node_->declare_parameter("fsm.no_progress_min_volume", 0.5);
  fp_->replan_reanchor_dist_ = node_->declare_parameter("fsm.replan_reanchor_dist", 0.5);
  fp_->plan_fail_backoff_ = node_->declare_parameter("fsm.plan_fail_backoff", 0.5);
  max_explore_time_ = node_->declare_parameter("fsm.max_explore_time", 0.0);
  fp_->plan_fail_ban_streak_ = node_->declare_parameter("fsm.plan_fail_ban_streak", 30);

  fp_->color_r_ = node_->declare_parameter("color_r", 1.0);
  fp_->color_g_ = node_->declare_parameter("color_g", 0.0);
  fp_->color_b_ = node_->declare_parameter("color_b", 0.0);
  fp_->color_a_ = node_->declare_parameter("color_a", 1.0);

  // Planning modules
  expl_manager_.reset(new C2ExplorationManager);
  expl_manager_->initialize(node_);
  visualization_.reset(new PlanningVisualization(node_));

  comm_graph_ = expl_manager_->comm_graph_;

  if (fp_->opt_reject_backoff_ < 0.0) fp_->opt_reject_backoff_ = 0.0;
  if (fp_->opt_reject_backoff_max_ < 0.0) fp_->opt_reject_backoff_max_ = 0.0;

  planner_manager_ = expl_manager_->planner_manager_;

  // FSM state
  fd_.reset(new FSMData);
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH",
    "IDL"
    "E" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  fd_->avoid_collision_ = false;
  fd_->go_back_ = false;
  fd_->expl_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // ROS interfaces
  exec_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.01), std::bind(&C2ExplorationFSM::FSMCallback, this));
  frontier_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.5), std::bind(&C2ExplorationFSM::frontierCallback, this));

  // Trigger, odom, the crash check, and safetyCallback (the preventive collision-replan
  // check) all go in their own reentrant group, off the default group that
  // exec_timer_/frontier_timer_ share. On a cluttered real scene, frontierCallback's
  // frontier/grid-tour computation - and FSMCallback's own PLAN_TRAJ replanning, which
  // runs the same grid-tour code - can take 10-20s and re-fire back to back, which
  // starves anything else stuck in that same default group. This bit a real crash twice:
  // first the tilt check couldn't report a crash that had already happened minutes
  // earlier because it lived in the default group; then, with that fixed, the drone
  // crashed again because checkTrajCollision - the check that's supposed to catch an
  // upcoming collision and replan *before* impact - was still in the default group and
  // couldn't run during the stall either, so the drone kept flying its existing
  // trajectory blind for however long the stall lasted. checkTrajCollision reads
  // planner_manager_->local_data_, which callExplorationPlanner (default group) mutates
  // in place; local_data_mutex_ (in FastPlannerManager) now serializes that access
  // instead of relying on callback-group exclusion. triggerCallback calls
  // updateFrontierStruct(), the same non-thread-safe call frontierCallback() makes, so
  // frontier_mutex_ (declared in the header) serializes those two the same way.
  // crashCheckCallback only reads fd_->odom_orient_/have_odom_ and calls transitState(),
  // which don't need either mutex.
  input_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  safety_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.05),
      std::bind(&C2ExplorationFSM::safetyCallback, this), input_cb_group_);
  crash_check_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.05),
      std::bind(&C2ExplorationFSM::crashCheckCallback, this), input_cb_group_);
  // 2s period: getExploredVolume() sweeps the whole occupancy buffer (2M voxels
  // at resolution 0.1 on a 20x20x5 map), which is a few milliseconds - fine
  // occasionally, not at the rate of the other timers. Same reentrant group, so
  // a busy planner can't stop the watchdog from noticing the planner is stuck.
  progress_timer_ = node_->create_wall_timer(std::chrono::duration<double>(2.0),
      std::bind(&C2ExplorationFSM::progressCheckCallback, this), input_cb_group_);

  // Copertura della telecamera, aggiornata a 5 Hz per tutto il volo.
  completamento_attivo_ = node_->declare_parameter("fsm.photo_completion", true);
  completamento_durata_ = node_->declare_parameter("fsm.photo_completion_time", 300.0);
  {
    CoperturaCamera::Parametri pc;
    pc.min_celle = node_->declare_parameter("fsm.photo_min_cells", 15);
    copertura_ = std::make_unique<CoperturaCamera>(expl_manager_->sdf_map_, pc);
  }
  visita_attiva_ = node_->declare_parameter("fsm.visit_completion", true);
  {
    CoperturaVisita::Parametri pv;
    pv.raggio = node_->declare_parameter("fsm.visit_radius", 2.0);
    pv.min_celle = static_cast<int>(node_->declare_parameter("fsm.visit_min_area", 1.0) /
                                    std::pow(expl_manager_->sdf_map_->getResolution(), 2));
    visita_ = std::make_unique<CoperturaVisita>(expl_manager_->sdf_map_, pv);
  }
  pari_visita_.assign(std::max<size_t>(1, expl_manager_->ep_->drone_num_), PariVisita());
  // IMBARDATA DA FOTO durante l'esplorazione (fsm.photo_yaw): la telecamera
  // si gira verso le superfici non ancora inquadrate invece di guardare
  // sempre avanti. In cubicle_office l'esplorazione finiva con il 76% della
  // superficie inquadrata e la fase fotografica, in 300 s, arrivava all'88%
  // con 65 zone ancora da fare (V28): conviene fotografare strada facendo.
  const bool imbardata_foto = node_->declare_parameter("fsm.photo_yaw", true);
  if (imbardata_foto) {
    planner_manager_->punteggio_imbardata_ = [this](const Eigen::Vector3d& p, double yaw) {
      std::lock_guard<std::mutex> lk(copertura_mutex_);
      return static_cast<double>(copertura_->contaNonViste(p, yaw));
    };
  }
  // La copertura della telecamera (80x60 raggi fino a 6 m, ~3 milioni di
  // letture della mappa al secondo) serve solo alla fase fotografica e
  // all'imbardata da foto, di serie spente: senza, sul Jetson era CPU tolta al
  // pianificatore e alla stessa callback group dell'odometria. La visita costa
  // poco (salta le celle gia' visitate) e resta sempre attiva.
  const bool serve_copertura = completamento_attivo_ || imbardata_foto;
  copertura_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.2),
      [this, serve_copertura]() {
    if (!fd_->have_odom_ || !fd_->trigger_ || finale_) return;
    // Compagni in volo con uno stato recente (vedi pari_visita_): lo spazio dove passano
    // loro e' visitato anche per questo drone. Il controllo della vista libera si fa
    // sulla mappa di questo drone, che riceve anche la loro (scambio dei pezzi di mappa).
    std::vector<Eigen::Vector3d> altri;
    {
      std::lock_guard<std::mutex> lp(pari_mutex_);
      const double ora = node_->now().seconds();
      for (const auto& p : pari_visita_)
        if (p.recv > 0.0 && ora - p.recv < 1.0 && p.fase < 2) altri.push_back(p.pos);
    }
    std::lock_guard<std::mutex> lk(copertura_mutex_);
    if (serve_copertura) copertura_->aggiorna(fd_->odom_pos_, fd_->odom_orient_);
    visita_->aggiorna(fd_->odom_pos_);
    for (const auto& q : altri) visita_->aggiorna(q);
  }, input_cb_group_);
  rclcpp::SubscriptionOptions input_sub_opts;
  input_sub_opts.callback_group = input_cb_group_;
  // The trigger stays an absolute topic on purpose: one publication on
  // /move_base_simple/goal starts every drone at once, whatever namespace each
  // exploration node runs in. Everything else below is relative, so it lands
  // under this node's /drone_N namespace.
  // transient_local + reliable (sciame, ottobre 2026): il via lo pubblica un processo nuovo
  // (ros2 topic pub nello script di run) e con tre droni sulla CPU satura la scoperta DDS fra
  // lui e i nodi poteva durare piu' della pubblicazione: nella seconda run su office_small
  // solo il drone 2 l'ha ricevuto, gli altri sono rimasti in attesa a mezz'aria. Con la
  // durabilita' "transient_local" anche dalla parte del pubblicatore, un nodo che si accoppia
  // in ritardo riceve comunque l'ultimo messaggio.
  trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&C2ExplorationFSM::triggerCallback, this, std::placeholders::_1), input_sub_opts);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("odom_world", 1,
      std::bind(&C2ExplorationFSM::odometryCallback, this, std::placeholders::_1), input_sub_opts);

  // Conferma di prontezza. transient_local perche' il coordinatore puo'
  // iscriversi dopo che il drone e' gia' pronto e deve comunque ricevere
  // l'ultimo stato invece di restare in attesa di un aggiornamento.
  ready_pub_ = node_->create_publisher<c2x_msgs::msg::ExplorationReady>(
      "exploration_ready", rclcpp::QoS(1).transient_local());
  ready_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&C2ExplorationFSM::publishReadiness, this),
      input_cb_group_);

  replan_pub_ = node_->create_publisher<std_msgs::msg::Empty>("planning/replan", 10);
  // Richiesta di atterraggio al ponte, a rientro concluso.
  land_pub_ = node_->create_publisher<std_msgs::msg::Empty>("land_request", 1);
  home_land_pub_ = node_->create_publisher<std_msgs::msg::Empty>("land_at_home", 1);
  // Il ponte annuncia qui un disarmo avvenuto in volo (atterraggio non
  // richiesto, caduta): la missione e' finita e non va ripianificata.
  flight_terminated_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "flight_terminated", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Empty::SharedPtr) {
        if (finale_) return;
        finale_ = true;
        returning_ = false;
        RCLCPP_ERROR(node_->get_logger(),
            "Drone %d: VOLO INTERROTTO - il velivolo e' stato disarmato da PX4 a missione "
            "iniziata, esplorazione chiusa", getId());
        state_ = FINISH;
      },
      input_sub_opts);
  attendi_volo_pronto_ = node_->declare_parameter("fsm.wait_flight_ready", false);
  if (attendi_volo_pronto_) {
    volo_pronto_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "volo_pronto", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr m) {
          if (m->data && !volo_pronto_) {
            volo_pronto_ = true;
            RCLCPP_INFO(node_->get_logger(), "Drone %d: decollo e giro iniziale conclusi", getId());
          }
        },
        input_sub_opts);
  }
  bspline_pub_ = node_->create_publisher<c2x_msgs::msg::Bspline>("planning/bspline", 10);

  // Swarm: state broadcast, MeetingOpt negotiation and trajectory exchange. The
  // *_send / *_recv topic pairs are bridged between the per-drone namespaces by
  // c2x_px4_bridge's swarm_relay node, which is where ROS1's swarm_bridge (UDP)
  // sat. With drone_num == 1 nothing is ever relayed, so these stay inert.
  //
  // These deliberately stay in the node's DEFAULT (mutually exclusive) callback
  // group, together with exec_timer_ and frontier_timer_. ROS1 ran the whole
  // FSM on one callback queue with a single spinner, so the negotiation
  // handlers could never overlap the planner; giving them their own group here
  // broke that assumption twice over. Both optTimerCallback (via allocateTasks)
  // and FSMCallback (via planExploreMotion -> findGlobalTourOfGrid) end up in
  // callSolveTSP, which blocks in rclcpp::spin_until_future_complete on the one
  // shared tsp_client_node_ - two threads spinning the same node throws, and it
  // did: drone 1 aborted mid-flight with an uncaught exception out of
  // librclcpp. They also both mutate HGrid's last-cost-matrix tables. Keeping
  // them mutually exclusive costs some responsiveness while a long plan runs -
  // which is what the protocol's retries and opt_wait_timeout are there for.
  //
  // Default-constructed options: no callback_group set, so the subscriptions
  // below land in that same default group.
  rclcpp::SubscriptionOptions swarm_sub_opts;

  drone_state_pub_ = node_->create_publisher<c2x_msgs::msg::DroneState>("swarm_expl/drone_state_send", 10);
  drone_state_sub_ = node_->create_subscription<c2x_msgs::msg::DroneState>(
      "swarm_expl/drone_state_recv", 10,
      std::bind(&C2ExplorationFSM::droneStateMsgCallback, this, std::placeholders::_1),
      swarm_sub_opts);
  drone_state_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.05),
      std::bind(&C2ExplorationFSM::droneStateTimerCallback, this));
  // Posizione, fase e meta di visita dei compagni (vedi pari_visita_sub_ nell'header): nel
  // gruppo degli ingressi, che gira anche mentre si pianifica. A tre droni su cubicle_office il
  // drone 3 chiudeva la visita della sua striscia mentre il drone 1, con buchi nella pianta
  // dove il 3 era passato, attraversava la mappa per visitarla di nuovo (ottobre 2026).
  pari_visita_sub_ = node_->create_subscription<c2x_msgs::msg::DroneState>(
      "swarm_expl/drone_state_recv", 50,
      [this](const c2x_msgs::msg::DroneState::SharedPtr msg) {
        if (msg->drone_id == getId() || msg->pos.size() < 3) return;
        std::lock_guard<std::mutex> lp(pari_mutex_);
        if (msg->drone_id < 1 || msg->drone_id > static_cast<int>(pari_visita_.size())) return;
        auto& p = pari_visita_[msg->drone_id - 1];
        p.pos = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
        p.recv = node_->now().seconds();
        p.fase = msg->fase;
        p.ha_meta = msg->ha_meta_visita;
        p.meta = Eigen::Vector3d(msg->meta_visita.x, msg->meta_visita.y, msg->meta_visita.z);
      },
      input_sub_opts);

  opt_pub_ = node_->create_publisher<c2x_msgs::msg::MeetingOpt>("swarm_expl/meeting_opt_send", 100);
  opt_sub_ = node_->create_subscription<c2x_msgs::msg::MeetingOpt>("swarm_expl/meeting_opt_recv",
      100, std::bind(&C2ExplorationFSM::optMsgCallback, this, std::placeholders::_1),
      swarm_sub_opts);
  opt_res_pub_ = node_->create_publisher<c2x_msgs::msg::MeetingOptResponse>(
      "swarm_expl/meeting_opt_res_send", 10);
  opt_res_sub_ = node_->create_subscription<c2x_msgs::msg::MeetingOptResponse>(
      "swarm_expl/meeting_opt_res_recv", 100,
      std::bind(&C2ExplorationFSM::optResMsgCallback, this, std::placeholders::_1),
      swarm_sub_opts);
  opt_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.5),
      std::bind(&C2ExplorationFSM::optTimerCallback, this));

  swarm_traj_pub_ = node_->create_publisher<c2x_msgs::msg::Bspline>("planning/swarm_traj_send", 100);
  swarm_traj_sub_ = node_->create_subscription<c2x_msgs::msg::Bspline>("planning/swarm_traj_recv",
      100, std::bind(&C2ExplorationFSM::swarmTrajCallback, this, std::placeholders::_1),
      swarm_sub_opts);
  swarm_traj_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.1),
      std::bind(&C2ExplorationFSM::swarmTrajTimerCallback, this));
}

int C2ExplorationFSM::getId() {
  return expl_manager_->ep_->drone_id_;
}

void C2ExplorationFSM::FSMCallback() {
  rclcpp::Clock clock(RCL_ROS_TIME);

  // TRACCIA DI STATO, una riga al secondo.
  //
  // Serve a rispondere a una domanda che finora ho solo ipotizzato: quando un
  // drone e' fermo, COSA sta facendo? Ho gia' escluso per misura la mancanza di
  // bersaglio, le fermate di sicurezza (<5% del tempo fermo), l'errore di
  // inseguimento (1 cm), la collisione fra droni e l'attesa dopo la conclusione.
  // Ogni ipotesi successiva e' costata una campagna, e sette su nove sono state
  // smentite: conviene misurare invece di dedurre.
  // Con stato, velocita', frontiere e celle sulla stessa riga e sullo stesso
  // orologio, un solo volo basta a dire dove va il tempo.
  {
    // t_sim: tempo dello stack (simulato con tempo_sim). L'intestazione della riga e'
    // l'orologio di parete del logger: le due insieme danno il RTF e permettono di
    // misurare le fasi della missione in secondi simulati (pagella_sciame.py).
    RCLCPP_INFO_THROTTLE(node_->get_logger(), clock, 1000,
        "[Traccia] stato=%s vel=%.2f frontiere=%d celle=%zu rientro=%d ricerca=%d fase=%d "
        "t_sim=%.2f",
        fd_->state_str_[static_cast<int>(state_)].c_str(), fd_->odom_vel_.norm(),
        frontiers_seen_, expl_manager_->ed_->swarm_state_[getId() - 1].grid_ids_.size(),
        returning_ ? 1 : 0, rdv_active_ ? 1 : 0, faseMissione(), node_->now().seconds());
  }

  // VIE D'USCITA DAL RITROVAMENTO - controllate a OGNI ciclo, in qualunque
  // stato. E' qui che il primo tentativo aveva sbagliato: le condizioni di
  // uscita stavano dentro il ramo IDLE, dove il drone non tornava piu' perche'
  // pianificare verso un punto riesce sempre. Risultato: 1305 righe di log e
  // zero cicli di esplorazione, il drone che inseguiva per sempre la posizione
  // di un compagno vista un minuto prima.
  //
  // Quattro uscite, e ne basta una. Meglio interrompere un ritrovamento di
  // troppo che restarci dentro: il costo di uscire presto e' qualche secondo,
  // il costo di non uscire e' la run intera.
  // Uscite dal RIENTRO, stesso principio del ritrovamento: controllate a ogni
  // ciclo e con un tetto di tempo, perche' un rientro che non termina sarebbe
  // la stessa trappola gia' vista.
  if (returning_) {
    // Distanza ORIZZONTALE: si e' sopra la partenza, alla discesa pensa
    // l'atterraggio. Pretendere anche la quota terrebbe il drone in volo a
    // inseguire un bersaglio che non deve raggiungere.
    const double dx = fd_->odom_pos_.x() - return_target_.x();
    const double dy = fd_->odom_pos_.y() - return_target_.y();
    const bool a_casa = std::hypot(dx, dy) < 1.5;
    const bool fuori_tempo = node_->now().seconds() > return_deadline_;
    if (a_casa || fuori_tempo) {
      returning_ = false;
      RCLCPP_WARN(node_->get_logger(), "[Rientro] Drone %d %s", getId(),
          a_casa ? "arrivato alla partenza: avvicinamento finale e atterraggio"
                 : "tempo scaduto, atterra dov'e'");
      // Atterraggio: si chiede in ogni caso, anche a tempo scaduto: lasciare un
      // drone in volo indefinitamente consuma batteria e lo tiene come ostacolo
      // per gli altri, che e' il problema da cui siamo partiti. Arrivati vicino
      // a casa l'ultimo tratto (fino alla verticale esatta del decollo) lo fa il
      // ponte: prima si atterrava a 1.2-1.5 m dalla partenza.
      if (a_casa)
        home_land_pub_->publish(std_msgs::msg::Empty());
      else
        land_pub_->publish(std_msgs::msg::Empty());
      transitState(FINISH, "rientro");
      finale_ = true;
    }
  }

  // TETTO AL TEMPO DI ESPLORAZIONE (fsm.max_explore_time).
  //
  // I criteri di conclusione esistenti - nessuna frontiera, guardiano dei
  // progressi, tentativi a vuoto in IDLE - coprono i casi visti, ma nessuno
  // GARANTISCE che la missione finisca: un drone che guadagna appena abbastanza
  // volume da non far scattare il guardiano puo' continuare all'infinito. Il
  // tetto e' quella garanzia: oltre, si rientra e si atterra con la mappa
  // raccolta fin li'. La riga riporta le frontiere rimaste, quindi un'uscita
  // per tetto resta distinguibile da un'esplorazione completa.
  if (max_explore_time_ > 0.0 && !returning_ && !finale_ && trigger_time_.nanoseconds() > 0 &&
      (state_ == PLAN_TRAJ || state_ == PUB_TRAJ || state_ == EXEC_TRAJ || state_ == IDLE) &&
      (node_->now() - trigger_time_).seconds() > max_explore_time_) {
    RCLCPP_WARN(node_->get_logger(),
        "Drone %d has finished exploration (frontiere note %d, tentativi a vuoto %d) - "
        "TEMPO MASSIMO (%.0fs dal via): rientro con la mappa raccolta [volume esplorato %.1f m3]",
        getId(), frontiers_seen_, fd_->idle_retries_, max_explore_time_,
        expl_manager_->sdf_map_->getExploredVolume());
    fd_->static_state_ = true;
    // Niente troncamento: vedi la nota "FERMATE DI COLPO" su "Idle since no frontier".
    concludiEsplorazione("tempo massimo");
    return;
  }

  // TETTO PER ZONA DI VISITA (ottobre 2026): 15 s + 1.5 s per metro di distanza al momento
  // della scelta. Una zona fra i cubicoli di cubicle_office, irraggiungibile per come la
  // vedeva la mappa, costava 55-63 s di "Replan: collision detected" in tre run di fila prima
  // di essere chiusa; scartandola si passa alla successiva (resta comunque vista dal lidar).
  if (completamento_ && zona_di_visita_ && !returning_ && !finale_ && scadenza_zona_ > 0.0 &&
      node_->now().seconds() > scadenza_zona_) {
    RCLCPP_WARN(node_->get_logger(),
        "[Visita] zona (%.1f %.1f) non raggiunta entro il suo tetto di tempo: scartata",
        zona_foto_.x(), zona_foto_.y());
    zone_scartate_.push_back(zona_foto_);
    scadenza_zona_ = 0.0;
    fd_->static_state_ = true;
    if (prossimaZona())
      transitState(PLAN_TRAJ, "visita");
    else
      chiudiCompletamento("zona non raggiunta, nessun'altra");
    return;
  }

  if (rdv_active_) {
    const double now_s = node_->now().seconds();
    const bool arrivato = (fd_->odom_pos_ - rdv_target_).norm() < 1.5;
    const bool scaduto = now_s > rdv_deadline_;
    const bool ha_compagni = expl_manager_->ed_->neighbor_ids_.size() > 1;
    if (arrivato || scaduto || ha_compagni) {
      rdv_active_ = false;
      RCLCPP_WARN(node_->get_logger(), "[Ritrovamento] Drone %d termina la ricerca (%s)", getId(),
          arrivato ? "arrivato" : (ha_compagni ? "compagno ritrovato" : "tempo scaduto"));
      if (state_ != FINISH) transitState(PLAN_TRAJ, "ritrovamento");
    }
  }
  // RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
  //     "\033[34m[FSM]\033[34m: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000, "no odom");
        return;
      }
      if ((node_->now() - fd_->fsm_init_time_).seconds() < 2.0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000, "wait for init");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000, "wait for trigger.");
      break;
    }

    case FINISH: {
      // RCLCPP_INFO_THROTTLE(node_->get_logger(), clock, 1000, "finish exploration.");
      break;
    }

    case IDLE: {
      // Rientro in corso: si riprende il volo verso casa invece di concludere.
      //
      // Senza questo il rientro si fermava dopo UN SOLO tratto: il drone
      // pianificava, volava, la traiettoria finiva, tornava in inattivita' - e
      // qui veniva dichiarato concluso, perche' questo ramo non sapeva del
      // rientro. Da cui anche il doppio conteggio: ogni drone stampava la
      // conclusione due volte, e il contatore arrivava a 6 su 3.
      // Le vie d'uscita restano quelle in cima al ciclo (arrivo, tempo
      // scaduto), quindi non si puo' restare qui per sempre.
      if (returning_) {
        transitState(PLAN_TRAJ, "rientro");
        break;
      }
      double check_interval = (node_->now() - fd_->last_check_frontier_time_).seconds();
      if (check_interval > 5.0) {
        // In IDLE ci si arriva anche da NO_GRID, che vuol dire "l'allocazione non
        // mi ha assegnato nessuna griglia adesso" - non "non c'e' piu' niente da
        // esplorare". Dichiarare finito un drone in quel caso ritira per sempre
        // un terzo della flotta: in una run di cubicle_office il drone 2 e' stato
        // dichiarato "has finished exploration" 40 s dopo il via senza aver mai
        // esplorato, mentre gli altri due facevano tutto il lavoro - e il conteggio
        // finiti=3/3 lo faceva sembrare un successo.
        // Si conclude solo con lo stesso criterio gia' usato in triggerCallback:
        // nessuna frontiera residua. Altrimenti il drone resta disponibile e
        // riprova, cosi' rientra appena l'allocazione gli assegna qualcosa.
        int num_frontiers;
        {
          std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
          num_frontiers = expl_manager_->updateFrontierStruct(fd_->odom_pos_);
    frontiers_seen_ = num_frontiers;
          frontiers_seen_ = num_frontiers;
        }
        fd_->last_check_frontier_time_ = node_->now();
        // Il numero di frontiere e' GLOBALE: il drone vede la mappa condivisa,
        // quindi resta positivo anche quando le frontiere superstiti sono tutte
        // di competenza degli altri. Usarlo da solo come condizione per non
        // concludere produce un ciclo IDLE <-> PLAN_TRAJ ogni 5 s che non finisce
        // mai - osservato per un'ora intera in una run da 4274 s, ed e' il motivo
        // per cui finiti restava 0/3.
        // Quindi si insiste per un numero limitato di tentativi (copre i buchi
        // transitori dell'allocazione, che era il difetto opposto: un drone
        // ritirato dopo 40 s senza aver mai esplorato) e poi si conclude: se
        // dopo un minuto l'allocazione non ha ancora niente per questo drone, il
        // suo lavoro e' finito e gli altri proseguono.
        // Periodo di grazia dopo il via: senza il riscaldamento fisso la mappa
        // puo' essere ancora vuota nei primi secondi, e concludere qui
        // significherebbe terminare la run prima di averla iniziata.
        const double since_trigger = (node_->now() - trigger_time_).seconds();
        if (num_frontiers == 0 && since_trigger < min_explore_time_) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
              "Drone %d: mappa ancora vuota %.1fs dopo il via, continuo a pianificare", getId(),
              since_trigger);
          transitState(PLAN_TRAJ, "FSM");
          break;
        }
        // SPARTIZIONE A STRISCE: niente celle proprie ne' libere da aiutare vuol dire che le
        // frontiere rimaste sono dei compagni, che ci stanno lavorando. Si conclude subito
        // (visita della propria striscia, poi rientro) invece di riprovare per un minuto.
        bool striscia_finita = false;
        if (num_frontiers > 0) {
          std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
          striscia_finita = expl_manager_->spartizioneAStrisce() &&
              expl_manager_->ed_->swarm_state_[getId() - 1].grid_ids_.empty();
        }
        if (num_frontiers == 0 || striscia_finita || ++fd_->idle_retries_ > 12) {
          // Due uscite molto diverse venivano riportate allo stesso modo, e la
          // metrica ne risultava bugiarda:
          //  - CONCLUSA: non restano frontiere, il lavoro e' finito davvero;
          //  - CODA: ne restano pochissime e le ha gia' qualcun altro. Con una
          //    sola frontiera e tre droni, due restano per forza fermi: e'
          //    serializzazione della coda, non un guasto. Misurato: nella run
          //    AFF 6 il drone 1 e' rimasto senza celle 12 volte mentre restava
          //    UNA frontiera;
          //  - RINUNCIA PREMATURA: ne restano molte e nessuno le sta prendendo.
          //    Questo si' e' un guasto, ed e' l'unico caso che deve allarmare.
          const char* motivo;
          if (num_frontiers == 0)
            motivo = "CONCLUSA (niente piu' da esplorare)";
          else if (striscia_finita)
            motivo = "STRISCIA FINITA (le frontiere rimaste sono dei compagni)";
          else if (num_frontiers <= static_cast<int>(expl_manager_->ep_->drone_num_))
            motivo = "CODA (restano meno frontiere che droni: nulla da assegnargli)";
          else
            motivo = "RINUNCIA PREMATURA (restano molte frontiere non assegnate)";
          RCLCPP_WARN(node_->get_logger(),
              "Drone %d has finished exploration (frontiere note %d, tentativi a vuoto %d) - %s "
              "[volume esplorato %.1f m3]",
              getId(), num_frontiers, fd_->idle_retries_, motivo,
              expl_manager_->sdf_map_->getExploredVolume());
          // RIENTRO ALLA PARTENZA prima di dichiarare concluso.
          //
          // Un drone che ha finito restava fermo dove si trovava, e se quel
          // punto e' una strettoia diventa un ostacolo per chi deve ancora
          // passare - osservato dal vivo. Rientrare libera la scena, e per un
          // impiego reale serve comunque: batteria, recupero, scarico dati.
          // Effetto collaterale utile con il collegamento ottico: convergendo
          // tutti verso lo stesso punto le linee di vista si ristabiliscono, ed
          // e' l'occasione naturale per riconciliare le mappe di chi ha
          // esplorato zone diverse senza mai vedersi.
          concludiEsplorazione("esplorazione conclusa");
        } else {
          // PROVATO E SCARTATO (campagna RDV, 4 run): mandare il drone senza
          // lavoro incontro a un compagno, riusando go_back_, ha PEGGIORATO
          // tutto - da 3 run complete su 4 a 0-1 su 3, tutte al tetto di tempo.
          //
          // Il difetto era nel percorso riusato, non nell'idea: con go_back_
          // attivo il pianificatore smette di cercare frontiere e vola solo
          // verso il punto assegnato, e l'azzeramento del flag stava DENTRO il
          // ramo IDLE - dove il drone non torna piu', perche' pianificare verso
          // un punto riesce sempre. Misurato: dopo il ritrovamento, 1305 righe
          // di log e ZERO cicli di esplorazione. Il drone inseguiva per sempre
          // la posizione dove un compagno si trovava un minuto prima: una
          // trappola sostituita da una peggiore, che consuma batteria e sembra
          // attiva.
          //
          // L'idea resta valida - con un collegamento ottico un drone fermo
          // dietro un muro e' irraggiungibile per sempre - ma va realizzata con
          // uno STATO PROPRIO, con uscita per arrivo E per tempo massimo, non
          // riusando un percorso che disattiva l'esplorazione.
          // Ingresso nel ritrovamento: senza celle e con frontiere ancora da
          // fare, muoversi verso un compagno vale piu' che restare fermi. La
          // mappatura continua durante il tragitto - map_ros e il calcolo delle
          // frontiere girano su timer propri, indipendenti dallo stato - quindi
          // il volo non e' tempo perso.
          if (!rdv_active_) {
            const double ora = node_->now().seconds();
            double piu_vicino = std::numeric_limits<double>::max();
            bool trovato = false;
            Eigen::Vector3d meta;
            for (size_t k = 0; k < expl_manager_->ed_->swarm_state_.size(); ++k) {
              const int pid = static_cast<int>(k) + 1;
              if (pid == getId()) continue;
              const auto& ps = expl_manager_->ed_->swarm_state_[k];
              // Anche uno stato vecchio serve: e' l'ultimo posto dove il
              // compagno e' stato visto, ed e' li' che conviene cercarlo.
              if (ps.recv_stamp_ <= 0.0 || ora - ps.recv_stamp_ > 60.0) continue;
              const double d = (ps.pos_ - fd_->odom_pos_).norm();
              if (d < piu_vicino) { piu_vicino = d; meta = ps.pos_; trovato = true; }
            }
            if (trovato && piu_vicino > 2.0) {
              rdv_active_ = true;
              rdv_target_ = meta;
              // Tetto di tempo generoso ma finito: se non lo si raggiunge entro
              // questo, si torna comunque a esplorare.
              rdv_deadline_ = ora + 25.0;
              RCLCPP_WARN(node_->get_logger(),
                  "[Ritrovamento] Drone %d senza lavoro: cerca un compagno a %.1f m", getId(),
                  piu_vicino);
              transitState(PLAN_TRAJ, "ritrovamento");
              break;
            }
          }
          RCLCPP_WARN(node_->get_logger(),
              "Drone %d senza griglie assegnate, ma restano %d frontiere: resta disponibile "
              "(tentativo %d/12)",
              getId(), num_frontiers, fd_->idle_retries_);
          transitState(PLAN_TRAJ, "FSM");
        }
      }
      break;
    }

    case PLAN_TRAJ: {
      // Back off after a failed plan instead of retrying at the timer rate.
      //
      // On FAIL the FSM stays in PLAN_TRAJ, so this 100Hz timer used to re-run
      // the whole planner immediately - and every attempt fires a request at
      // the LKH solver service. A drone that cannot plan therefore floods its
      // own service client: one run logged 3087 planning cycles in 198s (15/s),
      // and the crash that followed was inside FastDDS's send path
      // (rcl_send_request -> DataWriterImpl::write -> RTPSMessageGroup::send),
      // i.e. the writer history overflowing. The runaway cycle counts and the
      // segfaults were the same bug.
      // Una ripianificazione anti-collisione non aspetta: la traiettoria in
      // corso non e' sicura, e se non si trova di meglio si frena.
      if (!fd_->avoid_collision_ && fd_->last_plan_fail_time_.nanoseconds() > 0 &&
          (node_->now() - fd_->last_plan_fail_time_).seconds() < fp_->plan_fail_backoff_) {
        break;
      }
      // Dopo un piano scartato perche' in collisione si riprova fra poco, non
      // al ritmo del timer (100 Hz): la traiettoria in corso e' sicura.
      if (!fd_->avoid_collision_ && node_->now() < riprova_dopo_) break;
      // ESDF SU RICHIESTA. Lo leggono solo l'ottimizzatore e il controllo di
      // partenza, cioe' questa pianificazione: si ricalcola qui, prima di leggere
      // lo stato di partenza (cosi' non allunga il ritardo del piano), invece che a
      // ogni nuvola. Sul Jetson (cubicle_office, 17 ottobre 2026) l'ESDF periodico
      // costava 96 s su 328 di esplorazione nel thread della pianificazione,
      // saturo; il timer (map_ros.esdf_period) resta come rinfresco di riserva.
      expl_manager_->sdf_map_->aggiornaESDFSeServe();
      per_collisione_ = fd_->avoid_collision_;
      anticipo_corrente_ = fd_->avoid_collision_ ? replan_time_collisione_ : fp_->replan_time_;
      partenza_da_traiettoria_ = !fd_->static_state_;
      istante_stato_misurato_ = node_->now();
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (node_->now() - info->start_time_).seconds() + anticipo_corrente_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        // Get yaw angle and derivatives.
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];

        // Re-anchor to reality when the vehicle has fallen behind its own plan.
        //
        // The block above starts the new trajectory from where the drone is
        // *supposed* to be, read off the previous trajectory. That is the only
        // sensible choice while tracking is tight - and in MARSIM, where the
        // paper's results come from, an ideal cascade PID keeps the two within
        // centimetres, so upstream never has to care. With PX4 they diverge:
        // the setpoint was measured over a metre ahead of the vehicle for much
        // of a run, and because every replan re-anchors to the trajectory
        // rather than to the odometry, that error is inherited by the next
        // trajectory and never corrected. It is an open loop on position -
        // which also invalidates the inter-drone separation, since the
        // optimizer keeps trajectories apart, not drones.
        //
        // Past the threshold, plan from where the drone actually is instead,
        // exactly as the static-state branch does. Costs a moment of committed
        // velocity; buys a plan the vehicle can actually be on.
        //
        // L'errore si misura fra il drone e il riferimento di ADESSO, non il
        // punto di partenza del piano (che sta anticipo_corrente_ secondi piu'
        // avanti): con quello, a 1 m/s un drone che inseguiva bene risultava
        // gia' 0.2 m "indietro" per costruzione, e il ritardo normale di PX4
        // (0.3-0.4 m) bastava a superare la soglia. In cubicle_office si
        // ripartiva cosi' dallo stato misurato di continuo, ed era l'innesco del
        // dondolio (vedi callExplorationPlanner, run dell'utente del 5 ottobre).
        const double t_ora =
            std::clamp((node_->now() - info->start_time_).seconds(), 0.0, info->duration_);
        const double track_err =
            (info->position_traj_.evaluateDeBoorT(t_ora) - fd_->odom_pos_).norm();
        if (fp_->replan_reanchor_dist_ > 0.0 && track_err > fp_->replan_reanchor_dist_) {
          rclcpp::Clock clock(RCL_ROS_TIME);
          RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 2000,
              "Drone %d is %.2fm behind its trajectory (limit %.2fm) - replanning from the "
              "actual position instead of the predicted one",
              getId(), track_err, fp_->replan_reanchor_dist_);
          fd_->start_pt_ = fd_->odom_pos_;
          fd_->start_vel_ = fd_->odom_vel_;
          fd_->start_acc_.setZero();
          fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
          partenza_da_traiettoria_ = false;
        } else if (per_collisione_) {
          // RIPIANIFICAZIONE ANTI-COLLISIONE DALLO STATO MISURATO. La
          // traiettoria in corso entra nella fascia di gonfiaggio: partire dal
          // suo stato fra anticipo_corrente_ secondi voleva dire partire gia'
          // diretti contro l'ostacolo, piu' vicini di quanto sia il drone (che
          // le sta 0.3-0.5 m dietro). Ne uscivano piani di nuovo in collisione,
          // ripianificati a loro volta da se' stessi (V27, cubicle_office).
          fd_->start_pt_ = fd_->odom_pos_;
          fd_->start_vel_ = fd_->odom_vel_;
          fd_->start_acc_.setZero();
          fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
          partenza_da_traiettoria_ = false;
        }
      }
      // Il segnale al server di traiettoria TRONCA quella in corso dopo ~0.3 s.
      // Mandato a ogni ripianificazione, se il piano nuovo arrivava piu' tardi
      // il riferimento si fermava di colpo mentre il drone volava: frenata,
      // attesa e ripartenza (il dondolio avanti-indietro: riferimento fermo nel
      // 23-47% dei comandi), e a velocita' alta un sorpasso del riferimento fino
      // a 0.7 m verso la parete (0.30 m di margine, office_small a 1.0 m/s, V21).
      // Ora si tronca solo per scansare un ostacolo; negli altri casi il drone
      // continua sulla traiettoria in corso (gia' verificata) finche' arriva la
      // nuova, o finche' finisce da sola sul punto di vista, frenando come da
      // piano. Le pianificazioni sono ora rapide (memoria dei percorsi), quindi
      // il piano nuovo arriva quasi sempre prima.
      if (fd_->avoid_collision_) replan_pub_->publish(std_msgs::msg::Empty());

      // VELOCITA' DI PARTENZA LIMITATA. Il controllo dei limiti dinamici non
      // tocca lo stato iniziale di una traiettoria (per non frenare il drone a
      // ogni ripianificazione), quindi se la partenza e' gia' oltre il tetto -
      // drone in sovravelocita' dopo un sorpasso del riferimento, ripianificato
      // dalla velocita' misurata - il piano nuovo la mantiene, e quello dopo
      // anche: un circolo che ha portato il drone a 1.9-3 m/s con tetto 1.0 e
      // poi contro una parete (office_small, campagna V24). Qui la partenza si
      // limita a max_vel / max_acc: se il drone va piu' forte, il piano nuovo
      // comincia frenando.
      {
        const double vmax = planner_manager_->pp_.max_vel_;
        const double amax = planner_manager_->pp_.max_acc_;
        if (vmax > 0.0 && fd_->start_vel_.norm() > vmax)
          fd_->start_vel_ *= vmax / fd_->start_vel_.norm();
        if (amax > 0.0 && fd_->start_acc_.norm() > amax)
          fd_->start_acc_ *= amax / fd_->start_acc_.norm();
      }

      traj_prima_valida_ = partenza_da_traiettoria_;
      // Copia sempre: serve anche a rimettere al suo posto la traiettoria in
      // corso quando il piano nuovo e' in collisione (PIANO_IN_COLLISIONE).
      traj_prima_ = planner_manager_->local_data_;
      int res = callExplorationPlanner();
      if (res == PIANO_IN_COLLISIONE) {
        riprova_dopo_ = node_->now() + rclcpp::Duration::from_seconds(0.2);
        break;
      }
      if (res == PIANO_SCADUTO) {
        // Piano scartato perche' arrivato troppo tardi: il drone continua sulla
        // traiettoria in corso (rimessa al suo posto) e si ripianifica subito da
        // li', senza la pausa che segue un fallimento vero.
        break;
      }
      if (res == SUCCEED) {
        fd_->last_plan_fail_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        // Ha ricevuto lavoro: i tentativi a vuoto accumulati in IDLE non contano
        // piu', altrimenti si sommerebbero fra episodi lontani nel tempo e il
        // drone verrebbe ritirato durante un buco transitorio dell'allocazione.
        //
        // MA l'azzeramento non puo' dipendere dall'aver PIANIFICATO: deve
        // dipendere dall'aver fatto PROGRESSI. Legandolo al successo della
        // pianificazione si otteneva un blocco vivo: il drone restava senza
        // celle -> IDLE -> dopo 5 s ritentava -> riusciva a pianificare
        // qualcosa -> contatore a zero -> di nuovo senza celle. Misurato:
        // 75 e 125 rimbalzi IDLE<->PLAN_TRAJ in due run da 437 s, con il
        // contatore arrivato a 12 una sola volta in assoluto. Il drone non
        // progrediva abbastanza per finire ne' falliva abbastanza per
        // arrendersi, e restava fermo a mezz'aria fino allo scadere del tempo.
        //
        // Il volume esplorato e' la grandezza onesta: cresce solo se il drone
        // sta davvero scoprendo spazio. Se e' cresciuto dall'ultimo azzeramento,
        // il drone e' utile e il contatore riparte; se non e' cresciuto, i
        // tentativi continuano ad accumularsi anche quando la pianificazione
        // riesce, ed e' il caso che deve portare alla conclusione.
        const double vol_now = expl_manager_->sdf_map_->getExploredVolume();
        if (fd_->volume_at_retry_reset_ < 0.0 ||
            vol_now > fd_->volume_at_retry_reset_ + fp_->no_progress_min_volume_) {
          fd_->idle_retries_ = 0;
          fd_->volume_at_retry_reset_ = vol_now;
        }
        fd_->plan_fail_streak_ = 0;
        transitState(PUB_TRAJ, "FSM");
      } else if (res == FAIL && completamento_ && !returning_) {
        RCLCPP_WARN(node_->get_logger(),
            "[Fotografie] zona (%.1f %.1f %.1f) non raggiungibile: scartata", zona_foto_.x(),
            zona_foto_.y(), zona_foto_.z());
        zone_scartate_.push_back(zona_foto_);
        if (!prossimaZona()) chiudiCompletamento("nessun'altra zona raggiungibile");
      } else if (res == FAIL) {  // Keep trying to replan
        fd_->static_state_ = true;
        fd_->last_plan_fail_time_ = node_->now();
        RCLCPP_WARN(node_->get_logger(), "Plan fail");
        // Si aggiornano le frontiere PRIMA del prossimo tentativo. Senza, il
        // ciclo di ritentativi riusava gli stessi punti di vista calcolati
        // prima del fallimento: un punto appena messo in lista nera
        // (FrontierFinder::blacklistViewpoint) veniva riproposto a ogni giro -
        // 12 volte lo stesso punto nella run V7_LIO 1 - finche' non scattava
        // l'archiviazione per fallimenti consecutivi.
        {
          std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
          frontiers_seen_ = expl_manager_->updateFrontierStruct(fd_->odom_pos_);
        }
        // FALLIMENTI A RIPETIZIONE: archiviare cio' che non si raggiunge.
        //
        // Senza questo, una frontiera irraggiungibile tiene il drone in
        // hovering a ripianificare a vuoto finche' non scatta il guardiano dei
        // progressi - che pero' non la archivia (vedi
        // banUnreachableFrontiers). Dopo plan_fail_ban_streak fallimenti di
        // fila si archiviano le frontiere irraggiungibili da qui e si passa in
        // IDLE, che conclude se non resta altro.
        if (fp_->plan_fail_ban_streak_ > 0 &&
            ++fd_->plan_fail_streak_ >= fp_->plan_fail_ban_streak_) {
          int banned;
          {
            std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
            banned = expl_manager_->banUnreachableFrontiers(fd_->odom_pos_);
            if (banned == 0) banned = expl_manager_->banHeldFrontiers();
          }
          RCLCPP_WARN(node_->get_logger(),
              "Drone %d: %d pianificazioni fallite di fila - %d frontiere irraggiungibili "
              "da archiviare, passo in IDLE",
              getId(), fd_->plan_fail_streak_, banned);
          fd_->plan_fail_streak_ = 0;
          fd_->last_check_frontier_time_ = node_->now();
          // Niente troncamento: vedi la nota "FERMATE DI COLPO" su "Idle since no frontier".
          transitState(IDLE, "pianificazione");
        }
      } else if (res == NO_GRID) {
        // No grid available, exploration is finished
        fd_->static_state_ = true;
        fd_->last_check_frontier_time_ = node_->now();
        RCLCPP_WARN(node_->get_logger(), "No grid");
        transitState(IDLE, "FSM");
        visualize(1);
        // clearVisMarker();
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (node_->now() - rclcpp::Time(fd_->newest_traj_.start_time, RCL_ROS_TIME)).seconds();
      if (dt > 0) {
        if (fd_->expl_start_time_.nanoseconds() == 0) {
          fd_->expl_start_time_ = node_->now();
        }
        // Stamp the owner *before* publishing: swarmTrajCallback on the peers
        // keys everything off drone_id, and a message that went out with the
        // default 0 is discarded there (drone_id - 1 would index at -1).
        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        bspline_pub_->publish(fd_->newest_traj_);
        fd_->static_state_ = false;
        traiettoria_pubblicata_ = true;

        // Hand the freshly committed trajectory to the peers right away rather
        // than waiting up to 100ms for swarm_traj_timer_ - this is the one the
        // others' optimizer needs for the inter-drone avoidance term.
        swarm_traj_pub_->publish(fd_->newest_traj_);

        thread vis_thread(&C2ExplorationFSM::visualize, this, 2);
        vis_thread.detach();

        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      if (completamento_ && !returning_) {
        // Volo verso il punto di vista, poi una sosta di 1 s (imbardata
        // assestata, fotogrammi scattati), poi la zona successiva. Le
        // ripianificazioni dell'esplorazione qui non servono; quelle
        // anti-collisione passano comunque dal timer di sicurezza.
        LocalTrajData* traj_ora = &planner_manager_->local_data_;
        const double tc = (node_->now() - traj_ora->start_time_).seconds();
        if (node_->now().seconds() > completamento_scadenza_) {
          chiudiCompletamento("tempo massimo della fase");
          break;
        }
        // VISITA: la zona e' fatta appena la sua meta risulta visitata (drone
        // passato entro il raggio, vista libera), senza aspettare la fine della
        // traiettoria ne' fermarsi. Chiedere l'arrivo entro 0.5 m come per le
        // foto costava 7-47 s per meta "non raggiunta" (5 su 17 in una run V32).
        if (zona_di_visita_) {
          bool fatta;
          {
            std::lock_guard<std::mutex> lk(copertura_mutex_);
            fatta = visita_->visitata(meta_foto_);
            // La meta risulta visitata appena il drone le passa entro visit_radius (2 m),
            // spesso con meta' zona ancora scoperta dietro di lei: la zona tornava come
            // zona "nuova" col centro 10-60 cm piu' in la' e il drone ci arrivava a passi,
            // una ripianificazione per passo (sciame, ottobre 2026: 5 passi per una zona
            // di 8 m2 nell'angolo sud di office_small, visita del drone 2 durata 44 s). Si
            // prosegue invece sulla traiettoria in corso finche' il drone non e' a 1 m
            // dalla meta o attorno a lei (1.5 m) non resta quasi nulla da visitare.
            if (fatta && (fd_->odom_pos_ - meta_foto_).head<2>().norm() > 1.0 &&
                visita_->nonVisitateEntro(meta_foto_, 1.5) > 20)
              fatta = false;
          }
          if (fatta) {
            ++zone_visitate_;
            // Niente zone_scartate_ per le visite riuscite: in una zona lunga
            // (una striscia lungo una parete) la meta copre solo i 2 m attorno,
            // e scartandone il centro i pezzi rimasti venivano ignorati (V34:
            // 20 m2 lungo la parete est di cubicle_office). I pezzi rimasti
            // tornano come zone nuove con una meta non ancora visitata, quindi
            // ogni visita fa progressi e non si gira in tondo.
            if (prossimaZona()) {
              fd_->static_state_ = true;
              transitState(PLAN_TRAJ, "visita");
            } else {
              chiudiCompletamento("spazio visitato");
            }
            break;
          }
        }
        if (tc < traj_ora->duration_ - 0.05) break;
        // ARRIVATO DAVVERO? Per mete oltre 7 m il pianificatore traccia solo i
        // primi 7 m (planTrajToView): la fine della traiettoria non e' il punto
        // di vista. Prima la si prendeva per arrivo, si sostava a meta' strada e
        // la zona veniva data per fatta senza averla inquadrata (V26: la parete
        // est della stanza centrale, 15 m piu' in la', rimasta grigia).
        // ...E ASPETTANDO IL DRONE. Il tempo della traiettoria qui e' quello di
        // parete, mentre il server di traiettoria rallenta il riferimento
        // quando il drone resta indietro: a tempo scaduto il drone era spesso
        // ancora a 0.5-1 m dalla meta, si ripartiva con un altro tratto corto
        // (stesso problema) e dopo 6 la zona veniva scartata. Su cubicle_office
        // (V28) 19 zone su 31 perse cosi', 6-35 s ciascuna. Si decide solo con
        // il drone arrivato (entro 0.5 m) o fermo, al massimo 3 s dopo.
        const double scarto_meta = (fd_->odom_pos_ - meta_foto_).norm();
        if (scarto_meta > 0.5 && fd_->odom_vel_.norm() > 0.15 &&
            tc < traj_ora->duration_ + 3.0) {
          break;
        }
        if (scarto_meta > 0.5) {
          in_sosta_foto_ = false;
          if (++tentativi_foto_ <= 6) {
            fd_->static_state_ = true;
            transitState(PLAN_TRAJ, "fotografie");
          } else {
            RCLCPP_WARN(node_->get_logger(),
                "[Fotografie] punto di vista (%.1f %.1f %.1f) non raggiunto dopo %d tratti: zona "
                "scartata", meta_foto_.x(), meta_foto_.y(), meta_foto_.z(), tentativi_foto_ - 1);
            zone_scartate_.push_back(zona_foto_);
            if (prossimaZona()) {
              fd_->static_state_ = true;
              transitState(PLAN_TRAJ, "fotografie");
            } else {
              chiudiCompletamento("nessun'altra zona raggiungibile");
            }
          }
          break;
        }
        if (zona_di_visita_) {
          // Visita: arrivati basta, niente sosta (vedi sopra per lo scarto).
          ++zone_visitate_;
          if (prossimaZona()) {
            fd_->static_state_ = true;
            transitState(PLAN_TRAJ, "visita");
          } else {
            chiudiCompletamento("mappa visitata e inquadrata");
          }
          break;
        }
        if (!in_sosta_foto_) {
          in_sosta_foto_ = true;
          arrivo_foto_ = node_->now();
          break;
        }
        // Sosta: almeno 1 s, e finche' la telecamera e' rivolta alla zona (entro
        // 20 gradi), ma non oltre 2.5 s.
        const double sosta = (node_->now() - arrivo_foto_).seconds();
        const double err_yaw =
            std::abs(std::remainder(fd_->odom_yaw_ - yaw_foto_, 2.0 * M_PI));
        if (sosta < 1.0 || (err_yaw > 0.35 && sosta < 2.5)) break;
        in_sosta_foto_ = false;
        ++zone_fatte_;
        // Zona fatta: non si ripropone anche se ne resta qualche cella
        // (parti nascoste), cosi' la fase non gira in tondo.
        zone_scartate_.push_back(zona_foto_);
        if (prossimaZona()) {
          fd_->static_state_ = true;
          transitState(PLAN_TRAJ, "fotografie");
        } else {
          chiudiCompletamento("mappa 3D inquadrata");
        }
        break;
      }
      auto tn = node_->now();
      // Check whether replan is needed
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).seconds();

      if (!fd_->go_back_) {
        bool need_replan = false;
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
          RCLCPP_INFO(node_->get_logger(), "Replan: cluster covered=====================================");
          need_replan = true;
        } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan if traj is almost fully executed
          RCLCPP_INFO(node_->get_logger(), "Replan: traj fully executed=================================");
          need_replan = true;
        } else if (t_cur > fp_->replan_thresh3_) {
          // Replan after some time
          RCLCPP_INFO(node_->get_logger(), "Replan: periodic call=======================================");
          need_replan = true;
        }

        // RIENTRO: la ripianificazione periodica prosegue verso casa. Prima
        // passava dal ramo delle frontiere: a esplorazione finita non ce ne
        // sono, quindi IDLE + segnale di ripianificazione al server di
        // traiettoria (che tronca quella in corso) + ripartenza dallo stato
        // misurato. Ogni 1.5 s il drone frenava: rientri di 11 m in 20-22 s,
        // 12 tappe, velocita' a strappi fra 0.1 e 0.8 m/s (campagna V14).
        // Qui si pianifica la tappa successiva partendo dalla traiettoria in
        // corso, come in esplorazione.
        if (need_replan && returning_) {
          transitState(PLAN_TRAJ, "rientro");
          need_replan = false;
        }
        if (need_replan) {
          // New frontiers found.
          //
          // Il lucchetto qui MANCAVA, ed e' l'unica delle quattro chiamate a
          // updateFrontierStruct che ne fosse priva (le altre tre a riga 244,
          // 678 e 717 lo prendono). updateFrontierStruct ricostruisce
          // frontiers_/dormant_frontiers_, e i thread di visualizzazione
          // staccati poco sotto iterano quelle stesse liste tenendo
          // frontier_mutex_: un lucchetto protegge solo se lo prendono anche
          // gli scrittori, quindi qui non proteggeva nulla. E' la sorgente dei
          // segfault con firma getPathForTour -> searchPath -> free() ->
          // malloc_printerr, cioe' heap corrotto, che comparivano in modo
          // intermittente.
          int num_ftr;
          {
            std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
            num_ftr = expl_manager_->updateFrontierStruct(fd_->odom_pos_);
            frontiers_seen_ = num_ftr;
          }
          if (num_ftr != 0) {
            // Update frontier and plan new motion
            thread vis_thread(&C2ExplorationFSM::visualize, this, 1);
            vis_thread.detach();
            transitState(PLAN_TRAJ, "FSM");
          } else {
            // No frontier detected, go to IDLE and wait
            fd_->last_check_frontier_time_ = node_->now();
            transitState(IDLE, "FSM");
            RCLCPP_WARN(node_->get_logger(), "Idle since no frontier is detected");
            fd_->static_state_ = true;
            // FERMATE DI COLPO. Qui (e negli altri passaggi a IDLE o al rientro)
            // si mandava il segnale che fa troncare a traj_server la traiettoria
            // in corso dopo ~0.3 s: il riferimento si fermava di colpo con il
            // drone a ~1 m/s, che lo superava di 0.6 m, tornava indietro, lo
            // risuperava e si assestava dopo ~5 s. E' il "dondolio avanti e
            // indietro" visto a fine esplorazione (run del 6 ottobre 2026,
            // office_small). Ora la traiettoria in corso finisce da sola,
            // frenando come da piano sul punto di vista; il piano successivo
            // parte dallo stato misurato (static_state_). Il troncamento resta
            // solo per le ripianificazioni anti-collisione.
            // clearVisMarker();
            visualize(1);
          }
        }
      } else {
        // Check if reach goal
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        // Consider the goal reached within 1 m.
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_->publish(std_msgs::msg::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        // Goal not reached, replan.
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          // Replan for going back
          replan_pub_->publish(std_msgs::msg::Empty());
          transitState(PLAN_TRAJ, "FSM");
          thread vis_thread(&C2ExplorationFSM::visualize, this, 1);
          vis_thread.detach();
        }
      }

      break;
    }
  }
}

// Fine dell'esplorazione, qualunque ne sia il motivo.
//
// Se il drone e' lontano dalla partenza ci torna (vedi il rientro in
// FSMCallback, con la sua scadenza); se e' gia' vicino ATTERRA qui.
// Prima, in quel secondo caso, si passava a FINISH senza chiedere
// l'atterraggio: il drone restava in hovering per sempre e lo script
// aspettava invano l'atterraggio fino al proprio timeout.
void C2ExplorationFSM::concludiEsplorazione(const char* motivo) {
  if (finale_) return;
  // Chi ha concluso l'esplorazione non tiene piu' celle: i compagni (spartizione a strisce)
  // considerano libere solo le celle che nessuno tiene, e con l'ultimo elenco pubblicato le
  // frontiere residue restavano "prese" da un drone che non ci lavorava piu' (run 1234).
  {
    std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
    expl_manager_->ed_->swarm_state_[getId() - 1].grid_ids_.clear();
  }
  // COMPLETAMENTO FOTOGRAFICO prima del rientro (non dopo un tempo massimo:
  // li' si rientra subito con quello che si ha).
  if (completamento_) {
    // Richiamata durante la fase (es. dopo una serie di piani falliti): si
    // prova la zona successiva, altrimenti si chiude e si rientra.
    if (prossimaZona()) {
      transitState(PLAN_TRAJ, "fotografie");
      return;
    }
    completamento_ = false;
    completamento_fatto_ = true;
  } else if ((completamento_attivo_ || visita_attiva_) && !completamento_fatto_ && !returning_ &&
             fd_->trigger_ &&
             std::string(motivo) != "tempo massimo") {
    completamento_scadenza_ = node_->now().seconds() + completamento_durata_;
    zone_scartate_.clear();
    zone_fatte_ = 0;
    zone_visitate_ = 0;
    if (prossimaZona()) {
      completamento_ = true;
      fd_->static_state_ = true;
      RCLCPP_WARN(node_->get_logger(),
          "[Fotografie] Drone %d: frontiere finite (%s) - completamento (visita%s) prima del "
          "rientro", getId(), motivo, completamento_attivo_ ? " e foto" : "");
      transitState(PLAN_TRAJ, "fotografie");
      return;
    }
    completamento_fatto_ = true;
  }
  const bool lontano = (fd_->odom_pos_ - fd_->start_pos_).norm() > 2.0;
  if (!returning_ && fd_->trigger_ && lontano) {
    returning_ = true;
    rdv_active_ = false;   // il rientro ha la precedenza sulla ricerca
    // RIENTRO IN QUOTA ALTA, non al punto esatto di partenza.
    //
    // Puntare alla posizione di partenza faceva fallire la pianificazione: 16
    // "no path" e 32 fallimenti consecutivi, con il drone che restava fermo
    // fino allo scadere del tempo e atterrava dov'era - su 5 rientri, uno solo
    // arrivava. La linea di partenza ha margini di 0.8-1.3 m, e col gonfiaggio
    // degli ostacoli (0.2 m per lato) piu' l'ingombro del velivolo non resta
    // una cella libera. Si torna quindi sopra la verticale della partenza, a
    // quota di sicurezza dove lo spazio e' libero; la discesa la fa
    // l'atterraggio dell'autopilota, che e' verticale per costruzione.
    return_target_ = fd_->start_pos_;
    return_target_.z() = return_altitude_;
    // Tetto proporzionale alla strada (almeno 60 s, poi 20 s + 3 s per metro, cioe' un terzo
    // della velocita' di crociera con margine per il traffico). Con 60 s fissi, a tre droni su
    // cubicle_office il drone 3 doveva rientrare da 31.6 m lungo il corridoio delle partenze
    // affollato dai compagni: tempo scaduto e atterraggio lontano da casa (ottobre 2026).
    return_deadline_ = node_->now().seconds() +
        std::max(60.0, 20.0 + 3.0 * (fd_->odom_pos_ - fd_->start_pos_).norm());
    RCLCPP_WARN(node_->get_logger(), "[Rientro] Drone %d torna alla partenza (%.1f m, %s)",
        getId(), (fd_->odom_pos_ - fd_->start_pos_).norm(), motivo);
    transitState(PLAN_TRAJ, "rientro");
    return;
  }
  if (returning_) return;  // gia' in rientro: le uscite sono in FSMCallback
  RCLCPP_WARN(node_->get_logger(), "[Rientro] Drone %d gia' alla partenza (%s): atterra", getId(),
      motivo);
  home_land_pub_->publish(std_msgs::msg::Empty());
  transitState(FINISH, "conclusione");
  finale_ = true;
}

// Prossima zona non inquadrata da fotografare: la piu' vicina al drone fra quelle
// non ancora scartate per cui esiste un punto di vista valido.
// Prossima meta della fase di completamento: prima le zone mai visitate, poi
// (se la fase fotografica e' attiva) quelle mai inquadrate.
bool C2ExplorationFSM::prossimaZona() {
  if (visita_attiva_ && prossimaZonaVisita()) return true;
  if (completamento_attivo_ && prossimaZonaFoto()) return true;
  return false;
}

bool C2ExplorationFSM::prossimaZonaVisita() {
  std::vector<ZonaDaVisitare> zone;
  double frazione = 1.0;
  // Con la spartizione a strisce le zone si tagliano ai confini delle strisce: in una stanza
  // vuota tutto lo spazio non visitato formava UNA zona, il cui centro cadeva nella striscia di
  // un solo drone, e gli altri restavano senza zone e rientravano (empty_room, ottobre 2026).
  bool strisce = false;
  {
    std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
    strisce = expl_manager_->ep_->drone_num_ > 1 && expl_manager_->spartizioneAStrisce();
  }
  {
    std::lock_guard<std::mutex> lk(copertura_mutex_);
    if (strisce) {
      std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
      visita_->zone(zone, frazione, [this](const Eigen::Vector3d& p) {
        return expl_manager_->proprietarioStriscia(p);
      });
    } else {
      visita_->zone(zone, frazione);
    }
  }
  const Eigen::Vector3d pos = fd_->odom_pos_;
  // SPARTIZIONE FRA I DRONI (sciame, ottobre 2026). Compagni ancora attivi (non in
  // rientro) con uno stato recente: ogni zona va al drone che la paga meno, cioe'
  // distanza in linea d'aria + 0.25 m per ogni m2 di zone che ha gia' avuto (una visita
  // spazza una striscia larga ~2 x visit_radius = 4 m: un m2 costa ~0.25 m di volo), a
  // partire dalle zone piu' grandi. Ogni drone fa lo stesso conto con le posizioni di
  // tutti e tiene le sue; le mappe sono condivise ma non identiche, quindi le spartizioni
  // possono differire di poco: per questo si escludono anche le zone attorno alla meta
  // dichiarata da un compagno. Un drone senza zone proprie non rientra finche' ce ne sono
  // di altri non ancora prese: aiuta (le prende in ordine di giro come le sue). Chi
  // esplora ancora a frontiere conta come attivo: le zone nella sua area restano a lui,
  // che ci passera' comunque.
  int zone_totali = static_cast<int>(zone.size()), zone_mie = zone_totali;
  // Zone dei compagni non ancora prese: si provano DOPO le proprie, e solo se nessuna delle
  // proprie e' raggiungibile. Prima si ripiegava su di loro solo con zero zone proprie in
  // partenza: su cubicle_office il drone 2 aveva una sola zona sua, irraggiungibile, ed e'
  // rientrato lasciando ai compagni 4 zone, poi visitate dal drone 3 in 86 s (ottobre 2026).
  std::vector<ZonaDaVisitare> zone_altrui;
  {
    std::vector<Eigen::Vector3d> posti{ pos }, mete_altrui;
    {
      std::lock_guard<std::mutex> lp(pari_mutex_);
      const double ora = node_->now().seconds();
      for (size_t k = 0; k < pari_visita_.size(); ++k) {
        if (static_cast<int>(k) + 1 == getId()) continue;
        const auto& p = pari_visita_[k];
        if (p.recv <= 0.0 || ora - p.recv > 5.0 || p.fase >= 2) continue;
        posti.push_back(p.pos);
        if (p.ha_meta) mete_altrui.push_back(p.meta);
      }
    }
    if (posti.size() > 1 && !zone.empty()) {
      const double area_cella = std::pow(expl_manager_->sdf_map_->getResolution(), 2);
      auto presa = [&](const ZonaDaVisitare& z) {
        for (const auto& m : mete_altrui)
          if ((m - z.centro).head<2>().norm() < 2.5) return true;
        return false;
      };
      std::vector<size_t> ordine(zone.size());
      for (size_t k = 0; k < ordine.size(); ++k) ordine[k] = k;
      std::sort(ordine.begin(), ordine.end(),
          [&](size_t a, size_t b) { return zone[a].celle > zone[b].celle; });
      std::vector<double> carico(posti.size(), 0.0);
      std::vector<bool> mia(zone.size(), false);
      // Con la griglia spartita a strisce ogni zona e' del padrone della striscia che la
      // contiene: la stessa spartizione dell'esplorazione, identica su tutti i droni e stabile.
      // Con la regola per vicinanza (sotto) chi finiva l'esplorazione aiutando dall'altra parte
      // della mappa si prendeva le zone di la', e poi attraversava la mappa per le proprie: a
      // tre droni su cubicle_office, visita di 181-226 s contro i 172 del drone singolo.
      bool per_strisce = false;
      {
        std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
        if (expl_manager_->spartizioneAStrisce()) {
          per_strisce = true;
          for (size_t k = 0; k < zone.size(); ++k)
            mia[k] = expl_manager_->proprietarioStriscia(zone[k].centro) == getId();
        }
      }
      if (!per_strisce)
      for (const size_t k : ordine) {
        size_t migliore = 0;
        double costo_min = std::numeric_limits<double>::max();
        for (size_t d = 0; d < posti.size(); ++d) {
          const double c = (posti[d] - zone[k].centro).head<2>().norm() + 0.25 * carico[d];
          if (c < costo_min) {
            costo_min = c;
            migliore = d;
          }
        }
        carico[migliore] += zone[k].celle * area_cella;
        mia[k] = (migliore == 0);
      }
      // Con le strisce si aiuta SOLO nelle strisce di chi ha gia' finito (in rientro o
      // atterrato, o senza stato recente): le zone di un compagno ancora al lavoro restano sue.
      // Aiutandolo mentre lavora, a tre droni su cubicle_office i droni attraversavano la mappa
      // per zone da 1-3 m2 e si intralciavano: 28 zone a 15.5 s mediani l'una, contro 15 a 5.1 s
      // del drone singolo (ottobre 2026).
      // ...con UNA eccezione: una zona di un compagno al lavoro si prende se e' nettamente
      // piu' vicina a questo drone che a lui (meno del 70% della sua distanza). Senza, chi
      // finiva la propria striscia rientrava mentre un compagno aveva ancora un minuto e mezzo
      // di zone (run 1036 su cubicle_office: drone 2 a casa a +171 s, drone 3 in visita fino a
      // +292 s).
      // ...e una zona di un compagno GIA' IN VISITA si prende se ci si arriva prima di lui
      // contando il giro che ha davanti: la strada che farebbe per raggiungerla passando prima
      // dalle sue zone piu' vicine (vicino piu' vicino dalla sua posizione), con un margine
      // del 20%. Con la sola distanza in linea d'aria chi finiva rientrava anche quando il
      // compagno aveva ancora un giro lungo: office_small a tre droni, drone 3 a casa a +31 s
      // e drone 2 in visita da solo fino a +58 s (ottobre 2026). Le zone del compagno vicine
      // alla sua meta restano escluse (presa), e lui esclude la mia appena la dichiaro.
      std::map<int, std::pair<Eigen::Vector3d, int>> compagni_al_lavoro;  // id -> (pos, fase)
      if (per_strisce) {
        std::lock_guard<std::mutex> lp(pari_mutex_);
        const double ora = node_->now().seconds();
        for (size_t k = 0; k < pari_visita_.size(); ++k) {
          const auto& p = pari_visita_[k];
          if (static_cast<int>(k) + 1 != getId() && p.recv > 0.0 && ora - p.recv < 5.0 &&
              p.fase < 2)
            compagni_al_lavoro[static_cast<int>(k) + 1] = { p.pos, p.fase };
        }
      }
      std::vector<int> padrone_zona(zone.size(), -1);
      if (per_strisce) {
        std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
        for (size_t k = 0; k < zone.size(); ++k)
          padrone_zona[k] = expl_manager_->proprietarioStriscia(zone[k].centro);
      }
      std::vector<double> strada_padrone(zone.size(), 0.0);
      for (const auto& kv : compagni_al_lavoro) {
        if (kv.second.second < 1) continue;  // esplora ancora: vale la distanza diretta
        std::vector<size_t> sue;
        for (size_t k = 0; k < zone.size(); ++k)
          if (padrone_zona[k] == kv.first) sue.push_back(k);
        std::vector<bool> fatta(sue.size(), false);
        Eigen::Vector3d qui = kv.second.first;
        double strada = 0.0;
        for (size_t n = 0; n < sue.size(); ++n) {
          size_t best = 0;
          double dbest = std::numeric_limits<double>::max();
          for (size_t q = 0; q < sue.size(); ++q) {
            if (fatta[q]) continue;
            const double d = (zone[sue[q]].centro - qui).head<2>().norm();
            if (d < dbest) {
              dbest = d;
              best = q;
            }
          }
          fatta[best] = true;
          strada += dbest;
          strada_padrone[sue[best]] = strada;
          qui = zone[sue[best]].centro;
        }
      }
      std::vector<ZonaDaVisitare> proprie, libere;
      for (size_t k = 0; k < zone.size(); ++k) {
        if (presa(zone[k])) continue;
        if (!mia[k] && per_strisce) {
          const auto it = compagni_al_lavoro.find(padrone_zona[k]);
          if (it != compagni_al_lavoro.end()) {
            const double mia_strada = (pos - zone[k].centro).head<2>().norm();
            const double sua_strada = it->second.second >= 1 ?
                0.8 * strada_padrone[k] :
                0.7 * (it->second.first - zone[k].centro).head<2>().norm();
            if (mia_strada >= sua_strada) continue;
          }
        }
        (mia[k] ? proprie : libere).push_back(zone[k]);
      }
      zone_mie = static_cast<int>(proprie.size());
      zone.swap(proprie);
      zone_altrui.swap(libere);
    }
  }
  // ORDINE A GIRO: per visitare TUTTO conta la strada totale, non la resa
  // della singola zona. Con l'ordine per resa (area per metro) il drone
  // attraversava la mappa avanti e indietro e due run V32 su tre arrivavano
  // al tetto di tempo. Qui: giro aperto dal drone attraverso i centri delle
  // zone (vicino piu' vicino + 2-opt) e si va alla prima del giro.
  auto ordinaGiro = [&pos](std::vector<ZonaDaVisitare>& zone) {
    std::vector<ZonaDaVisitare> giro;
    std::vector<bool> usata(zone.size(), false);
    Eigen::Vector3d qui = pos;
    for (size_t n = 0; n < zone.size(); ++n) {
      size_t best = 0;
      double dbest = std::numeric_limits<double>::max();
      for (size_t k = 0; k < zone.size(); ++k) {
        if (usata[k]) continue;
        const double d = (zone[k].centro - qui).head<2>().norm();
        if (d < dbest) {
          dbest = d;
          best = k;
        }
      }
      usata[best] = true;
      giro.push_back(zone[best]);
      qui = zone[best].centro;
    }
    auto punto = [&](int k) { return k < 0 ? pos : giro[k].centro; };
    auto dd = [&](int a, int b) { return (punto(a) - punto(b)).head<2>().norm(); };
    bool migliorato = true;
    for (int giri = 0; migliorato && giri < 50; ++giri) {
      migliorato = false;
      const int n = static_cast<int>(giro.size());
      for (int i = 0; i < n - 1; ++i)
        for (int j = i + 1; j < n; ++j) {
          // inverte il tratto i..j (giro aperto: dopo j puo' non esserci nulla)
          const double prima = dd(i - 1, i) + (j + 1 < n ? dd(j, j + 1) : 0.0);
          const double dopo = dd(i - 1, j) + (j + 1 < n ? dd(i, j + 1) : 0.0);
          if (dopo < prima - 1e-6) {
            std::reverse(giro.begin() + i, giro.begin() + j + 1);
            migliorato = true;
          }
        }
    }
    zone.swap(giro);
  };
  ordinaGiro(zone);
  ordinaGiro(zone_altrui);
  zone.insert(zone.end(), zone_altrui.begin(), zone_altrui.end());
  for (const auto& z : zone) {
    bool scartata = false;
    for (const auto& v : zone_scartate_)
      if ((v - z.centro).norm() < 0.8) scartata = true;
    if (scartata) continue;
    // Meta: la cella della zona piu' vicina al suo centro con lo stesso
    // margine dagli ostacoli dei punti di vista dell'esplorazione.
    bool trovata = false;
    for (const auto& m : z.mete) {
      if (spazioLibero(m, 0.55)) {
        meta_foto_ = m;
        trovata = true;
        break;
      }
    }
    if (!trovata) {
      zone_scartate_.push_back(z.centro);
      continue;
    }
    yaw_foto_ = std::atan2(meta_foto_.y() - pos.y(), meta_foto_.x() - pos.x());
    zona_foto_ = z.centro;
    zona_di_visita_ = true;
    scadenza_zona_ = node_->now().seconds() + 15.0 + 1.5 * (meta_foto_ - pos).head<2>().norm();
    tentativi_foto_ = 0;
    in_sosta_foto_ = false;
    RCLCPP_WARN(node_->get_logger(),
        "[Visita] spazio percorribile visitato %.1f%%, %d zone mancanti (%d mie, %d visitate): "
        "prossima a (%.1f %.1f), %.1f m2, meta (%.1f %.1f %.1f)",
        100.0 * frazione, zone_totali, zone_mie, zone_visitate_, z.centro.x(), z.centro.y(),
        z.celle * std::pow(expl_manager_->sdf_map_->getResolution(), 2), meta_foto_.x(),
        meta_foto_.y(), meta_foto_.z());
    return true;
  }
  RCLCPP_WARN(node_->get_logger(),
      "[Visita] spazio percorribile visitato %.1f%%: nessun'altra zona da visitare (%d mancanti, "
      "%d mie, %zu scartate)", 100.0 * frazione, zone_totali, zone_mie, zone_scartate_.size());
  return false;
}

bool C2ExplorationFSM::prossimaZonaFoto() {
  zona_di_visita_ = false;
  std::vector<ZonaNonInquadrata> zone;
  double frazione = 1.0;
  {
    std::lock_guard<std::mutex> lk(copertura_mutex_);
    copertura_->zone(zone, frazione);
  }
  // ORDINE PER RESA, non per vicinanza: superficie della zona per metro di
  // strada (piu' 3 m, il costo fisso di andarci e fermarsi). Prendendo sempre
  // la piu' vicina, su cubicle_office i 300 s della fase finivano su decine di
  // zone da 20 celle sparse, e il blocco grande rimasto grigio (l'angolo
  // nord-ovest, oltre 1000 celle fra pavimento e parete) non veniva mai
  // raggiunto: superficie inquadrata dall'84.7% all'85.1% in 300 s (V29).
  const Eigen::Vector3d pos = fd_->odom_pos_;
  auto resa = [&](const ZonaNonInquadrata& z) {
    return static_cast<double>(z.celle) / ((z.centro - pos).norm() + 3.0);
  };
  std::sort(zone.begin(), zone.end(), [&](const ZonaNonInquadrata& a, const ZonaNonInquadrata& b) {
    return resa(a) > resa(b);
  });
  for (const auto& z : zone) {
    bool scartata = false;
    for (const auto& v : zone_scartate_)
      if ((v - z.centro).norm() < 0.8) scartata = true;
    if (scartata) continue;
    Eigen::Vector3d vp;
    double yaw;
    if (!puntoDiVistaFoto(z, vp, yaw)) {
      zone_scartate_.push_back(z.centro);
      continue;
    }
    meta_foto_ = vp;
    yaw_foto_ = yaw;
    zona_foto_ = z.centro;
    tentativi_foto_ = 0;
    RCLCPP_WARN(node_->get_logger(),
        "[Fotografie] superficie inquadrata %.1f%%, %zu zone mancanti (%d fatte): prossima a "
        "(%.1f %.1f %.1f), %d celle, punto di vista (%.1f %.1f %.1f) imbardata %.0f gradi",
        100.0 * frazione, zone.size(), zone_fatte_, z.centro.x(), z.centro.y(), z.centro.z(),
        z.celle, vp.x(), vp.y(), vp.z(), yaw * 180.0 / M_PI);
    return true;
  }
  RCLCPP_WARN(node_->get_logger(),
      "[Fotografie] superficie inquadrata %.1f%%, nessuna zona raggiungibile (%zu mancanti, %zu "
      "scartate)", 100.0 * frazione, zone.size(), zone_scartate_.size());
  return false;
}

// Spazio libero (nessuna cella occupata) entro 'raggio' da p: lo stesso margine
// dei punti di vista dell'esplorazione. Non si usa la distanza dell'ESDF perche'
// e' aggiornata solo attorno al drone.
bool C2ExplorationFSM::spazioLibero(const Eigen::Vector3d& p, double raggio) {
  auto mappa = expl_manager_->sdf_map_;
  const double res = mappa->getResolution();
  if (!mappa->isInBox(p) || mappa->getOccupancy(p) != SDFMap::FREE) return false;
  for (double dx = -raggio; dx <= raggio + 1e-6; dx += res)
    for (double dy = -raggio; dy <= raggio + 1e-6; dy += res)
      for (double dz = -0.3; dz <= 0.3 + 1e-6; dz += res) {
        if (dx * dx + dy * dy > raggio * raggio) continue;
        const Eigen::Vector3d q = p + Eigen::Vector3d(dx, dy, dz);
        if (!mappa->isInMap(q) || mappa->getOccupancy(q) == SDFMap::OCCUPIED) return false;
      }
  return true;
}

// Punto di vista per una zona: davanti alla superficie (lungo la sua normale,
// con qualche angolazione) a 1.2-3.2 m, alla quota di volo, in spazio libero,
// con la telecamera rivolta alla zona; vince quello che ne inquadra di piu',
// a parita' il piu' vicino al drone. Per il pavimento si gira attorno alla zona.
bool C2ExplorationFSM::puntoDiVistaFoto(
    const ZonaNonInquadrata& z, Eigen::Vector3d& vp, double& yaw) {
  Eigen::Vector3d nh(z.normale.x(), z.normale.y(), 0.0);
  std::vector<Eigen::Vector3d> direzioni;
  if (nh.norm() > 0.3) {
    nh.normalize();
    for (double a : { 0.0, 0.5, -0.5, 1.0, -1.0 })
      direzioni.push_back(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()) * nh);
  } else {
    for (int k = 0; k < 8; ++k)
      direzioni.push_back(Eigen::Vector3d(std::cos(k * M_PI / 4), std::sin(k * M_PI / 4), 0.0));
  }
  const int minimo = std::max(3, static_cast<int>(0.3 * z.campioni.size()));
  double migliore = -1e9;
  bool trovato = false;
  for (double d : { 1.2, 1.8, 2.5, 3.2 }) {
    for (const auto& dir : direzioni) {
      for (double zq : { 1.2, 1.5, 1.0 }) {
        Eigen::Vector3d p = z.centro + dir * d;
        p.z() = zq;
        if (!spazioLibero(p, 0.55)) continue;
        const double y = std::atan2(z.centro.y() - p.y(), z.centro.x() - p.x());
        // Conta solo cio' che si vede abbastanza di FRONTE (entro 60 gradi dalla
        // normale della superficie), pesato per quanto e' frontale: viste di
        // taglio lasciano la superficie grigia nella mappa 3D anche quando e'
        // nell'inquadratura.
        int vis = 0;
        double peso = 0.0;
        const Eigen::Vector3d cam = p + Eigen::Vector3d(0.0, 0.0, copertura_->parametri().offset.z());
        for (const auto& c : z.campioni) {
          const double cos_inc = (cam - c).normalized().dot(z.normale);
          if (cos_inc < 0.5) continue;
          if (copertura_->visibileDa(p, y, c)) {
            ++vis;
            peso += cos_inc;
          }
        }
        if (vis < minimo) continue;
        const double punteggio = peso - 0.3 * (p - fd_->odom_pos_).norm();
        if (punteggio > migliore) {
          migliore = punteggio;
          vp = p;
          yaw = y;
          trovato = true;
        }
      }
    }
  }
  return trovato;
}

void C2ExplorationFSM::chiudiCompletamento(const char* perche) {
  completamento_ = false;
  completamento_fatto_ = true;
  in_sosta_foto_ = false;
  std::vector<ZonaNonInquadrata> zone;
  double frazione = 1.0;
  {
    std::lock_guard<std::mutex> lk(copertura_mutex_);
    copertura_->zone(zone, frazione);
  }
  double visitato = 1.0;
  {
    std::vector<ZonaDaVisitare> zv;
    std::lock_guard<std::mutex> lk(copertura_mutex_);
    visita_->zone(zv, visitato);
  }
  RCLCPP_WARN(node_->get_logger(),
      "[Fotografie] fase conclusa (%s): %d zone visitate, %d fotografate, spazio percorribile "
      "visitato %.1f%%, superficie inquadrata %.1f%% - rientro",
      perche, zone_visitate_, zone_fatte_, 100.0 * visitato, 100.0 * frazione);
  fd_->static_state_ = true;
  concludiEsplorazione("fotografie concluse");
}

int C2ExplorationFSM::callExplorationPlanner() {
  rclcpp::Clock clock_rdv_(RCL_ROS_TIME);
  rclcpp::Time time_r = node_->now() + rclcpp::Duration::from_seconds(anticipo_corrente_);
  rclcpp::Time t1 = node_->now();

  int res;
  // Simple trajectory replanning: only plan from the current state to the next target.
  if (fd_->avoid_collision_ || fd_->go_back_ || rdv_active_ || returning_ || completamento_) {
    // Il ritrovamento ha un bersaglio proprio, aggiornato a ogni pianificazione
    // con l'ultima posizione nota del compagno: inseguire un punto fisso
    // significherebbe volare dove si trovava e non dove si trova.
    // Durante il rientro la meta si RICALCOLA a ogni pianificazione: e' la
    // prossima tappa raggiungibile verso casa, non un punto fisso. Se la strada
    // per casa non esiste ancora, si punta al tratto piu' avanzato percorribile
    // e ci si riprova dal punto nuovo - cosi' il drone avanza comunque.
    if (returning_) {
      Eigen::Vector3d tappa;
      Eigen::Vector3d casa = fd_->start_pos_;
      casa.z() = return_altitude_;
      if (expl_manager_->findReturnWaypoint(fd_->start_pt_, casa, tappa)) {
        return_target_ = tappa;
      } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), clock_rdv_, 3000,
            "[Rientro] Drone %d: nessuna tappa percorribile verso casa", getId());
      }
    }
    const Eigen::Vector3d meta = returning_ ? return_target_
        : (completamento_ ? meta_foto_
        : (rdv_active_ ? rdv_target_ : expl_manager_->ed_->next_pos_));
    const double yaw_meta =
        (completamento_ && !returning_) ? yaw_foto_ : expl_manager_->ed_->next_yaw_;
    res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, meta, yaw_meta);
    fd_->avoid_collision_ = false;
  } else {  // Do full planning normally
    RCLCPP_INFO(node_->get_logger(), "\033[1;32mDrone %d =========================\033[0m",
        expl_manager_->ep_->drone_id_);
    // Sotto frontier_mutex_: planExploreMotion PERCORRE ed_->points_,
    // ed_->averages_ e ed_->frontiers_, che updateFrontierStruct RICOSTRUISCE
    // dagli altri callback. Una riallocazione di quei vettori sotto i piedi di
    // chi li itera spiega tutte le firme di crash viste finora - sempre in punti
    // diversi, perche' il malloc se ne accorge molto dopo il danno.
    // Il rilevatore di memoria ha escluso una scrittura fuori dai limiti (sotto
    // di esso, che rallenta tutto, il difetto sparisce): e' una corsa.
    // Serializzare qui e' sostenibile ORA: la sezione critica e' la
    // pianificazione, ~0.15 s da quando il grafo di connettivita' e' passato da
    // 2.4 s a 0.14. Con il costo precedente avrebbe affamato l'FSM, ed e'
    // esattamente l'errore commesso serializzando l'interfaccia di HGrid.
    std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
    res = expl_manager_->planExploreMotion(
        fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  }

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    // Piano arrivato in ritardo (dopo time_r): se lo stato iniziale e' letto
    // sulla traiettoria precedente, che il drone nel frattempo ha continuato a
    // seguire (non viene piu' troncata), la nuova parte comunque a time_r e il
    // server la prende gia' avviata, nel punto in cui il drone e' arrivato.
    // Farla partire "adesso" la riporterebbe indietro di ritardo x velocita'.
    // "Adesso" resta per i piani che partono dallo stato misurato.
    const bool in_ritardo = (node_->now() - time_r).seconds() > 0;
    // PIANO SCADUTO. Preso gia' avviato, un piano molto in ritardo puo' trovarsi
    // lontano da dove il drone e' arrivato seguendo la traiettoria vecchia, se
    // nel frattempo ha preso un'altra direzione: il riferimento salterebbe (un
    // salto di 0.9 m, con 1.7 s di ritardo, ha mandato il drone contro un
    // ostacolo nella campagna V22). Si confronta il punto in cui il piano nuovo
    // verrebbe preso con quello in cui si trova ora la traiettoria vecchia; oltre
    // 25 cm il piano si butta, la traiettoria vecchia torna al suo posto e si
    // ripianifica da li'. Se la vecchia e' gia' finita (drone fermo sul punto di
    // vista) i due punti coincidono e il piano passa.
    // Se lo stato di partenza era gia' la FINE della traiettoria vecchia (drone
    // fermo sul punto di vista), il piano nuovo comincia esattamente dove il
    // drone e' fermo: si parte adesso, senza confronto. Senza questa eccezione
    // un piano da fermo, "preso gia' avviato", risultava sempre spostato e
    // veniva scartato di continuo: 228 piani buttati e drone fermo il 62% del
    // tempo in una run da 844 s (V23).
    const bool da_fine_traiettoria = traj_prima_valida_ &&
        (time_r - traj_prima_.start_time_).seconds() >= traj_prima_.duration_ - 0.05;
    if (in_ritardo && da_fine_traiettoria) {
      info->start_time_ = node_->now();
    } else if (in_ritardo && partenza_da_traiettoria_ && traj_prima_valida_) {
      const double ritardo = (node_->now() - time_r).seconds();
      const double t_vecchia = std::clamp(
          (node_->now() - traj_prima_.start_time_).seconds(), 0.0, traj_prima_.duration_);
      const double t_nuova = std::clamp(ritardo, 0.0, info->duration_);
      const double scarto = (info->position_traj_.evaluateDeBoorT(t_nuova) -
                             traj_prima_.position_traj_.evaluateDeBoorT(t_vecchia)).norm();
      // Anche la VELOCITA' deve combaciare. Un piano che parte dalla traiettoria
      // in corso puo' frenare al massimo e invertire la marcia: preso con 0.7 s di
      // ritardo, la posizione e' ancora entro 25 cm ma il riferimento passa di
      // colpo da +1.1 a -0.2 m/s con -1.8 m/s2. Sul Jetson (17 ottobre 2026,
      // cubicle_office) il drone ha superato il riferimento di 1 m, e' tornato
      // indietro a 1.4 m/s e ha urtato 5 s dopo. Oltre 0.5 m/s di salto il piano si
      // scarta come quelli spostati.
      const double salto_vel = (info->velocity_traj_.evaluateDeBoorT(t_nuova) -
                                traj_prima_.velocity_traj_.evaluateDeBoorT(t_vecchia)).norm();
      if (scarto > 0.25 || salto_vel > 0.5) {
        RCLCPP_WARN(node_->get_logger(),
            "[Ripianificazione] piano arrivato con %.2f s di ritardo, a %.2f m e %.2f m/s dalla "
            "traiettoria in corso: scartato, si ripianifica da questa",
            ritardo, scarto, salto_vel);
        planner_manager_->local_data_ = traj_prima_;
        return PIANO_SCADUTO;
      }
      info->start_time_ = time_r;
    } else if (!partenza_da_traiettoria_) {
      // PIANO DALLO STATO MISURATO: parte dove il drone era quando e' stato
      // letto lo stato, con la sua velocita'. Farlo partire "adesso" (com'era)
      // metteva il riferimento dove il drone si trovava ritardo x velocita'
      // fa: 0.6-1.1 s di pianificazione in cubicle_office, 0.6-1.1 m DIETRO il
      // drone. PX4 lo riportava indietro, il server di traiettoria leggeva la
      // distanza come un ritardo e rallentava ancora il riferimento, il piano
      // seguente trovava il drone "1.4-2.8 m dietro" (in realta' davanti) e
      // ripartiva di nuovo dallo stato misurato: il dondolio della run
      // dell'utente del 5 ottobre (98-110 s, 166-178 s, 297-309 s).
      // L'origine dei tempi vera e' l'istante della misura. Come istante di
      // presa si sceglie, fra quello e adesso, il punto del piano nuovo piu'
      // vicino al drone ORA: se nel frattempo ha proseguito dritto e' circa
      // l'istante della misura, se ha frenato o girato e' prima; da fermo e'
      // l'inizio del piano.
      const rclcpp::Time ora = node_->now();
      const double trascorso =
          std::clamp((ora - istante_stato_misurato_).seconds(), 0.0, info->duration_);
      double t_presa = 0.0, d_presa = std::numeric_limits<double>::max();
      for (double t = 0.0; t <= trascorso + 1e-9; t += 0.02) {
        const double d = (info->position_traj_.evaluateDeBoorT(t) - fd_->odom_pos_).norm();
        if (d < d_presa) {
          d_presa = d;
          t_presa = t;
        }
      }
      // PIANO DALLO STATO MISURATO SCADUTO. Se durante la pianificazione il drone
      // ha proseguito sulla traiettoria vecchia in un'altra direzione, nessun punto
      // del piano nuovo gli e' vicino e il riferimento salterebbe dove il drone non
      // e': il ponte lo tronca a 1 m, il drone ci si getta tagliando gli angoli, e
      // il controllo di collisione fatto sul piano non copre quel tratto. Sul
      // Jetson (17 ottobre 2026, cubicle_office a 1.0 m/s) due urti su due run
      // cosi': grafo di connettivita' da 6.1 s, piano preso a 2.8 m dal drone,
      // urto 3 s dopo. Oltre 0.5 m (la soglia di riancoraggio) il piano si scarta
      // come gli altri piani scaduti; la traiettoria vecchia torna al suo posto e
      // si ripianifica dallo stato di adesso.
      if (d_presa > 0.5) {
        RCLCPP_WARN(node_->get_logger(),
            "[Ripianificazione] piano dallo stato misurato arrivato dopo %.2f s, a %.2f m dal "
            "drone: scartato, si ripianifica",
            trascorso, d_presa);
        planner_manager_->local_data_ = traj_prima_;
        return PIANO_SCADUTO;
      }
      info->start_time_ = ora - rclcpp::Duration::from_seconds(t_presa);
    } else {
      info->start_time_ = time_r;
    }

    // CONTROLLO DI COLLISIONE DEL PIANO NUOVO, con la stessa regola del
    // controllo periodico (safetyCallback) e dall'istante in cui il server lo
    // prendera'. Prima un piano veniva pubblicato comunque: se entrava nella
    // fascia di gonfiaggio il controllo periodico lo segnalava subito, si
    // ripianificava da esso e si ripubblicava un piano ancora in collisione,
    // fino a sette volte in 1.5 s - e intanto il drone lo seguiva verso la
    // parete (V27, cubicle_office: 2 ribaltamenti su 2 run, entrambi cosi').
    const double t_inizio = std::max(0.0, (node_->now() - info->start_time_).seconds());
    if (!planner_manager_->trajLiberaDa(t_inizio)) {
      ++piani_in_collisione_;
      const bool vecchia_sicura = traiettoria_pubblicata_ && !per_collisione_ && !frenata_ &&
          (node_->now() - traj_prima_.start_time_).seconds() < traj_prima_.duration_ - 0.05;
      if (vecchia_sicura) {
        // La traiettoria in corso e' ancora sicura: si tiene quella e si
        // riprova fra poco.
        RCLCPP_WARN(node_->get_logger(),
            "[PianoInCollisione] piano nuovo nella fascia di gonfiaggio: scartato, prosegue la "
            "traiettoria in corso (%d finora)",
            piani_in_collisione_);
        planner_manager_->local_data_ = traj_prima_;
        return PIANO_IN_COLLISIONE;
      }
      if (fd_->odom_vel_.norm() < 0.2 && expl_manager_->ed_->path_next_goal_.size() >= 2) {
        // Fermo: riserva sulla spezzata del percorso (vedi
        // FastPlannerManager::planPolilinea), primi 7 m come i piani normali.
        planner_manager_->planPolilinea(expl_manager_->ed_->path_next_goal_, 0.5, 1.0, 7.0);
        if (planner_manager_->trajLiberaDa(0.0)) {
          const double yaw_fine = completamento_ ? yaw_foto_ : expl_manager_->ed_->next_yaw_;
          planner_manager_->planYawExplore(fd_->start_yaw_, yaw_fine, true,
              expl_manager_->ep_->relax_time_);
          info->start_time_ = node_->now();
          RCLCPP_WARN(node_->get_logger(),
              "[PianoInCollisione] piano nuovo nella fascia di gonfiaggio con il drone fermo: "
              "percorso di riserva sulla spezzata, fermandosi ai vertici (%d finora)",
              piani_in_collisione_);
          frenata_ = false;
          goto pubblica;
        }
      }
      if (fd_->odom_vel_.norm() < 0.2) {
        // Fermo: nessuna urgenza, e' un fallimento come gli altri.
        RCLCPP_WARN(node_->get_logger(),
            "[PianoInCollisione] piano nuovo nella fascia di gonfiaggio con il drone fermo: "
            "fallito (%d finora)",
            piani_in_collisione_);
        if (traiettoria_pubblicata_) planner_manager_->local_data_ = traj_prima_;
        return FAIL;
      }
      // FRENATA: la traiettoria in corso non e' sicura e quella nuova
      // nemmeno. Si frena in linea retta dallo stato misurato, poi si
      // ripianifica da fermi (da fermi il piano segue il percorso della
      // ricerca, senza l'abbrivio che lo porta fuori).
      Eigen::Vector3d v = fd_->odom_vel_;
      const double vmax = planner_manager_->pp_.max_vel_;
      if (vmax > 0.0 && v.norm() > vmax) v *= vmax / v.norm();
      planner_manager_->planStop(fd_->odom_pos_, v, planner_manager_->pp_.max_acc_);
      planner_manager_->planYawExplore(Eigen::Vector3d(fd_->odom_yaw_, 0, 0), fd_->odom_yaw_,
          false, expl_manager_->ep_->relax_time_);
      info->start_time_ = node_->now();
      frenata_ = true;
      RCLCPP_WARN(node_->get_logger(),
          "[Frenata] traiettoria in corso e piano nuovo in collisione: frenata in linea retta da "
          "%.2f m/s (%d piani in collisione finora)",
          v.norm(), piani_in_collisione_);
    } else {
      frenata_ = false;
    }

  pubblica:
    c2x_msgs::msg::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = static_cast<builtin_interfaces::msg::Time>(info->start_time_);
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

    fd_->newest_traj_ = bspline;
  }
  rclcpp::Time t2 = node_->now();
  RCLCPP_INFO(node_->get_logger(), "Exploration planning time: %.3f s", (t2 - t1).seconds());

  return res;
}

void C2ExplorationFSM::visualize(int content) {
  if (content == 1) {
    drawFrontierVisualization();
  } else if (content == 2) {
    drawPlanningVisualization();
  }
}

void C2ExplorationFSM::drawFrontierVisualization() {
  // Runs on a detached thread (see the vis_thread launches in FSMCallback), so
  // it reads ExplorationData and HGrid concurrently with the subscription
  // callbacks that rewrite them - optMsgCallback and droneStateMsgCallback both
  // replace whole vectors in swarm_state_ and re-derive HGrid's centre tables.
  // Unsynchronised, that is a container being resized under an iterating
  // reader; exploration_node segfaulted inside execute_subscription with no
  // other explanation. Same lock the rest of that state already uses.
  std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
  auto ed_ptr = expl_manager_->ed_;

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
        visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4), "frontier", i,
        PlanningVisualization::PUBLISHER::FRONTIER);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i,
        PlanningVisualization::PUBLISHER::FRONTIER);
  }
  last_ftr_num = ed_ptr->frontiers_.size();

  // Draw top viewpoints
  visualization_->drawSpheres(ed_ptr->points_, 0.2, PlanningVisualization::Color::DeepGreen(),
      "points", 0, PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawLines(ed_ptr->views1_, ed_ptr->views2_, 0.02,
      PlanningVisualization::Color::Black(), "view_fov", 0,
      PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw next goal
  if (ed_ptr->next_goal_.allFinite() && std::isfinite(ed_ptr->next_yaw_)) {
    Eigen::Quaterniond next_q;
    next_q = Eigen::AngleAxisd(ed_ptr->next_yaw_, Eigen::Vector3d::UnitZ());
    visualization_->drawPose(ed_ptr->next_goal_, next_q, "next_goal", 0);
  }
}

void C2ExplorationFSM::drawPlanningVisualization() {
  constexpr double kExploredGridVisZ = 0.24;
  constexpr double kGridOutlineVisZ = 0.40;
  constexpr double kOtherHullVisZ = 0.58;
  constexpr double kEgoHullVisZ = 0.74;
  constexpr double kOtherCenterVisZ = 0.86;
  constexpr double kEgoCenterVisZ = 0.96;

  // Detached thread, same reasoning as drawFrontierVisualization above: this one
  // walks swarm_state_[i].center_positions_/center_hulls_ and HGrid's
  // connectivity graph while the MeetingOpt handlers are free to replace them.
  std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);

  const bool finished = (state_ == FINISH || state_ == IDLE);
  const double color_a = finished ? 0.15 : fp_->color_a_;
  const Vector4d drone_color(fp_->color_r_, fp_->color_g_, fp_->color_b_, color_a);

  auto& states = expl_manager_->ed_->swarm_state_;
  auto& ego_state = states[getId() - 1];
  std::unordered_set<int> ego_hull_grid_ids;

  // Draw assigned center convex hulls. Ego tasks stay highlighted, while other drones are muted
  // context rendered above the grid outlines.
  for (size_t di = 0; di < states.size(); ++di) {
    vector<Vector3d> hull_l1, hull_l2;
    const auto& drone_state = states[di];
    const bool is_ego_drone = (static_cast<int>(di) + 1 == getId());
    const double hull_z = is_ego_drone ? kEgoHullVisZ : kOtherHullVisZ;
    const double center_z = is_ego_drone ? kEgoCenterVisZ : kOtherCenterVisZ;
    for (int ci = 0; ci < static_cast<int>(drone_state.center_positions_.size()); ++ci) {
      vector<Vector3d> hull;
      if (ci < static_cast<int>(drone_state.center_hulls_.size())) {
        hull = drone_state.center_hulls_[ci];
      }
      if (hull.empty()) {
        expl_manager_->hgrid_->getCenterHullByPos(drone_state.center_positions_[ci], hull, 1.0);
      }
      if (hull.size() < 2) continue;
      for (size_t i = 0; i < hull.size(); ++i) {
        Vector3d p1 = hull[i];
        Vector3d p2 = hull[(i + 1) % hull.size()];
        p1.z() = hull_z;
        p2.z() = hull_z;
        hull_l1.push_back(p1);
        hull_l2.push_back(p2);
      }
      if (is_ego_drone) {
        const int gid = resolveCenterGridId(expl_manager_->hgrid_, drone_state, ci);
        if (gid >= 0) ego_hull_grid_ids.insert(gid);
      }
    }

    const Vector4d hull_color =
        is_ego_drone ? drone_color : Vector4d(0.28, 0.33, 0.40, finished ? 0.05 : 0.22);
    vector<Vector3d> center_positions = drone_state.center_positions_;
    for (auto& point : center_positions) point.z() = center_z;
    visualization_->drawSpheres(center_positions, is_ego_drone ? 0.24 : 0.18, hull_color,
        "center_hull_centers", static_cast<int>(di) + 1, PlanningVisualization::PUBLISHER::HGRID);
    visualization_->drawLines(hull_l1, hull_l2, is_ego_drone ? 0.10 : 0.06, hull_color,
        "center_hulls", static_cast<int>(di) + 1, PlanningVisualization::PUBLISHER::HGRID);
  }

  // Draw assigned grid outlines that are not already represented by ego hull tasks.
  vector<int> ego_grid_outline_ids;
  ego_grid_outline_ids.reserve(ego_state.grid_ids_.size());
  for (const int gid : ego_state.grid_ids_) {
    if (ego_hull_grid_ids.find(gid) == ego_hull_grid_ids.end()) {
      ego_grid_outline_ids.push_back(gid);
    }
  }

  vector<Vector3d> pts1, pts2;
  expl_manager_->hgrid_->getGridMarker(ego_grid_outline_ids, pts1, pts2);
  for (auto& point : pts1) point.z() = kGridOutlineVisZ;
  for (auto& point : pts2) point.z() = kGridOutlineVisZ;
  visualization_->drawLines(pts1, pts2, 0.07, drone_color, "partition", 1,
      PlanningVisualization::PUBLISHER::HGRID);

  // Draw explored grid outlines as semi-transparent history.
  vector<int> explored_ids;
  expl_manager_->hgrid_->getExploredGrids(explored_ids);
  vector<Vector3d> exp_pts1, exp_pts2;
  expl_manager_->hgrid_->getGridMarker(explored_ids, exp_pts1, exp_pts2);
  for (auto& point : exp_pts1) point.z() = kExploredGridVisZ;
  for (auto& point : exp_pts2) point.z() = kExploredGridVisZ;
  visualization_->drawLines(exp_pts1, exp_pts2, 0.05, Vector4d(0.35, 0.35, 0.35, 0.10),
      "partition_explored", 2, PlanningVisualization::PUBLISHER::HGRID);

  // Draw current grid tour and B-spline trajectory.
  auto info = &planner_manager_->local_data_;
  auto grid_tour = expl_manager_->ed_->region_tour_;
  visualization_->drawLines(
      grid_tour, 0.1, drone_color, "grid_tour", 0, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawBspline(
      info->position_traj_, 0.1, drone_color, false, 0.15, Vector4d(1, 1, 0, 1));

  // Clear stale next-viewpoint markers previously published on the hgrid topic.
  visualization_->drawSpheres(
      {}, 0.35, Vector4d(0, 0, 0, 1), "grid_tour", 1, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawLines({}, {}, 0.08, Vector4d(0, 0, 0, 1), "grid_tour", 2,
      PlanningVisualization::PUBLISHER::HGRID);

  // Draw hgrid connectivity graph.
  vector<Vector3d> cg_unknown_s, cg_unknown_e, cg_free_s, cg_free_e, cg_portal_s, cg_portal_e,
      cg_active_unknown_nodes, cg_active_free_nodes, cg_inactive_unknown_nodes,
      cg_inactive_free_nodes, cg_edge_cost_pos;
  vector<double> cg_edge_costs;
  expl_manager_->hgrid_->getConnectivityGraphVisualizationData(1, cg_unknown_s, cg_unknown_e,
      cg_free_s, cg_free_e, cg_portal_s, cg_portal_e, cg_active_unknown_nodes,
      cg_active_free_nodes, cg_inactive_unknown_nodes, cg_inactive_free_nodes, cg_edge_cost_pos,
      cg_edge_costs);
  visualization_->drawConnectivityGraph(cg_unknown_s, cg_unknown_e, cg_free_s, cg_free_e,
      cg_portal_s, cg_portal_e, cg_active_unknown_nodes, cg_active_free_nodes,
      cg_inactive_unknown_nodes, cg_inactive_free_nodes, cg_edge_cost_pos, cg_edge_costs,
      "hgrid_connectivity_graph_d" + std::to_string(getId()),
      PlanningVisualization::PUBLISHER::CONNECTIVITY_GRAPH);
}

void C2ExplorationFSM::clearVisMarker() {
  for (int i = 0; i < 10; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i,
        PlanningVisualization::PUBLISHER::FRONTIER);
  }
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0,
      PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0,
      PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawSpheres(
      {}, 0.35, Vector4d(0, 0, 0, 1), "grid_tour", 1, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawLines(
      {}, {}, 0.08, Vector4d(0, 0, 0, 1), "grid_tour", 2, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawConnectivityGraph({}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
      "hgrid_connectivity_graph_d" + std::to_string(getId()),
      PlanningVisualization::PUBLISHER::CONNECTIVITY_GRAPH);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);
}

void C2ExplorationFSM::frontierCallback() {
  if (state_ == WAIT_TRIGGER) {
    auto ed = expl_manager_->ed_;

    bool status;
    {
      std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
      expl_manager_->updateFrontierStruct(fd_->odom_pos_);

      vector<int> tmp_id1;
      vector<vector<int>> tmp_id2;
      // Optimize the global path with single-drone ATSP.
      status = expl_manager_->findGlobalTourOfGrid(
          { fd_->odom_pos_ }, { fd_->odom_vel_ }, tmp_id1, tmp_id2, true);
    }

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4), "frontier", i,
          PlanningVisualization::PUBLISHER::FRONTIER);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i,
          PlanningVisualization::PUBLISHER::FRONTIER);
    }
    if (status)
      visualize(2);
    else
      visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0,
          PlanningVisualization::PUBLISHER::VIEWPOINT);
  }
}

// Dichiara lo stato di prontezza finche' il drone attende il via.
//
// Le precondizioni sono pubblicate separatamente perche' quando un drone non
// parte serve sapere DOVE si e' fermato: senza odometria, senza mappa, o con la
// mappa ma senza niente da esplorare. Prima quella distinzione si poteva solo
// dedurre leggendo i log a posteriori.
void C2ExplorationFSM::publishReadiness() {
  if (!ready_pub_) return;
  // Una volta partiti la conferma non serve piu': si smette di calcolare le
  // frontiere fuori dal ciclo di pianificazione.
  if (state_ != INIT && state_ != WAIT_TRIGGER) return;

  c2x_msgs::msg::ExplorationReady msg;
  msg.drone_id = getId();
  msg.odom_ok = fd_->have_odom_;
  msg.stamp = node_->now().seconds();
  msg.num_frontiers = 0;
  msg.map_ok = false;

  if (!msg.odom_ok) {
    msg.detail = "nessuna odometria";
  } else {
    // Stessa chiamata che esegue il via: se restituisce frontiere, la mappa
    // esiste ed e' esplorabile. E' la verifica diretta della condizione che il
    // riscaldamento a tempo fisso si limitava ad approssimare.
    int num = 0;
    {
      std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
      num = expl_manager_->updateFrontierStruct(fd_->odom_pos_);
      frontiers_seen_ = num;
    }
    msg.num_frontiers = num;
    msg.map_ok = num > 0;
    msg.detail = num > 0 ? "pronto" : "mappa ancora senza frontiere";
  }

  msg.ready = msg.odom_ok && msg.map_ok;
  if (attendi_volo_pronto_ && !volo_pronto_) {
    msg.ready = false;
    if (msg.odom_ok && msg.map_ok) msg.detail = "decollo / giro iniziale in corso";
  }
  ready_pub_->publish(msg);
}

void C2ExplorationFSM::triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (state_ != WAIT_TRIGGER) return;

  fd_->expl_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  fd_->start_pos_ = fd_->odom_pos_;
  RCLCPP_WARN_STREAM(node_->get_logger(), "Start expl pos: " << fd_->start_pos_.transpose());

  // Il conteggio serve solo all'avviso qui sotto: se il lock e' occupato non lo si aspetta.
  // SCIAME (ottobre 2026): in attesa del via i droni continuano a calcolare frontiere e
  // ripartizione (frontier_timer_, MeetingOpt) tenendo frontier_mutex_, e con tre droni a
  // RTF 0.24 il via restava bloccato qui ~40 s di parete (~10 s simulati): il drone 1 partiva
  // da solo e gli altri dopo, a ogni run. La pianificazione in PLAN_TRAJ aggiorna comunque
  // le frontiere da se'.
  int num_frontiers = -1;
  {
    std::unique_lock<std::recursive_mutex> lock(frontier_mutex_, std::try_to_lock);
    if (lock.owns_lock()) num_frontiers = expl_manager_->updateFrontierStruct(fd_->odom_pos_);
  }
  trigger_time_ = node_->now();

  // Zero frontiere QUI non significa "niente da esplorare": significa che la
  // mappa non e' ancora stata costruita. E' esattamente il caso che il
  // riscaldamento fisso di 30 s nello script serviva a evitare, facendo stare
  // fermi i droni finche' la prima scansione non era accumulata - 30 s su ogni
  // run, misurati come un terzo dell'intera finestra di missione.
  // Tollerare l'istante iniziale qui costa molto meno: si va comunque a
  // pianificare, e se davvero non c'e' nulla da esplorare ci pensa il ramo IDLE,
  // che ha gia' i suoi tentativi a vuoto.
  if (num_frontiers == 0)
    RCLCPP_WARN(node_->get_logger(),
        "Drone %d: nessuna frontiera al via, la mappa non e' ancora pronta - si pianifica lo "
        "stesso invece di concludere",
        getId());
  transitState(PLAN_TRAJ, "triggerCallback");
}

// Physical-crash check: independent of state_ and of the default callback group, so it
// keeps running even while frontier_timer_'s grid-tour/ConnectivityGraph computation is
// hogging that group for 10+ seconds on a cluttered map (see input_cb_group_'s comment in
// init() - a real crash once went undetected for that entire stall because this check used
// to live in safetyCallback, in the default group). checkTrajCollision() in safetyCallback
// below only asks whether the *planned* B-spline still avoids inflated-occupied map cells -
// it has no idea whether the drone has actually made contact with something in Gazebo (thin
// geometry the map missed, real PX4 tracking overshoot, etc). A real crash tips the drone
// over; a wall/floor-standing crash easily exceeds any tilt reachable in normal flight, so
// use that as a direct physical signal instead of trying to infer it from the map. Checked
// from odom_orient_ (already parsed every odometryCallback) rather than a new subscription.
void C2ExplorationFSM::crashCheckCallback() {
  if (fd_->crashed_ || !fd_->have_odom_) return;
  const auto& q = fd_->odom_orient_;
  double roll = atan2(
      2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  double pitch = fabs(sinp) >= 1.0 ? copysign(M_PI / 2.0, sinp) : asin(sinp);
  // 50 gradi per 0.2 s, non piu' 60 per 0.5 s.
  //
  // Con il vecchio criterio un urto vero passava inosservato: nella run
  // BASE_GPS 2 il drone ha colpito la parete cw106, ha toccato terra e ha
  // oscillato fra 10 e 76 gradi per due secondi senza MAI restare sopra 60 per
  // mezzo secondo consecutivo - e la run e' stata contata come riuscita.
  // Con MPC_TILTMAX_AIR limitato nell'airframe 22000, in volo normale
  // l'inclinazione resta sotto i 30 gradi (misurata: 13-16 nelle run pulite),
  // quindi 50 gradi si raggiungono solo con un urto.
  const double tilt_limit = 50.0 * M_PI / 180.0;
  // Timer a 20 Hz: 4 tick = 0.2 s.
  const int sustained_ticks_required = 4;
  if (fabs(roll) > tilt_limit || fabs(pitch) > tilt_limit) {
    fd_->tilt_over_threshold_ticks_++;
  } else {
    fd_->tilt_over_threshold_ticks_ = 0;
  }
  if (fd_->tilt_over_threshold_ticks_ >= sustained_ticks_required) {
    fd_->crashed_ = true;
    RCLCPP_ERROR(node_->get_logger(),
        "CRASH DETECTED: roll=%.1f deg, pitch=%.1f deg (sustained %.1fs) - stopping "
        "exploration, drone is no longer upright",
        roll * 180.0 / M_PI, pitch * 180.0 / M_PI, sustained_ticks_required * 0.05);
    // Dopo un urto non si riprende la missione: si atterra. Un velivolo che ha
    // colpito qualcosa puo' avere eliche danneggiate, e proseguire significa
    // scommettere che non sia cosi'.
    returning_ = false;
    land_pub_->publish(std_msgs::msg::Empty());
    transitState(FINISH, "crashCheckCallback");
    finale_ = true;
  }
}

// Give up on frontiers this drone cannot actually clear.
//
// The FSM only leaves the plan/execute loop when updateFrontierStruct() reports
// zero frontiers. A frontier that survives every visit - unknown space sealed
// behind geometry, or beyond what the sensor can ever observe from a reachable
// viewpoint - therefore keeps the drone flying at it forever. That is exactly
// what happened in a 3-drone run: drone 3 completed 181 plan/execute cycles
// against a single frontier in grid 9, replanning on "traj fully executed"
// every time, while drones 1 and 2 had finished minutes earlier.
//
// Explored volume is the honest progress signal: it grows while the drone is
// learning anything at all, including during a long transit, and flatlines only
// when it genuinely is not. Once it has been flat for no_progress_timeout the
// drone goes IDLE, which ends in FINISH - and is still a state a later
// MeetingOpt round can pull it out of if a peer hands it real work.
void C2ExplorationFSM::progressCheckCallback() {
  if (fp_->no_progress_timeout_ <= 0.0) return;
  // Only meaningful while actually exploring.
  if (state_ != PLAN_TRAJ && state_ != PUB_TRAJ && state_ != EXEC_TRAJ) {
    fd_->last_explored_volume_ = -1.0;
    return;
  }

  const double volume = expl_manager_->sdf_map_->getExploredVolume();
  const auto now = node_->now();

  if (fd_->last_explored_volume_ < 0.0) {
    // First sample of this exploration stretch: open the window.
    fd_->last_explored_volume_ = volume;
    fd_->last_progress_time_ = now;
    return;
  }

  // Sliding window, deliberately not "reset the clock on any growth": with the
  // latter a single scrap of new volume postponed the verdict by another full
  // timeout, and a drone circling the same spot always finds a scrap. The
  // question asked here is the useful one - over the last no_progress_timeout
  // seconds, did this drone gain at least no_progress_min_volume of new space?
  const double elapsed = (now - fd_->last_progress_time_).seconds();
  if (elapsed < fp_->no_progress_timeout_) return;

  const double gained = volume - fd_->last_explored_volume_;
  if (gained >= fp_->no_progress_min_volume_) {
    // Productive window: slide it forward and keep exploring.
    fd_->last_explored_volume_ = volume;
    fd_->last_progress_time_ = now;
    return;
  }

  // RILEVATORE DI NON CONVERGENZA GLOBALE (solo diagnostica).
  //
  // Il guardiano qui sotto e' PER DRONE: scatta se questo velivolo non guadagna
  // volume. Ma esiste un guasto che nessuno vedeva - tutti e tre i droni che
  // pianificano e volano regolarmente, ognuno guadagnando abbastanza da non far
  // scattare il proprio guardiano, mentre nel complesso l'esplorazione non
  // avanza. La run AFF 1 ha bruciato cosi' 437 s con 168, 289 e 202 cicli di
  // pianificazione e nessun drone che concludesse, e il primo segnale e' stato
  // lo scadere del tetto di tempo.
  // Il numero di frontiere e' GLOBALE (viene dalla mappa condivisa), quindi
  // ogni drone puo' accorgersene da solo: se non cala per un tempo lungo mentre
  // si sta pianificando, lo sciame nel suo insieme e' fermo.
  {
    const int nf = frontiers_seen_;
    const auto now_conv = node_->now();
    if (nf != conv_last_frontiers_) {
      conv_last_frontiers_ = nf;
      conv_last_change_ = now_conv;
    } else if (conv_last_change_.nanoseconds() > 0 &&
               (now_conv - conv_last_change_).seconds() > 120.0) {
      rclcpp::Clock ck(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(node_->get_logger(), ck, 30000,
          "[NonConvergenza] il numero di frontiere e' fermo a %d da %.0fs mentre lo sciame "
          "pianifica: l'esplorazione nel suo complesso non sta avanzando",
          nf, (now_conv - conv_last_change_).seconds());
    }
  }

  // Non basta mandare il drone in IDLE: le frontiere che non riesce a eliminare
  // restano nella lista globale, quindi TUTTI i droni continuano a vederle e
  // nessuno puo' concludere. Vanno archiviate, altrimenti la run resta appesa
  // fino al tetto di tempo con gli altri droni fermi senza griglie assegnate.
  const int banned = expl_manager_->banHeldFrontiers();
  RCLCPP_WARN(node_->get_logger(),
      "Drone %d gained only %.2f m3 in %.0fs (total %.1f m3, threshold %.2f m3) - the frontiers "
      "it still holds cannot be cleared from anywhere it can reach, giving up (%d da archiviare)",
      getId(), gained, elapsed, volume, fp_->no_progress_min_volume_, banned);
  fd_->last_explored_volume_ = -1.0;
  fd_->last_check_frontier_time_ = now;
  fd_->static_state_ = true;
  // Niente troncamento: vedi la nota "FERMATE DI COLPO" su "Idle since no frontier".
  transitState(IDLE, "progressCheckCallback");
}

// Avoid collisions.
void C2ExplorationFSM::safetyCallback() {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Inter-drone conflicts, checked here at 20Hz rather than only when a peer
    // message happens to arrive. The message-driven check in swarmTrajCallback
    // only ever looks at the one peer that just published, and only while that
    // peer keeps publishing - so a conflict that develops against an already
    // known trajectory (ours changed, theirs didn't) went unnoticed until the
    // next broadcast. This runs in input_cb_group_, so it also keeps working
    // while the default group is busy replanning.
    if (expl_manager_->ep_->drone_num_ > 1) {
      const int conflict_id = planner_manager_->firstSwarmConflict();
      if (conflict_id > 0) {
        handleSwarmConflict(conflict_id, "safetyCallback");
        return;
      }
    }

    // Durante una frenata d'emergenza il controllo tace: la frenata va in
    // linea retta dallo stato del drone e puo' sfiorare la fascia di
    // gonfiaggio, e ripianificarla di continuo non servirebbe a nulla.
    if (frenata_) return;
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      RCLCPP_WARN(node_->get_logger(), "Replan: collision detected==================================");
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void C2ExplorationFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = node_->now();
  }
}

void C2ExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  if (finale_ && new_state != FINISH) {
    RCLCPP_WARN(node_->get_logger(),
        "[%s]: Drone %d resta in FINISH (stato finale), transizione a %s ignorata",
        pos_call.c_str(), getId(), fd_->state_str_[int(new_state)].c_str());
    state_ = FINISH;
    return;
  }
  int pre_s = int(state_);
  state_ = new_state;
  RCLCPP_INFO_STREAM(node_->get_logger(), "[" + pos_call + "]: Drone "
                  << getId()
                  << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}

int C2ExplorationFSM::resolveCenterGridId(
    const shared_ptr<HGrid>& hgrid, const DroneState& drone_state, const int center_idx) {
  if (center_idx >= 0 && center_idx < static_cast<int>(drone_state.center_grid_ids_.size()) &&
      drone_state.center_grid_ids_[center_idx] >= 0) {
    return drone_state.center_grid_ids_[center_idx];
  }
  if (center_idx >= 0 && center_idx < static_cast<int>(drone_state.center_positions_.size())) {
    int gid = -1;
    if (hgrid->getGridIdByCenterPos(drone_state.center_positions_[center_idx], gid, 1.0)) {
      return gid;
    }
  }
  return -1;
}

// ===================== Swarm / MeetingOpt layer =====================
// Decentralized task coordination from the paper: a periodic state broadcast,
// the contiguity-driven allocation solved by whichever connected drone has the
// lowest id ("temporary host"), dispatched with a versioned
// Proposal -> Commit -> Finalize handshake (Cancel + rollback on any reject or
// timeout), plus a trajectory exchange that feeds the optimizer's inter-drone
// avoidance term.

void C2ExplorationFSM::droneStateTimerCallback() {
  if (expl_manager_->ep_->drone_num_ < 2) return;

  c2x_msgs::msg::DroneState msg;
  msg.drone_id = getId();

  auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  // A missione finita (atterraggio in corso o avvenuto) la posizione e' quella MISURATA: il
  // ponte fa scendere il drone senza traiettoria, e l'ultima traiettoria lo dava ancora in
  // quota sopra casa. I compagni lo credevano in volo nel corridoio delle partenze e restavano
  // in stallo accanto a un drone gia' appoggiato a terra (office_small, ottobre 2026).
  if (fd_->static_state_ || finale_ || state_ == FINISH) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  } else {
    // local_data_ is rewritten in place by the planner on the default callback
    // group; NonUniformBspline::setUniformBspline resizes its control-point
    // matrix, so an unlocked evaluateDeBoorT() here could read mid-resize.
    std::lock_guard<std::mutex> lock(planner_manager_->local_data_mutex_);
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (node_->now() - info->start_time_).seconds();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }
  state.stamp_ = node_->now().seconds();
  state.recv_stamp_ = state.stamp_;
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.yaw = state.yaw_;
  for (auto& id : state.grid_ids_) {
    msg.grid_ids.push_back(id);
  }
  for (const auto& c : state.center_positions_) {
    geometry_msgs::msg::Point p;
    p.x = c[0];
    p.y = c[1];
    p.z = c[2];
    msg.center_points.push_back(p);
  }
  for (const int cid : state.center_ids_) {
    msg.center_ids.push_back(cid);
  }
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;
  // Fase della missione e meta di visita (vedi pari_visita_ nell'header).
  msg.fase = static_cast<uint8_t>(faseMissione());
  if (completamento_ && zona_di_visita_) {
    msg.ha_meta_visita = true;
    msg.meta_visita.x = meta_foto_.x();
    msg.meta_visita.y = meta_foto_.y();
    msg.meta_visita.z = meta_foto_.z();
  }

  drone_state_pub_->publish(msg);
}

int C2ExplorationFSM::faseMissione() const {
  if (finale_ || returning_) return 2;
  if (completamento_) return 1;
  return 0;
}

// Receive other drones' states.
void C2ExplorationFSM::droneStateMsgCallback(const c2x_msgs::msg::DroneState::SharedPtr msg) {
  // Update other drones' states
  if (msg->drone_id == getId()) return;
  // Bounds check on an id that came off the wire: swarm_state_ is sized to
  // exploration.drone_num, and a stale peer from a differently-sized run would
  // otherwise index out of range.
  if (msg->drone_id < 1 ||
      msg->drone_id > static_cast<int>(expl_manager_->ed_->swarm_state_.size())) {
    rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[Swarm] Ignore drone state with out-of-range id=%d (drone_num=%zu)", msg->drone_id,
        expl_manager_->ed_->swarm_state_.size());
    return;
  }

  auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;  // Avoid unordered msg

  const vector<Vector3d> old_center_positions = drone_state.center_positions_;
  const vector<int> old_center_ids = drone_state.center_ids_;
  const vector<int> old_center_grid_ids = drone_state.center_grid_ids_;
  const vector<vector<Vector3d>> old_center_hulls = drone_state.center_hulls_;

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;
  drone_state.grid_ids_.clear();
  for (size_t i = 0; i < msg->grid_ids.size(); ++i) {
    drone_state.grid_ids_.push_back(msg->grid_ids[i]);
  }
  drone_state.center_positions_.clear();
  for (const auto& p : msg->center_points) {
    drone_state.center_positions_.emplace_back(p.x, p.y, p.z);
  }
  {
    // centersTo*/preserveCenterHulls read HGrid's last-cost-matrix tables, which
    // findGlobalTourOfGrid rewrites from the default callback group.
    std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
    centersToCenterGridIds(
        expl_manager_->hgrid_, drone_state.center_positions_, drone_state.center_grid_ids_);
    drone_state.center_ids_.clear();
    for (const auto& cid : msg->center_ids) {
      drone_state.center_ids_.push_back(cid);
    }
    if (drone_state.center_ids_.empty() && !drone_state.center_positions_.empty()) {
      centersToCenterIds(
          expl_manager_->hgrid_, drone_state.center_positions_, drone_state.center_ids_, true);
    }
    preserveCenterHulls(expl_manager_->hgrid_, old_center_positions, old_center_ids,
        old_center_grid_ids, old_center_hulls, drone_state.center_positions_,
        drone_state.center_ids_, drone_state.center_grid_ids_, drone_state.center_hulls_);
  }

  drone_state.stamp_ = msg->stamp;
  drone_state.recv_stamp_ = node_->now().seconds();
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;

}

// Start allocation optimization.
void C2ExplorationFSM::optTimerCallback() {
  if (expl_manager_->ep_->drone_num_ < 2) return;
  // Con la spartizione a strisce ogni drone sa gia' quali celle sono sue: nessuna
  // negoziazione (riassegnerebbe celle fuori striscia).
  {
    std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
    if (expl_manager_->spartizioneAStrisce()) return;
  }
  rclcpp::Clock clock(RCL_ROS_TIME);
  if (state_ == INIT) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: state=INIT");
    return;
  }
  auto ed = expl_manager_->ed_;
  if (state_ == FINISH) {
    if (ed->wait_opt_commit_ack_ || ed->wait_opt_response_) {
      clearHostOptSession(*ed);
    }
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: state=FINISH");
    return;
  }

  comm_graph_->getNeighborIds(expl_manager_->ed_->neighbor_ids_);

  // Select nearby drone not interacting with recently
  auto& states = expl_manager_->ed_->swarm_state_;
  auto& ego_state = states[getId() - 1];
  auto& neighbor = expl_manager_->ed_->neighbor_ids_;
  auto tn = node_->now().seconds();
  optDebugWarn(fp_, "[Meeting Opt Debug] optTimer enter: drone_id=" + std::to_string(getId()) +
                        " state=" + std::to_string(int(state_)) +
                        " neighbors=" + idsToStr(neighbor) +
                        " wait_opt_response=" + std::to_string(int(ed->wait_opt_response_)));
  if (ed->wait_opt_commit_ack_) {
    handleHostWaitSession(*ed, fp_, opt_pub_, getId(), tn, true);
    return;
  }

  if (ed->wait_opt_response_) {
    handleHostWaitSession(*ed, fp_, opt_pub_, getId(), tn, false);
    return;
  }

  // In terminal/idle states, skip meeting-opt probing when neither self nor neighbors
  // hold any assigned grids/centers.
  if (state_ == IDLE || state_ == FINISH) {
    auto hasAssignedWork = [](const DroneState& s) {
      return !s.grid_ids_.empty() || !s.center_positions_.empty() || !s.center_ids_.empty();
    };
    const bool self_has_work = hasAssignedWork(ego_state);
    bool neighbor_has_work = false;
    for (const int id : neighbor) {
      if (id <= 0 || id > static_cast<int>(states.size()) || id == getId()) continue;
      if (hasAssignedWork(states[id - 1])) {
        neighbor_has_work = true;
        break;
      }
    }
    if (!self_has_work && !neighbor_has_work) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: idle/finish no assigned work");
      return;
    }
  }

  // Avoid frequent attempt
  if (tn - ego_state.recent_attempt_time_ < fp_->attempt_interval_) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: attempt interval gate, dt=" +
                          std::to_string(tn - ego_state.recent_attempt_time_) + " < " +
                          std::to_string(fp_->attempt_interval_));
    return;
  }
  if (tn - ego_state.recent_interact_time_ < fp_->meeting_opt_interval_) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: interact interval gate, dt=" +
                          std::to_string(tn - ego_state.recent_interact_time_) + " < " +
                          std::to_string(fp_->meeting_opt_interval_));
    return;
  }

  // Choose host from a globally consistent leader set (neighbor view + self),
  // independent of participant freshness filtering.
  vector<int> leader_ids = neighbor;
  pushUniqueId(leader_ids, getId());
  std::sort(leader_ids.begin(), leader_ids.end());
  leader_ids.erase(std::unique(leader_ids.begin(), leader_ids.end()), leader_ids.end());
  if (leader_ids.size() < 2) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: drone_id=" + std::to_string(getId()) +
                          " leader_ids<2 final_leader_ids=" + idsToStr(leader_ids));
    return;
  }
  optDebugWarn(fp_, "[Meeting Opt Debug] optTimer leader_ids decided: drone_id=" +
                        std::to_string(getId()) + " final_leader_ids=" + idsToStr(leader_ids));
  const int preferred_host_id = leader_ids.front();
  // Select host with deterministic id order.
  // Primary failover condition: preferred host stale.
  // Secondary failover condition: preferred host interval-gated for too long.
  const double failover_timeout =
      std::max(3.0 * fp_->state_fresh_timeout_, fp_->meeting_opt_interval_);
  const double gate_failover_timeout =
      std::max(std::max(2.0 * fp_->meeting_opt_interval_, 4.0 * fp_->attempt_interval_), 0.5);
  auto hostFreshEnough = [&](const int id) {
    if (id == getId()) return true;
    if (fp_->state_fresh_timeout_ <= 1e-6) return true;
    return (tn - states[id - 1].recv_stamp_) <= failover_timeout;
  };
  auto hostCanStartNow = [&](const int id) {
    const auto& s = states[id - 1];
    return (tn - s.recent_attempt_time_ >= fp_->attempt_interval_) &&
           (tn - s.recent_interact_time_ >= fp_->meeting_opt_interval_);
  };
  int host_id = preferred_host_id;
  if (!hostFreshEnough(preferred_host_id)) {
    ed->opt_preferred_gate_host_id_ = -1;
    ed->opt_preferred_gate_since_ = 0.0;
    host_id = -1;
    for (const int id : leader_ids) {
      if (hostFreshEnough(id) && hostCanStartNow(id)) {
        host_id = id;
        break;
      }
    }
    if (host_id < 0) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: no eligible host candidate");
      return;
    }
    if (host_id != preferred_host_id) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
            "[Meeting Opt] Host failover from d" << preferred_host_id << " to d" << host_id
                                                 << " due to stale state");
      }
    }
  } else if (!hostCanStartNow(preferred_host_id)) {
    if (ed->opt_preferred_gate_host_id_ != preferred_host_id) {
      ed->opt_preferred_gate_host_id_ = preferred_host_id;
      ed->opt_preferred_gate_since_ = tn;
    }
    const double gate_duration = std::max(0.0, tn - ed->opt_preferred_gate_since_);
    if (gate_duration + 1e-6 < gate_failover_timeout) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: preferred host gated, keep host=d" +
                            std::to_string(preferred_host_id) +
                            " gated_for=" + std::to_string(gate_duration) +
                            " < failover=" + std::to_string(gate_failover_timeout));
      return;
    }

    host_id = -1;
    for (const int id : leader_ids) {
      if (id == preferred_host_id) continue;
      if (hostFreshEnough(id) && hostCanStartNow(id)) {
        host_id = id;
        break;
      }
    }
    if (host_id < 0) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: preferred host gated for "
                        "too long but no alternative host");
      return;
    }
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[Meeting Opt] Host soft-failover from d" << preferred_host_id << " to d" << host_id
                                                    << " due to sustained gate dt=" << gate_duration);
    }
  } else {
    ed->opt_preferred_gate_host_id_ = -1;
    ed->opt_preferred_gate_since_ = 0.0;
  }
  if (getId() != host_id) {
    optDebugWarn(fp_,
        "[Meeting Opt Debug] optTimer return: not host, selected host=" + std::to_string(host_id));
    return;
  }

  vector<int> drone_ids;
  drone_ids.reserve(neighbor.size());
  for (auto id : neighbor) {
    if (id == getId()) {
      drone_ids.push_back(id);
      continue;
    }
    if (fp_->state_fresh_timeout_ > 1e-6 &&
        tn - states[id - 1].recv_stamp_ > fp_->state_fresh_timeout_) {
      optDebugWarn(fp_, "[Meeting Opt Debug] skip participant " + std::to_string(id) +
                            ": stale recv dt=" + std::to_string(tn - states[id - 1].recv_stamp_));
      continue;
    }
    if (tn - states[id - 1].recent_interact_time_ < fp_->meeting_opt_interval_) {
      optDebugWarn(fp_,
          "[Meeting Opt Debug] skip participant " + std::to_string(id) +
              ": recent interact dt=" + std::to_string(tn - states[id - 1].recent_interact_time_));
      continue;
    }
    if (id - 1 < static_cast<int>(ed->opt_peer_backoff_until_.size())) {
      const double backoff_until = ed->opt_peer_backoff_until_[id - 1];
      if (backoff_until > tn + 1e-6) {
        const int streak = (id - 1 < static_cast<int>(ed->opt_peer_reject_streak_.size())) ?
                               ed->opt_peer_reject_streak_[id - 1] :
                               0;
        optDebugWarn(fp_, "[Meeting Opt Debug] skip participant " + std::to_string(id) +
                              ": reject-backoff remain=" + std::to_string(backoff_until - tn) +
                              " streak=" + std::to_string(streak));
        continue;
      }
      if (id - 1 < static_cast<int>(ed->opt_peer_reject_streak_.size()) &&
          ed->opt_peer_reject_streak_[id - 1] > 0) {
        clearPeerRejectBackoff(*ed, id);
      }
    }
    drone_ids.push_back(id);
  }
  pushUniqueId(drone_ids, getId());

  if (drone_ids.size() < 2) {
    optDebugWarn(fp_,
        "[Meeting Opt Debug] optTimer return: participant_ids<2, ids=" + idsToStr(drone_ids));
    return;
  }

  // Everything below reads/writes HGrid's task tables and runs the ACVRP solve,
  // which findGlobalTourOfGrid (default callback group) touches concurrently.
  std::lock_guard<std::recursive_mutex> alloc_lock(frontier_mutex_);

  // Do global optimization with all participant drones
  vector<int> opt_ids;
  unordered_map<int, char> opt_ids_map;
  vector<int> state_task_ids;
  for (auto id : drone_ids) {
    collectStateTaskGridIds(states[id - 1], expl_manager_->hgrid_, state_task_ids);
    for (const auto gid : state_task_ids) {
      opt_ids_map[gid] = 1;
    }
  }
  for (auto pair : opt_ids_map) opt_ids.push_back(pair.first);

  if (fp_ && fp_->debug_opt_output_) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[MeetingOpt] host="
                                                << getId() << " participants=" << drone_ids.size()
                                                << " opt_grids=" << opt_ids.size());
  }
  if (opt_ids.empty()) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: opt_ids empty");
    return;
  }

  vector<Eigen::Vector3d> positions;
  vector<Eigen::Vector3d> velocities;
  vector<DroneState> participant_states;
  vector<vector<int>> first_ids_list, second_ids_list;
  positions.reserve(drone_ids.size());
  velocities.reserve(drone_ids.size());
  participant_states.reserve(drone_ids.size());
  first_ids_list.resize(drone_ids.size());
  second_ids_list.resize(drone_ids.size());

  for (size_t i = 0; i < drone_ids.size(); ++i) {
    const auto& state = states[drone_ids[i] - 1];
    positions.push_back(state.pos_);
    velocities.push_back(Eigen::Vector3d(0, 0, 0));
    participant_states.push_back(state);
    if (state_ != WAIT_TRIGGER) {
      vector<int> ids_only;
      collectStateTaskGridIds(state, expl_manager_->hgrid_, ids_only);
      expl_manager_->hgrid_->getConsistentGrid(
          ids_only, ids_only, first_ids_list[i], second_ids_list[i]);
    }
  }

  // Non-participant non-hull centers are protected by connectivity graph node id.
  // Split hull tasks do not have stable node-id identity, so they are protected by hull geometry.
  unordered_set<int> participant_set(drone_ids.begin(), drone_ids.end());
  unordered_set<int> blocked_node_id_set;
  vector<int> blocked_center_node_ids;
  vector<vector<Vector3d>> blocked_center_hulls;
  for (int did = 1; did <= static_cast<int>(states.size()); ++did) {
    if (participant_set.find(did) != participant_set.end()) continue;
    const auto& st = states[did - 1];
    for (int i = 0; i < static_cast<int>(st.center_ids_.size()); ++i) {
      const bool has_hull =
          (i < static_cast<int>(st.center_hulls_.size()) && !st.center_hulls_[i].empty());
      if (has_hull) continue;
      const int cid = st.center_ids_[i];
      if (cid < 0) continue;
      if (blocked_node_id_set.insert(cid).second) {
        blocked_center_node_ids.push_back(cid);
      }
    }
    for (const auto& hull : st.center_hulls_) {
      if (!hull.empty()) blocked_center_hulls.push_back(hull);
    }
  }

  const vector<DroneState>* ownership_states =
      (state_ == WAIT_TRIGGER) ? nullptr : &participant_states;
  if (state_ == WAIT_TRIGGER) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
          "[MeetingOpt] Skip participant-task ownership filter during WAIT_TRIGGER");
    }
  }
  C2ExplorationManager::AllocationRequest allocation_request;
  allocation_request.drone_positions = positions;
  allocation_request.drone_velocities = velocities;
  allocation_request.drone_ids = drone_ids;
  allocation_request.grid_ids = opt_ids;
  allocation_request.drone_states = ownership_states;
  allocation_request.blocked_center_node_ids = blocked_center_node_ids;
  allocation_request.blocked_center_hulls = blocked_center_hulls;

  C2ExplorationManager::AllocationResult allocation_result;
  expl_manager_->allocateTasks(allocation_request, allocation_result);
  vector<vector<Vector3d>>& alloc_centers = allocation_result.centers;
  vector<vector<vector<Vector3d>>>& alloc_center_hulls = allocation_result.center_hulls;
  if (alloc_centers.size() != drone_ids.size()) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: alloc_centers size mismatch, alloc=" +
                          std::to_string(alloc_centers.size()) +
                          " participants=" + std::to_string(drone_ids.size()));
    return;
  }
  if (alloc_center_hulls.size() != drone_ids.size()) {
    alloc_center_hulls.resize(drone_ids.size());
  }
  for (size_t i = 0; i < drone_ids.size(); ++i) {
    if (alloc_center_hulls[i].size() < alloc_centers[i].size()) {
      vector<vector<Vector3d>> recovered_hulls;
      centersToCenterHulls(expl_manager_->hgrid_, alloc_centers[i], recovered_hulls);
      alloc_center_hulls[i].resize(alloc_centers[i].size());
      for (int j = 0; j < static_cast<int>(alloc_centers[i].size()); ++j) {
        if (!alloc_center_hulls[i][j].empty()) continue;
        if (j < static_cast<int>(recovered_hulls.size())) {
          alloc_center_hulls[i][j] = recovered_hulls[j];
        }
      }
    }
  }

  vector<vector<int>> alloc_ids;
  alloc_ids.resize(drone_ids.size());
  for (size_t i = 0; i < drone_ids.size(); ++i) {
    centersToGridIds(expl_manager_->hgrid_, alloc_centers[i], alloc_ids[i], true);
  }

  vector<int> filtered_drone_ids;
  vector<vector<Vector3d>> filtered_alloc_centers;
  vector<vector<vector<Vector3d>>> filtered_alloc_center_hulls;
  vector<vector<int>> filtered_alloc_ids;
  vector<int> pruned_drone_ids;
  filtered_drone_ids.reserve(drone_ids.size());
  filtered_alloc_centers.reserve(drone_ids.size());
  filtered_alloc_center_hulls.reserve(drone_ids.size());
  filtered_alloc_ids.reserve(drone_ids.size());

  for (size_t i = 0; i < drone_ids.size(); ++i) {
    const int pid = drone_ids[i];
    const auto& st = states[pid - 1];
    const bool has_new_alloc = !alloc_ids[i].empty() || !alloc_centers[i].empty();
    const bool has_existing_alloc = !st.grid_ids_.empty() || !st.center_positions_.empty();
    const bool keep = (pid == getId()) || has_new_alloc || has_existing_alloc;
    if (!keep) {
      pruned_drone_ids.push_back(pid);
      continue;
    }
    filtered_drone_ids.push_back(pid);
    filtered_alloc_centers.push_back(alloc_centers[i]);
    filtered_alloc_center_hulls.push_back(alloc_center_hulls[i]);
    filtered_alloc_ids.push_back(alloc_ids[i]);
  }

  if (!pruned_drone_ids.empty() && fp_ && fp_->debug_opt_output_) {
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "[MeetingOpt] pruned empty participants count=" << pruned_drone_ids.size());
  }
  drone_ids.swap(filtered_drone_ids);
  alloc_centers.swap(filtered_alloc_centers);
  alloc_center_hulls.swap(filtered_alloc_center_hulls);
  alloc_ids.swap(filtered_alloc_ids);

  if (drone_ids.size() < 2) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: filtered participant_ids<2, ids=" +
                          idsToStr(drone_ids));
    return;
  }

  for (size_t i = 0; i < drone_ids.size(); ++i) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[MeetingOpt] Alloc to drone "
                                                  << drone_ids[i]
                                                  << " centers=" << alloc_centers[i].size()
                                                  << " grids=" << alloc_ids[i].size());
    }
  }

  ed->opt_participant_ids_ = drone_ids;
  ed->opt_alloc_centers_ = alloc_centers;
  ed->opt_alloc_center_hulls_ = alloc_center_hulls;
  ed->opt_alloc_ids_ = alloc_ids;
  ed->opt_wait_ids_.clear();
  ed->opt_ack_ids_.clear();
  ed->opt_commit_ack_ids_.clear();
  forEachRemoteParticipant(
      drone_ids, getId(), [&](size_t /*idx*/, int id) { ed->opt_wait_ids_.push_back(id); });
  ed->meeting_opt_stamp_ = tn;
  ed->opt_wait_start_time_ = tn;
  ed->opt_last_send_time_ = tn;
  ed->opt_retry_count_ = 0;
  ed->wait_opt_response_ = true;
  ed->wait_opt_commit_ack_ = false;
  ed->opt_commit_wait_start_time_ = 0.0;
  ed->opt_commit_last_send_time_ = 0.0;
  ed->opt_commit_retry_count_ = 0;
  ego_state.recent_attempt_time_ = tn;

  if (fp_ && fp_->debug_opt_output_) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "[Meeting Opt] Host "
                                                << getId() << " sending proposal to "
                                                << ed->opt_wait_ids_.size() << " peers");
  }

  size_t host_idx = 0;
  if (!findParticipantIndex(drone_ids, getId(), host_idx)) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
          "[Meeting Opt] Host " << getId() << " cannot find self in participant_ids, abort");
    }
    optDebugWarn(fp_, "[Meeting Opt Debug] optTimer return: host not found in participant_ids");
    return;
  }
  forEachRemoteParticipant(drone_ids, getId(), [&](size_t i, int to_id) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[Meeting Opt] Host "
                                                  << getId() << " send proposal to " << to_id
                                                  << " ego=" << alloc_ids[host_idx].size()
                                                  << " peer=" << alloc_ids[i].size());
    }
    sendMeetingOptAdaptive(opt_pub_, fp_, getId(), to_id, tn, kMeetingOptProposal,
        alloc_ids[host_idx], alloc_ids[i], alloc_centers[host_idx], alloc_centers[i],
        alloc_center_hulls[host_idx], alloc_center_hulls[i]);
  });
}

// Handle pairwise optimization requests from other drones.
void C2ExplorationFSM::optMsgCallback(const c2x_msgs::msg::MeetingOpt::SharedPtr msg) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) {
    optDebugTrace(fp_, "[Meeting Opt Debug] optMsg return: not for me/self msg from=" +
                           std::to_string(msg->from_drone_id) +
                           " to=" + std::to_string(msg->to_drone_id));
    return;
  }
  if (state_ == FINISH) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: state=FINISH");
    return;
  }
  auto ed = expl_manager_->ed_;
  if (msg->from_drone_id < 1 ||
      msg->from_drone_id > static_cast<int>(ed->swarm_state_.size())) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[Meeting Opt] Ignore opt from out-of-range id=%d (drone_num=%zu)", msg->from_drone_id,
        ed->swarm_state_.size());
    return;
  }

  const bool is_commit = (msg->phase == kMeetingOptCommit);
  const bool is_cancel = (msg->phase == kMeetingOptCancel);
  const bool is_finalize = (msg->phase == kMeetingOptFinalize);
  const bool is_proposal = (msg->phase == kMeetingOptProposal);
  if (!is_proposal && !is_commit && !is_cancel && !is_finalize) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[Meeting Opt] Drone " << getId() << " drop opt from " << msg->from_drone_id
                                 << ": unknown phase=" << static_cast<int>(msg->phase));
    }
    return;
  }
  const int from_idx = msg->from_drone_id - 1;
  constexpr double kStampEps = 1e-4;
  auto& state1 = ed->swarm_state_[msg->from_drone_id - 1];  // Requesting drone.
  auto& state2 = ed->swarm_state_[getId() - 1];             // Local drone.

  if (is_proposal) {
    // Last proposal stamp processed from the peer.
    const double last_stamp = ed->meeting_opt_stamps_[from_idx];
    if (msg->stamp < last_stamp - kStampEps) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
            "[Meeting Opt] Drone " << getId() << " drop opt from " << msg->from_drone_id
                                   << ": old stamp=" << msg->stamp);
      }
      optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: proposal too old, stamp=" +
                            std::to_string(msg->stamp) + " last=" + std::to_string(last_stamp));
      return;
    }
    if (msg->stamp > last_stamp + kStampEps) {
      ed->meeting_opt_stamps_[from_idx] = msg->stamp;
      optDebugWarn(fp_, "[Meeting Opt Debug] proposal new stamp update: from=" +
                            std::to_string(msg->from_drone_id) +
                            " stamp=" + std::to_string(msg->stamp));
    } else {
      optDebugWarn(fp_, "[Meeting Opt Debug] proposal same-stamp retransmission: from=" +
                            std::to_string(msg->from_drone_id) +
                            " stamp=" + std::to_string(msg->stamp));
    }
  }

  if (fp_ && fp_->debug_opt_output_) {
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "[Meeting Opt] Drone " << getId() << " recv opt from " << msg->from_drone_id
                               << " stamp=" << msg->stamp
                               << " phase=" << static_cast<int>(msg->phase)
                               << " ego_ids=" << msg->ego_ids.size()
                               << " other_ids=" << msg->other_ids.size());
  }

  if (is_finalize) {
    if (hasAppliedUnconfirmedTx(*ed, msg->from_drone_id, msg->stamp)) {
      clearAppliedUnconfirmedTx(*ed);
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[Meeting Opt] Drone "
                                                    << getId() << " finalize tx from "
                                                    << msg->from_drone_id
                                                    << " stamp=" << msg->stamp);
      }
    }
    return;
  }

  if (is_proposal) {
    const double now = node_->now().seconds();
    if (ed->wait_opt_response_ || ed->wait_opt_commit_ack_) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "[Meeting Opt] Drone " << getId() << " reject from " << msg->from_drone_id
                                   << ": local host session busy wait_proposal="
                                   << ed->wait_opt_response_
                                   << " wait_commit=" << ed->wait_opt_commit_ack_);
      }
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
      return;
    }
    if (ed->opt_pending_host_id_ > 0 && fp_->opt_wait_timeout_ > 1e-6 &&
        now - ed->opt_pending_recv_time_ > fp_->opt_wait_timeout_) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
            "[Meeting Opt] Drone " << getId()
                                   << " clear stale pending host=" << ed->opt_pending_host_id_
                                   << " dt=" << (now - ed->opt_pending_recv_time_));
      }
      clearOptPendingLock(*ed);
    }

    if (ed->opt_commit_apply_pending_ && fp_->opt_wait_timeout_ > 1e-6 &&
        now - ed->opt_commit_apply_time_ > 2.0 * fp_->opt_wait_timeout_) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
            "[Meeting Opt] Drone " << getId() << " clear stale unconfirmed commit tx: host="
                                   << ed->opt_commit_apply_host_id_
                                   << " stamp=" << ed->opt_commit_apply_stamp_);
      }
      clearAppliedUnconfirmedTx(*ed);
    }

    // If the same host starts a newer negotiation round, old unconfirmed commit is obsolete.
    if (ed->opt_commit_apply_pending_ && msg->from_drone_id == ed->opt_commit_apply_host_id_ &&
        msg->stamp > ed->opt_commit_apply_stamp_ + kStampEps) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "[Meeting Opt] Drone " << getId() << " clear obsolete unconfirmed commit tx from host "
                                   << ed->opt_commit_apply_host_id_
                                   << ": old_stamp=" << ed->opt_commit_apply_stamp_
                                   << " new_stamp=" << msg->stamp);
      }
      clearAppliedUnconfirmedTx(*ed);
    }

    if (ed->opt_commit_apply_pending_) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "[Meeting Opt] Drone " << getId() << " reject proposal from " << msg->from_drone_id
                                   << ": unconfirmed commit tx host="
                                   << ed->opt_commit_apply_host_id_
                                   << " stamp=" << ed->opt_commit_apply_stamp_);
      }
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
      return;
    }

    // Idempotent retransmission of already accepted proposal from the same host.
    if (hasOptPendingFromHost(*ed, msg->from_drone_id, msg->stamp)) {
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptAccept);
      optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: proposal retransmission acked");
      return;
    }

    // Single pending-host lock on receiver: reject other hosts until current pending session ends.
    if (ed->opt_pending_host_id_ > 0 && ed->opt_pending_host_id_ != msg->from_drone_id) {
      const double cooldown = registerPeerRejectBackoff(*ed, fp_, msg->from_drone_id, now);
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "[Meeting Opt] Drone " << getId() << " reject from " << msg->from_drone_id
                                   << ": pending host=" << ed->opt_pending_host_id_
                                   << " stamp=" << ed->opt_pending_stamp_
                                   << " cooldown=" << cooldown);
      }
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
      return;
    }

    // This drone just initiated a pairing attempt too.
    const double local_attempt_gap = node_->now().seconds() - state2.recent_attempt_time_;
    if (local_attempt_gap < fp_->attempt_interval_) {
      // Just made another meeting opt attempt, should reject this attempt to avoid frequent changes
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN(node_->get_logger(),
            "[Meeting Opt] Drone %d reject from %d: recent attempt %.3f interval %.3f", getId(),
            msg->from_drone_id, local_attempt_gap, fp_->attempt_interval_);
      }
      sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
          msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
      return;
    } else {
      int invalid_ego_centers = 0;
      int invalid_other_centers = 0;
      bool centers_resolvable;
      {
        std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
        centers_resolvable = isCenterSetResolvableOnLocalHGrid(expl_manager_->hgrid_,
                                 msg->ego_centers, 1.5, invalid_ego_centers) &&
                             isCenterSetResolvableOnLocalHGrid(expl_manager_->hgrid_,
                                 msg->other_centers, 1.5, invalid_other_centers);
      }
      if (!centers_resolvable) {
        if (fp_ && fp_->debug_opt_output_) {
          RCLCPP_WARN(node_->get_logger(),
              "[Meeting Opt] Drone %d reject from %d: unresolved centers ego=%d other=%d", getId(),
              msg->from_drone_id, invalid_ego_centers, invalid_other_centers);
        }
        sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
            msg->stamp, kMeetingOptProposalResponse, kMeetingOptReject);
        optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: reject proposal due to unresolved "
                          "centers, response sent");
        return;
      }
    }

    // No opt attempt recently, and the grid info between drones are consistent, accept proposal.
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN(node_->get_logger(), "[Meeting Opt] Drone %d accept proposal from %d", getId(),
          msg->from_drone_id);
    }
    ed->opt_pending_host_id_ = msg->from_drone_id;
    ed->opt_pending_stamp_ = msg->stamp;
    ed->opt_pending_recv_time_ = now;
    sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
        msg->stamp, kMeetingOptProposalResponse, kMeetingOptAccept);
    optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: proposal accepted");
    return;
  }

  if (is_cancel) {
    const bool has_pending = hasOptPendingFromHost(*ed, msg->from_drone_id, msg->stamp);
    const bool has_unconfirmed = hasAppliedUnconfirmedTx(*ed, msg->from_drone_id, msg->stamp);
    if (!has_pending && !has_unconfirmed) {
      optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: cancel without pending, from=" +
                            std::to_string(msg->from_drone_id));
      return;
    }
    if (has_pending) {
      clearOptPendingLock(*ed);
    }
    if (has_unconfirmed) {
      rollbackAppliedUnconfirmedTx(*ed, getId());
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[Meeting Opt] Drone "
                                                    << getId() << " rollback tx from "
                                                    << msg->from_drone_id
                                                    << " stamp=" << msg->stamp);
      }
      // Only back off when an already-applied commit was rolled back.
      state2.recent_attempt_time_ = node_->now().seconds();
    }
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[Meeting Opt] Drone "
                                                  << getId() << " cancel pending from "
                                                  << msg->from_drone_id << " stamp=" << msg->stamp);
    }
    state1.recent_interact_time_ = msg->stamp;
    optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: cancel handled and pending cleared");
    return;
  }

  // Commit phase
  if (hasAppliedUnconfirmedTx(*ed, msg->from_drone_id, msg->stamp)) {
    sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
        msg->stamp, kMeetingOptCommitAck, kMeetingOptAccept);
    return;
  }
  if (!hasOptPendingFromHost(*ed, msg->from_drone_id, msg->stamp)) {
    sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
        msg->stamp, kMeetingOptCommitAck, kMeetingOptReject);
    optDebugWarn(fp_, "[Meeting Opt Debug] optMsg return: commit without pending, from=" +
                          std::to_string(msg->from_drone_id));
    return;
  }
  vector<int> msg_ego_ids(msg->ego_ids.begin(), msg->ego_ids.end());
  vector<int> msg_other_ids(msg->other_ids.begin(), msg->other_ids.end());
  vector<Vector3d> msg_ego_centers, msg_other_centers;
  msgPointsToCenters(msg->ego_centers, msg_ego_centers);
  msgPointsToCenters(msg->other_centers, msg_other_centers);
  vector<vector<Vector3d>> msg_ego_hulls, msg_other_hulls;
  unpackHullsFromMsg(msg->ego_hull_offsets, msg->ego_hull_points, msg_ego_hulls);
  unpackHullsFromMsg(msg->other_hull_offsets, msg->other_hull_points, msg_other_hulls);
  if (msg_ego_hulls.size() < msg_ego_centers.size()) msg_ego_hulls.resize(msg_ego_centers.size());
  if (msg_other_hulls.size() < msg_other_centers.size()) {
    msg_other_hulls.resize(msg_other_centers.size());
  }

  const DroneState host_state_before = state1;
  const DroneState self_state_before = state2;
  {
    std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
    applyAllocationToState(expl_manager_->hgrid_, msg_ego_ids, msg_ego_centers, msg_ego_hulls,
        state1);
    applyAllocationToState(expl_manager_->hgrid_, msg_other_ids, msg_other_centers, msg_other_hulls,
        state2);
  }

  if (fp_ && fp_->debug_opt_output_) {
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "[Meeting Opt] Drone " << getId() << " commit apply: state1(drone " << msg->from_drone_id
                               << ") grids=" << state1.grid_ids_.size()
                               << " centers=" << state1.center_ids_.size() << " state2(drone "
                               << getId() << ") grids=" << state2.grid_ids_.size()
                               << " centers=" << state2.center_ids_.size());
  }

  state1.recent_interact_time_ = msg->stamp;
  state2.recent_attempt_time_ = node_->now().seconds();
  ed->reallocated_ = true;
  ed->opt_commit_backup_host_state_ = host_state_before;
  ed->opt_commit_backup_self_state_ = self_state_before;
  ed->opt_commit_backup_valid_ = true;
  ed->opt_commit_apply_pending_ = true;
  ed->opt_commit_apply_host_id_ = msg->from_drone_id;
  ed->opt_commit_apply_stamp_ = msg->stamp;
  ed->opt_commit_apply_time_ = node_->now().seconds();

  clearOptPendingLock(*ed);
  sendMeetingOptResponseAdaptive(opt_res_pub_, fp_, msg->to_drone_id, msg->from_drone_id,
      msg->stamp, kMeetingOptCommitAck, kMeetingOptAccept);

  if (state_ == IDLE && !state2.grid_ids_.empty()) {
    transitState(PLAN_TRAJ, "optMsgCallback");
    RCLCPP_WARN(node_->get_logger(), "Restart after opt!");
  }
}

void C2ExplorationFSM::optResMsgCallback(
    const c2x_msgs::msg::MeetingOptResponse::SharedPtr msg) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) {
    optDebugTrace(fp_, "[Meeting Opt Debug] optRes return: not for me/self msg from=" +
                           std::to_string(msg->from_drone_id) +
                           " to=" + std::to_string(msg->to_drone_id));
    return;
  }
  if (state_ == FINISH) {
    optDebugWarn(fp_, "[Meeting Opt Debug] optRes return: state=FINISH");
    return;
  }

  auto ed = expl_manager_->ed_;
  if (msg->from_drone_id < 1 ||
      msg->from_drone_id > static_cast<int>(ed->swarm_state_.size())) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[Meeting Opt] Ignore opt_res from out-of-range id=%d (drone_num=%zu)", msg->from_drone_id,
        ed->swarm_state_.size());
    return;
  }
  const bool is_proposal_res = (msg->phase == kMeetingOptProposalResponse);
  const bool is_commit_ack = (msg->phase == kMeetingOptCommitAck);
  if (!is_proposal_res && !is_commit_ack) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[Meeting Opt] Host " << getId() << " drop opt_res from " << msg->from_drone_id
                                << ": unknown phase=" << static_cast<int>(msg->phase));
    }
    return;
  }

  if (fabs(ed->meeting_opt_stamp_ - msg->stamp) > 1e-5) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[Meeting Opt] Host " << getId() << " drop opt_res from " << msg->from_drone_id
                                << ": wait_proposal=" << ed->wait_opt_response_
                                << " wait_commit=" << ed->wait_opt_commit_ack_
                                << " expect_stamp=" << ed->meeting_opt_stamp_
                                << " got=" << msg->stamp);
    }
    optDebugWarn(fp_, "[Meeting Opt Debug] optRes return: stamp/session mismatch, wait=" +
                          std::to_string(int(ed->wait_opt_response_)) + "/" +
                          std::to_string(int(ed->wait_opt_commit_ack_)) +
                          " expect=" + std::to_string(ed->meeting_opt_stamp_) +
                          " got=" + std::to_string(msg->stamp));
    return;
  }

  vector<double>& phase_stamps =
      is_commit_ack ? ed->meeting_opt_commit_res_stamps_ : ed->meeting_opt_res_stamps_;
  if (msg->stamp <= phase_stamps[msg->from_drone_id - 1] + 1e-4) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[Meeting Opt] Host " << getId() << " drop opt_res from " << msg->from_drone_id
                                << ": old stamp=" << msg->stamp
                                << " phase=" << static_cast<int>(msg->phase));
    }
    optDebugTrace(fp_,
        "[Meeting Opt Debug] optRes return: old stamp from=" + std::to_string(msg->from_drone_id) +
            " stamp=" + std::to_string(msg->stamp));
    return;
  }
  phase_stamps[msg->from_drone_id - 1] = msg->stamp;

  if (is_proposal_res) {
    if (!ed->wait_opt_response_ || ed->wait_opt_commit_ack_) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
            "[Meeting Opt] Host " << getId() << " drop proposal-res from " << msg->from_drone_id
                                  << ": no proposal wait session");
      }
      return;
    }

    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
          "[Meeting Opt] Host " << getId() << " recv proposal-res from " << msg->from_drone_id
                                << " status=" << msg->status << " stamp=" << msg->stamp
                                << " wait=" << ed->opt_wait_ids_.size()
                                << " ack=" << ed->opt_ack_ids_.size());
    }

    if (msg->status != kMeetingOptAccept) {
      const double now = node_->now().seconds();
      const double cooldown = registerPeerRejectBackoff(*ed, fp_, msg->from_drone_id, now);
      ed->swarm_state_[msg->from_drone_id - 1].recent_interact_time_ = now;
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[Meeting Opt] Host "
                                                    << getId() << " cooldown peer "
                                                    << msg->from_drone_id
                                                    << " after reject: hold=" << cooldown << "s");
        RCLCPP_WARN(node_->get_logger(), "[Meeting Opt] Abort: drone %d rejected proposal",
            msg->from_drone_id);
      }
      forEachRemoteParticipant(ed->opt_participant_ids_, getId(), [&](size_t /*idx*/, int to_id) {
        sendMeetingOptAdaptive(
            opt_pub_, fp_, getId(), to_id, ed->meeting_opt_stamp_, kMeetingOptCancel);
      });
      clearHostOptSession(*ed);
      return;
    }

    clearPeerRejectBackoff(*ed, msg->from_drone_id);

    pushUniqueId(ed->opt_ack_ids_, msg->from_drone_id);
    if (ed->opt_ack_ids_.size() < ed->opt_wait_ids_.size()) return;

    size_t host_idx = 0;
    if (!findParticipantIndex(ed->opt_participant_ids_, getId(), host_idx)) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
            "[Meeting Opt] Host " << getId() << " clear invalid proposal wait session");
      }
      clearHostOptSession(*ed);
      return;
    }

    forEachRemoteParticipant(ed->opt_participant_ids_, getId(), [&](size_t i, int to_id) {
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "[Meeting Opt] Host " << getId() << " send commit to " << to_id
                                  << " ego=" << ed->opt_alloc_ids_[host_idx].size()
                                  << " peer=" << ed->opt_alloc_ids_[i].size());
      }
      sendMeetingOptAdaptive(opt_pub_, fp_, getId(), to_id, ed->meeting_opt_stamp_,
          kMeetingOptCommit, ed->opt_alloc_ids_[host_idx], ed->opt_alloc_ids_[i],
          ed->opt_alloc_centers_[host_idx], ed->opt_alloc_centers_[i],
          ed->opt_alloc_center_hulls_[host_idx], ed->opt_alloc_center_hulls_[i]);
    });

    const double now = node_->now().seconds();
    ed->wait_opt_response_ = false;
    ed->wait_opt_commit_ack_ = true;
    ed->opt_commit_ack_ids_.clear();
    ed->opt_commit_wait_start_time_ = now;
    ed->opt_commit_last_send_time_ = now;
    ed->opt_commit_retry_count_ = 0;
    return;
  }

  // Commit ACK phase
  if (!ed->wait_opt_commit_ack_ || ed->wait_opt_response_) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[Meeting Opt] Host " << getId() << " drop commit-ack from " << msg->from_drone_id
                                << ": no commit wait session");
    }
    return;
  }

  if (fp_ && fp_->debug_opt_output_) {
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "[Meeting Opt] Host " << getId() << " recv commit-ack from " << msg->from_drone_id
                              << " status=" << msg->status << " stamp=" << msg->stamp
                              << " wait=" << ed->opt_wait_ids_.size()
                              << " ack=" << ed->opt_commit_ack_ids_.size());
  }

  if (msg->status != kMeetingOptAccept) {
    if (fp_ && fp_->debug_opt_output_) {
      RCLCPP_WARN(node_->get_logger(), "[Meeting Opt] Abort: drone %d reject/invalid commit ack",
          msg->from_drone_id);
    }
    forEachRemoteParticipant(ed->opt_participant_ids_, getId(), [&](size_t /*idx*/, int to_id) {
      sendMeetingOptAdaptive(
          opt_pub_, fp_, getId(), to_id, ed->meeting_opt_stamp_, kMeetingOptCancel);
    });
    clearHostOptSession(*ed);
    return;
  }

  pushUniqueId(ed->opt_commit_ack_ids_, msg->from_drone_id);
  if (ed->opt_commit_ack_ids_.size() < ed->opt_wait_ids_.size()) return;

  // All commit ACKs received, apply allocation locally.
  {
    std::lock_guard<std::recursive_mutex> lock(frontier_mutex_);
    for (size_t i = 0; i < ed->opt_participant_ids_.size(); ++i) {
      auto& state = ed->swarm_state_[ed->opt_participant_ids_[i] - 1];
      applyAllocationToState(expl_manager_->hgrid_, ed->opt_alloc_ids_[i],
          ed->opt_alloc_centers_[i], ed->opt_alloc_center_hulls_[i], state);
      state.recent_interact_time_ = msg->stamp;
      if (fp_ && fp_->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[Meeting Opt] Commit drone "
                                                    << ed->opt_participant_ids_[i]
                                                    << " grids=" << state.grid_ids_.size()
                                                    << " centers=" << state.center_ids_.size());
      }
    }
  }
  ed->reallocated_ = true;

  size_t host_idx = 0;
  if (findParticipantIndex(ed->opt_participant_ids_, getId(), host_idx)) {
    forEachRemoteParticipant(ed->opt_participant_ids_, getId(), [&](size_t i, int to_id) {
      sendMeetingOptAdaptive(opt_pub_, fp_, getId(), to_id, ed->meeting_opt_stamp_,
          kMeetingOptFinalize, ed->opt_alloc_ids_[host_idx], ed->opt_alloc_ids_[i],
          ed->opt_alloc_centers_[host_idx], ed->opt_alloc_centers_[i],
          ed->opt_alloc_center_hulls_[host_idx], ed->opt_alloc_center_hulls_[i]);
    });
  }
  clearHostOptSession(*ed);

  auto& state1 = ed->swarm_state_[getId() - 1];
  if (state_ == IDLE && !state1.grid_ids_.empty()) {
    transitState(PLAN_TRAJ, "optResMsgCallback");
    RCLCPP_WARN(node_->get_logger(), "Restart after opt!");
  }
}

void C2ExplorationFSM::swarmTrajCallback(const c2x_msgs::msg::Bspline::SharedPtr msg) {
  // Get newest trajs from other drones, for inter-drone collision avoidance
  auto& sdat = planner_manager_->swarm_traj_data_;

  // Ignore self trajectory
  if (msg->drone_id == sdat.drone_id_) return;
  if (msg->drone_id < 1 || msg->drone_id > sdat.drone_num_) return;
  if (msg->pos_pts.empty() || msg->knots.empty()) return;

  // Validate the B-spline invariant before rebuilding anything from it. These
  // three numbers come straight off the wire, and NonUniformBspline indexes its
  // knot vector using dimensions derived from the control points: if they
  // disagree, evaluateDeBoor walks off the end of the array. That is not
  // hypothetical - drone 2's exploration_node segfaulted in exactly that path.
  if (msg->order < 1 ||
      msg->knots.size() != msg->pos_pts.size() + static_cast<size_t>(msg->order) + 1) {
    rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "Dropping malformed swarm trajectory from drone %d: %zu knots for %zu control points at "
        "order %d (expected %zu)",
        msg->drone_id, msg->knots.size(), msg->pos_pts.size(), msg->order,
        msg->pos_pts.size() + static_cast<size_t>(msg->order) + 1);
    return;
  }

  const double start_time = rclcpp::Time(msg->start_time).seconds();

  // Ignore outdated trajectory
  if (sdat.receive_flags_[msg->drone_id - 1] == true &&
      start_time <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
    return;

  // Convert the msg to B-spline
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  // All drones share the PX4/Gazebo world frame, so no basecoor transform is
  // needed here (the ROS1 original left that step commented out too).
  {
    std::lock_guard<std::mutex> lock(planner_manager_->swarm_traj_mutex_);
    sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
    sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
    sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = start_time;
    sdat.receive_flags_[msg->drone_id - 1] = true;
    sdat.recv_times_[msg->drone_id - 1] = node_->now().seconds();
  }

  if (state_ == EXEC_TRAJ && !planner_manager_->checkSwarmCollision(msg->drone_id)) {
    handleSwarmConflict(msg->drone_id, "swarmTrajCallback");
  }
}

// One side has to give way. Both drones detect the same conflict at the same
// instant and, left to themselves, both replan - straight back into each other,
// which is exactly what the logs showed: drones 1 and 3 traded "collide with"
// errors eight times over two seconds and one of them ended up on its back.
// Resolving it by drone id gives a total order every drone agrees on without
// any extra negotiation: the lower id keeps its trajectory, the higher id
// yields. Deadlock-free because the yielding drone is always the one that moves.
void C2ExplorationFSM::handleSwarmConflict(const int peer_id, const string& caller) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  // Precedenza all'id piu' basso solo se il compagno puo' ripianificare. Un compagno FERMO
  // (traiettoria statica: in attesa del via, in sosta, finito) non lo fara' mai, e tenere la
  // rotta voleva dire andargli addosso: seconda run a tre droni su office_small, il drone 2
  // ha urtato il drone 3 fermo in attesa del via (sciame, ottobre 2026). Verso un compagno
  // fermo si cede sempre.
  if (getId() < peer_id && !planner_manager_->swarmTrajStatica(peer_id)) {
    // Right of way: hold course, the peer is the one replanning.
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "Drone %d conflicts with drone %d - holding course (lower id has right of way)", getId(),
        peer_id);
    return;
  }

  RCLCPP_ERROR_THROTTLE(node_->get_logger(), clock, 1000,
      "Drone %d collide with drone %d - yielding, replanning", getId(), peer_id);
  fd_->avoid_collision_ = true;
  if (state_ == EXEC_TRAJ) {
    transitState(PLAN_TRAJ, caller);
  }
}

void C2ExplorationFSM::swarmTrajTimerCallback() {
  if (expl_manager_->ep_->drone_num_ < 2) return;
  if (!fd_->have_odom_) return;

  // Broadcast newest traj of this drone to others.
  //
  // Every state has to publish something. The original only covered EXEC_TRAJ
  // and WAIT_TRIGGER, so a drone that was replanning (PLAN_TRAJ/PUB_TRAJ), had
  // run out of tasks (IDLE) or was done (FINISH) went silent - and a silent
  // drone is one its peers will happily plan a trajectory straight through,
  // even though it is sitting right there hovering. Outside EXEC_TRAJ we send a
  // static trajectory pinned to the current position, which is what the drone
  // is physically doing anyway.
  if (state_ == EXEC_TRAJ) {
    swarm_traj_pub_->publish(fd_->newest_traj_);

  } else {
    // Publish a virtual traj at current pose, to avoid collision
    c2x_msgs::msg::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = static_cast<builtin_interfaces::msg::Time>(node_->now());
    bspline.traj_id = planner_manager_->local_data_.traj_id_;

    Eigen::MatrixXd pos_pts(4, 3);
    for (int i = 0; i < 4; ++i) pos_pts.row(i) = fd_->odom_pos_.transpose();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
    Eigen::VectorXd knots = tmp.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    bspline.drone_id = expl_manager_->ep_->drone_id_;
    swarm_traj_pub_->publish(bspline);
  }
}

std::string C2ExplorationFSM::idsToStr(const vector<int>& ids) {
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < ids.size(); ++i) {
    if (i) ss << " ";
    ss << static_cast<int>(ids[i]);
  }
  ss << "]";
  return ss.str();
}

void C2ExplorationFSM::centersToGridIds(const shared_ptr<HGrid>& hgrid,
    const vector<Vector3d>& centers, vector<int>& grid_ids, bool dedup) {
  grid_ids.clear();
  if (!hgrid) return;
  unordered_set<int> seen;
  for (const auto& c : centers) {
    int gid = -1;
    if (!hgrid->getGridIdByCenterPos(c, gid)) continue;
    if (!dedup || seen.insert(gid).second) {
      grid_ids.push_back(gid);
    }
  }
}

void C2ExplorationFSM::centersToCenterIds(const shared_ptr<HGrid>& hgrid,
    const vector<Vector3d>& centers, vector<int>& center_ids, bool dedup) {
  (void)dedup;
  center_ids.clear();
  center_ids.reserve(centers.size());
  if (!hgrid) {
    center_ids.resize(centers.size(), -1);
    return;
  }
  const auto& ids = hgrid->getLastCostMatrixCenterIds();
  for (const auto& c : centers) {
    int idx = -1;
    int cid = -1;
    if (hgrid->findCenterIndexByPos(c, idx, 1.0) && idx >= 0 &&
        idx < static_cast<int>(ids.size())) {
      cid = ids[idx];
    }
    center_ids.push_back(cid);
  }
}

void C2ExplorationFSM::centersToCenterGridIds(
    const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers, vector<int>& center_grid_ids) {
  center_grid_ids.clear();
  center_grid_ids.reserve(centers.size());
  if (!hgrid) {
    center_grid_ids.resize(centers.size(), -1);
    return;
  }
  for (const auto& c : centers) {
    int gid = -1;
    if (hgrid->getGridIdByCenterPos(c, gid, 1.0)) {
      center_grid_ids.push_back(gid);
    } else {
      center_grid_ids.push_back(-1);
    }
  }
}

void C2ExplorationFSM::centersToCenterHulls(const shared_ptr<HGrid>& hgrid,
    const vector<Vector3d>& centers, vector<vector<Vector3d>>& center_hulls) {
  center_hulls.clear();
  if (!hgrid) return;
  const auto& center_hulls_all = hgrid->getLastCostMatrixCenterHulls();

  center_hulls.reserve(centers.size());
  for (const auto& c : centers) {
    vector<Vector3d> hull;
    int idx = -1;
    // 1.0m tolerance keeps matching robust after short map updates.
    if (hgrid->findCenterIndexByPos(c, idx, 1.0) && idx >= 0 &&
        idx < static_cast<int>(center_hulls_all.size())) {
      hull = center_hulls_all[idx];
    }
    center_hulls.push_back(hull);
  }
}

void C2ExplorationFSM::preserveCenterHulls(const shared_ptr<HGrid>& hgrid,
    const vector<Vector3d>& old_centers, const vector<int>& old_center_ids,
    const vector<int>& old_center_grid_ids, const vector<vector<Vector3d>>& old_center_hulls,
    const vector<Vector3d>& new_centers, const vector<int>& new_center_ids,
    const vector<int>& new_center_grid_ids, vector<vector<Vector3d>>& new_center_hulls) {
  (void)old_center_ids;
  (void)new_center_ids;
  new_center_hulls.clear();
  new_center_hulls.resize(new_centers.size());

  unordered_set<int> used_old;
  for (int i = 0; i < static_cast<int>(new_centers.size()); ++i) {
    const int gid = (i < static_cast<int>(new_center_grid_ids.size())) ? new_center_grid_ids[i] : -1;
    int best_idx = -1;
    double best_d2 = 1.0;
    for (int j = 0; j < static_cast<int>(old_centers.size()); ++j) {
      if (used_old.find(j) != used_old.end()) continue;
      if (j >= static_cast<int>(old_center_hulls.size()) || old_center_hulls[j].empty()) continue;
      const int old_gid =
          (j < static_cast<int>(old_center_grid_ids.size())) ? old_center_grid_ids[j] : -1;
      if (gid >= 0 && old_gid >= 0 && gid != old_gid) continue;
      const double d2 = (new_centers[i] - old_centers[j]).squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        best_idx = j;
      }
    }
    if (best_idx >= 0) {
      used_old.insert(best_idx);
      new_center_hulls[i] = old_center_hulls[best_idx];
      continue;
    }

    if (hgrid) {
      vector<Vector3d> hull;
      if (hgrid->getCenterHullByPos(new_centers[i], hull, 1.0) && !hull.empty()) {
        new_center_hulls[i] = hull;
      }
    }
  }
}

void C2ExplorationFSM::packHullsToMsg(const vector<vector<Vector3d>>& hulls,
    vector<int32_t>& offsets, vector<geometry_msgs::msg::Point>& points) {
  offsets.clear();
  points.clear();
  offsets.reserve(hulls.size() + 1);
  offsets.push_back(0);
  for (const auto& hull : hulls) {
    for (const auto& v : hull) {
      geometry_msgs::msg::Point p;
      p.x = v.x();
      p.y = v.y();
      p.z = v.z();
      points.push_back(p);
    }
    offsets.push_back(static_cast<int32_t>(points.size()));
  }
}

void C2ExplorationFSM::unpackHullsFromMsg(const vector<int32_t>& offsets,
    const vector<geometry_msgs::msg::Point>& points, vector<vector<Vector3d>>& hulls) {
  hulls.clear();
  if (offsets.empty()) return;

  int prev = 0;
  for (size_t i = 1; i < offsets.size(); ++i) {
    int cur = offsets[i];
    if (cur < prev) {
      hulls.clear();
      return;
    }
    if (cur > static_cast<int>(points.size())) {
      hulls.clear();
      return;
    }
    vector<Vector3d> hull;
    hull.reserve(cur - prev);
    for (int j = prev; j < cur; ++j) {
      hull.emplace_back(points[j].x, points[j].y, points[j].z);
    }
    hulls.push_back(hull);
    prev = cur;
  }
}

void C2ExplorationFSM::uniqueIds(const vector<int>& in, vector<int>& out) {
  out.clear();
  unordered_set<int> seen;
  for (const int id : in) {
    if (id < 0) continue;
    if (seen.insert(id).second) out.push_back(id);
  }
}

void C2ExplorationFSM::applyAllocationToState(const shared_ptr<HGrid>& hgrid, const vector<int>& ids,
    const vector<Vector3d>& centers, const vector<vector<Vector3d>>& center_hulls,
    DroneState& state) {
  state.center_positions_ = centers;
  state.center_hulls_ = center_hulls;
  if (state.center_hulls_.size() < state.center_positions_.size()) {
    state.center_hulls_.resize(state.center_positions_.size());
  }
  centersToCenterGridIds(hgrid, state.center_positions_, state.center_grid_ids_);
  // Keep solver-provided grid ids as authoritative to avoid remap drift after movement.
  uniqueIds(ids, state.grid_ids_);
  state.center_ids_.clear();
  state.center_ids_.reserve(state.center_positions_.size());
  const auto* last_center_ids = hgrid ? &hgrid->getLastCostMatrixCenterIds() : nullptr;
  for (int i = 0; i < static_cast<int>(state.center_positions_.size()); ++i) {
    int cid = -1;
    if (hgrid && last_center_ids != nullptr) {
      int idx = -1;
      if (hgrid->findCenterIndexByPos(state.center_positions_[i], idx, 1.0) && idx >= 0 &&
          idx < static_cast<int>(last_center_ids->size())) {
        cid = (*last_center_ids)[idx];
      }
    }
    state.center_ids_.push_back(cid);
  }
}

void C2ExplorationFSM::msgPointsToCenters(
    const vector<geometry_msgs::msg::Point>& points, vector<Vector3d>& centers) {
  centers.clear();
  centers.reserve(points.size());
  for (const auto& p : points) {
    centers.emplace_back(p.x, p.y, p.z);
  }
}

void C2ExplorationFSM::centersToMsgPoints(
    const vector<Vector3d>& centers, vector<geometry_msgs::msg::Point>& points) {
  points.clear();
  points.reserve(centers.size());
  for (const auto& c : centers) {
    geometry_msgs::msg::Point p;
    p.x = c.x();
    p.y = c.y();
    p.z = c.z();
    points.push_back(p);
  }
}

void C2ExplorationFSM::fillMeetingOptPayload(const vector<int>& ego_ids,
    const vector<int>& other_ids, const vector<Vector3d>& ego_centers,
    const vector<Vector3d>& other_centers, const vector<vector<Vector3d>>& ego_hulls,
    const vector<vector<Vector3d>>& other_hulls, c2x_msgs::msg::MeetingOpt& opt) {
  opt.ego_ids.clear();
  opt.other_ids.clear();
  for (const int id : ego_ids) opt.ego_ids.push_back(id);
  for (const int id : other_ids) opt.other_ids.push_back(id);
  centersToMsgPoints(ego_centers, opt.ego_centers);
  centersToMsgPoints(other_centers, opt.other_centers);
  packHullsToMsg(ego_hulls, opt.ego_hull_offsets, opt.ego_hull_points);
  packHullsToMsg(other_hulls, opt.other_hull_offsets, opt.other_hull_points);
}

void C2ExplorationFSM::clearOptPendingLock(ExplorationData& ed) {
  ed.opt_pending_host_id_ = -1;
  ed.opt_pending_stamp_ = 0.0;
  ed.opt_pending_recv_time_ = 0.0;
}

bool C2ExplorationFSM::hasOptPendingFromHost(
    const ExplorationData& ed, const int host_id, const double stamp) {
  if (host_id <= 0 || ed.opt_pending_host_id_ != host_id) return false;
  return std::fabs(ed.opt_pending_stamp_ - stamp) <= 1e-4;
}

bool C2ExplorationFSM::hasAppliedUnconfirmedTx(
    const ExplorationData& ed, const int host_id, const double stamp) {
  if (!ed.opt_commit_apply_pending_) return false;
  if (host_id <= 0 || ed.opt_commit_apply_host_id_ != host_id) return false;
  return std::fabs(ed.opt_commit_apply_stamp_ - stamp) <= 1e-4;
}

void C2ExplorationFSM::clearAppliedUnconfirmedTx(ExplorationData& ed) {
  ed.opt_commit_apply_pending_ = false;
  ed.opt_commit_apply_host_id_ = -1;
  ed.opt_commit_apply_stamp_ = 0.0;
  ed.opt_commit_apply_time_ = 0.0;
  ed.opt_commit_backup_valid_ = false;
}

void C2ExplorationFSM::rollbackAppliedUnconfirmedTx(ExplorationData& ed, const int self_id) {
  if (!ed.opt_commit_apply_pending_ || !ed.opt_commit_backup_valid_) {
    clearAppliedUnconfirmedTx(ed);
    return;
  }
  const int host_idx = ed.opt_commit_apply_host_id_ - 1;
  const int self_idx = self_id - 1;
  if (host_idx >= 0 && host_idx < static_cast<int>(ed.swarm_state_.size())) {
    ed.swarm_state_[host_idx] = ed.opt_commit_backup_host_state_;
  }
  if (self_idx >= 0 && self_idx < static_cast<int>(ed.swarm_state_.size())) {
    ed.swarm_state_[self_idx] = ed.opt_commit_backup_self_state_;
  }
  ed.reallocated_ = true;
  clearAppliedUnconfirmedTx(ed);
}

void C2ExplorationFSM::clearHostOptSession(ExplorationData& ed) {
  ed.wait_opt_response_ = false;
  ed.wait_opt_commit_ack_ = false;
  ed.opt_wait_start_time_ = 0.0;
  ed.opt_last_send_time_ = 0.0;
  ed.opt_retry_count_ = 0;
  ed.opt_commit_wait_start_time_ = 0.0;
  ed.opt_commit_last_send_time_ = 0.0;
  ed.opt_commit_retry_count_ = 0;
  ed.opt_wait_ids_.clear();
  ed.opt_ack_ids_.clear();
  ed.opt_commit_ack_ids_.clear();
  ed.opt_alloc_ids_.clear();
  ed.opt_alloc_centers_.clear();
  ed.opt_alloc_center_hulls_.clear();
  ed.opt_participant_ids_.clear();
}

void C2ExplorationFSM::collectStateTaskGridIds(
    const DroneState& st, const shared_ptr<HGrid>& hgrid, vector<int>& out) {
  out.clear();
  out.reserve(st.grid_ids_.size() + st.center_grid_ids_.size() + st.center_positions_.size());
  unordered_set<int> seen;
  auto add_gid = [&](const int gid) {
    if (gid < 0) return;
    if (seen.insert(gid).second) out.push_back(gid);
  };
  for (const int gid : st.grid_ids_) add_gid(gid);
  for (const int gid : st.center_grid_ids_) add_gid(gid);
  if (!hgrid) return;
  for (const auto& c : st.center_positions_) {
    int gid = -1;
    if (hgrid->getGridIdByCenterPos(c, gid, 1.0)) {
      add_gid(gid);
    }
  }
}

void C2ExplorationFSM::clearPeerRejectBackoff(ExplorationData& ed, const int peer_id) {
  if (peer_id <= 0) return;
  const int idx = peer_id - 1;
  if (idx < 0 || idx >= static_cast<int>(ed.opt_peer_backoff_until_.size())) return;
  ed.opt_peer_backoff_until_[idx] = 0.0;
  if (idx < static_cast<int>(ed.opt_peer_reject_streak_.size())) {
    ed.opt_peer_reject_streak_[idx] = 0;
  }
}

double C2ExplorationFSM::registerPeerRejectBackoff(
    ExplorationData& ed, const shared_ptr<FSMParam>& fp, const int peer_id, const double now) {
  if (!fp || fp->opt_reject_backoff_ <= 1e-6 || peer_id <= 0) {
    return 0.0;
  }
  const int idx = peer_id - 1;
  if (idx < 0 || idx >= static_cast<int>(ed.opt_peer_backoff_until_.size())) return 0.0;

  int streak = 0;
  if (idx < static_cast<int>(ed.opt_peer_reject_streak_.size())) {
    streak = ed.opt_peer_reject_streak_[idx];
  }
  if (ed.opt_peer_backoff_until_[idx] + 1e-6 < now) {
    streak = 0;
  }
  streak = std::min(streak + 1, 8);
  if (idx < static_cast<int>(ed.opt_peer_reject_streak_.size())) {
    ed.opt_peer_reject_streak_[idx] = streak;
  }

  double duration = fp->opt_reject_backoff_ * std::pow(1.5, std::max(0, streak - 1));
  if (fp->opt_reject_backoff_max_ > 1e-6) {
    duration = std::min(duration, fp->opt_reject_backoff_max_);
  }
  ed.opt_peer_backoff_until_[idx] = std::max(ed.opt_peer_backoff_until_[idx], now + duration);
  return std::max(0.0, ed.opt_peer_backoff_until_[idx] - now);
}

bool C2ExplorationFSM::containsId(const vector<int>& ids, const int id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

bool C2ExplorationFSM::pushUniqueId(vector<int>& ids, const int id) {
  if (containsId(ids, id)) return false;
  ids.push_back(id);
  return true;
}

void C2ExplorationFSM::forEachRemoteParticipant(const vector<int>& participant_ids,
    const int self_id, const std::function<void(size_t, int)>& fn) {
  for (size_t idx = 0; idx < participant_ids.size(); ++idx) {
    const int peer_id = participant_ids[idx];
    if (peer_id == self_id) continue;
    fn(idx, peer_id);
  }
}

bool C2ExplorationFSM::findParticipantIndex(
    const vector<int>& participant_ids, const int drone_id, size_t& idx) {
  auto it = std::find(participant_ids.begin(), participant_ids.end(), drone_id);
  if (it == participant_ids.end()) return false;
  idx = static_cast<size_t>(it - participant_ids.begin());
  return true;
}

bool C2ExplorationFSM::isCenterSetResolvableOnLocalHGrid(const shared_ptr<HGrid>& hgrid,
    const vector<geometry_msgs::msg::Point>& points, const double tol, int& invalid_count) {
  invalid_count = 0;
  if (!hgrid) return false;
  for (const auto& p : points) {
    int gid = -1;
    if (!hgrid->getGridIdByCenterPos(Vector3d(p.x, p.y, p.z), gid, tol)) {
      ++invalid_count;
    }
  }
  return invalid_count == 0;
}

void C2ExplorationFSM::optDebugWarn(const shared_ptr<FSMParam>& fp, const std::string& msg) {
  if (!fp || !fp->debug_opt_output_) return;
  RCLCPP_WARN_STREAM(node_->get_logger(), msg);
}

void C2ExplorationFSM::optDebugTrace(const shared_ptr<FSMParam>& fp, const std::string& msg) {
  if (!fp || !fp->debug_opt_output_) return;
  rclcpp::Clock clock(RCL_ROS_TIME);
  RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), clock, 200, msg);
}

void C2ExplorationFSM::publishRepeated(
    const MeetingOptPub& pub, const c2x_msgs::msg::MeetingOpt& msg, const int repeat_send_num) {
  for (int i = 0; i < repeat_send_num; ++i) pub->publish(msg);
}

void C2ExplorationFSM::publishRepeated(const MeetingOptResPub& pub,
    const c2x_msgs::msg::MeetingOptResponse& msg, const int repeat_send_num) {
  for (int i = 0; i < repeat_send_num; ++i) pub->publish(msg);
}

int C2ExplorationFSM::getMeetingOptRepeatNum(const shared_ptr<FSMParam>& fp, const int8_t phase) {
  const int base_repeat = fp ? std::max(1, fp->repeat_send_num_) : 1;
  if (!fp) return base_repeat;
  if (phase == kMeetingOptProposal) return std::max(base_repeat, 2);
  if (phase == kMeetingOptCommit || phase == kMeetingOptCancel || phase == kMeetingOptFinalize) {
    return std::max(base_repeat, 3);
  }
  return base_repeat;
}

int C2ExplorationFSM::getMeetingOptResRepeatNum(const shared_ptr<FSMParam>& fp, const int8_t phase) {
  const int base_repeat = fp ? std::max(1, fp->repeat_send_num_) : 1;
  if (!fp) return base_repeat;
  if (phase == kMeetingOptProposalResponse) return std::max(base_repeat, 2);
  if (phase == kMeetingOptCommitAck) return std::max(base_repeat, 3);
  return base_repeat;
}

void C2ExplorationFSM::sendMeetingOptAdaptive(const MeetingOptPub& opt_pub,
    const shared_ptr<FSMParam>& fp, const int from_id, const int to_id, const double stamp,
    const int8_t phase) {
  c2x_msgs::msg::MeetingOpt opt;
  opt.from_drone_id = from_id;
  opt.to_drone_id = to_id;
  opt.stamp = stamp;
  opt.phase = phase;
  publishRepeated(opt_pub, opt, getMeetingOptRepeatNum(fp, phase));
}

void C2ExplorationFSM::sendMeetingOptAdaptive(const MeetingOptPub& opt_pub,
    const shared_ptr<FSMParam>& fp, const int from_id, const int to_id, const double stamp,
    const int8_t phase, const vector<int>& ego_ids, const vector<int>& other_ids,
    const vector<Vector3d>& ego_centers, const vector<Vector3d>& other_centers,
    const vector<vector<Vector3d>>& ego_hulls, const vector<vector<Vector3d>>& other_hulls) {
  c2x_msgs::msg::MeetingOpt opt;
  opt.from_drone_id = from_id;
  opt.to_drone_id = to_id;
  opt.stamp = stamp;
  opt.phase = phase;
  fillMeetingOptPayload(ego_ids, other_ids, ego_centers, other_centers, ego_hulls, other_hulls, opt);
  publishRepeated(opt_pub, opt, getMeetingOptRepeatNum(fp, phase));
}

void C2ExplorationFSM::sendMeetingOptResponseAdaptive(const MeetingOptResPub& opt_res_pub,
    const shared_ptr<FSMParam>& fp, const int from_id, const int to_id, const double stamp,
    const int8_t phase, const int32_t status) {
  c2x_msgs::msg::MeetingOptResponse response;
  response.from_drone_id = from_id;
  response.to_drone_id = to_id;
  response.phase = phase;
  response.status = status;
  response.stamp = stamp;
  publishRepeated(opt_res_pub, response, getMeetingOptResRepeatNum(fp, phase));
}

void C2ExplorationFSM::handleHostWaitSession(ExplorationData& ed, const shared_ptr<FSMParam>& fp,
    const MeetingOptPub& opt_pub, const int self_id, const double now,
    const bool wait_commit_ack) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  size_t host_idx = 0;
  if (!findParticipantIndex(ed.opt_participant_ids_, self_id, host_idx)) {
    if (fp && fp->debug_opt_output_) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[Meeting Opt] Host " << self_id
                                << (wait_commit_ack ? " clear invalid commit-ack waiting session" :
                                                      " clear invalid waiting session"));
    }
    clearHostOptSession(ed);
    return;
  }

  double* wait_start_time =
      wait_commit_ack ? &ed.opt_commit_wait_start_time_ : &ed.opt_wait_start_time_;
  double* last_send_time =
      wait_commit_ack ? &ed.opt_commit_last_send_time_ : &ed.opt_last_send_time_;
  int* retry_count = wait_commit_ack ? &ed.opt_commit_retry_count_ : &ed.opt_retry_count_;
  const vector<int>* ack_ids = wait_commit_ack ? &ed.opt_commit_ack_ids_ : &ed.opt_ack_ids_;
  const int8_t phase = wait_commit_ack ? kMeetingOptCommit : kMeetingOptProposal;

  if (fp->opt_wait_timeout_ > 1e-6 && now - *wait_start_time > fp->opt_wait_timeout_) {
    if (fp && fp->debug_opt_output_) {
      if (wait_commit_ack) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "[Meeting Opt] Host " << self_id
                                  << " commit-ack wait timeout, abort stamp=" << ed.meeting_opt_stamp_
                                  << " need=" << ed.opt_wait_ids_.size()
                                  << " ack=" << ed.opt_commit_ack_ids_.size());
      } else {
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "[Meeting Opt] Host " << self_id
                                  << " opt wait timeout, cancel stamp=" << ed.meeting_opt_stamp_
                                  << " wait=" << ed.opt_wait_ids_.size()
                                  << " ack=" << ed.opt_ack_ids_.size());
      }
    }
    forEachRemoteParticipant(ed.opt_participant_ids_, self_id, [&](size_t /*idx*/, int to_id) {
      sendMeetingOptAdaptive(opt_pub, fp, self_id, to_id, ed.meeting_opt_stamp_, kMeetingOptCancel);
    });
    clearHostOptSession(ed);
    return;
  }

  const bool can_retry = fp->opt_retry_max_ > 0 && *retry_count < fp->opt_retry_max_;
  if (can_retry && (now - *last_send_time >= fp->opt_retry_interval_)) {
    int resent = 0;
    for (const int to_id : ed.opt_wait_ids_) {
      if (containsId(*ack_ids, to_id)) {
        if (!wait_commit_ack) {
          optDebugWarn(fp, "[Meeting Opt Debug] wait session skip retry to " +
                               std::to_string(to_id) + ": already acked");
        }
        continue;
      }
      size_t peer_idx = 0;
      if (!findParticipantIndex(ed.opt_participant_ids_, to_id, peer_idx)) {
        if (!wait_commit_ack) {
          optDebugWarn(fp, "[Meeting Opt Debug] wait session skip retry to " +
                               std::to_string(to_id) + ": participant index missing");
        }
        continue;
      }
      sendMeetingOptAdaptive(opt_pub, fp, self_id, to_id, ed.meeting_opt_stamp_, phase,
          ed.opt_alloc_ids_[host_idx], ed.opt_alloc_ids_[peer_idx], ed.opt_alloc_centers_[host_idx],
          ed.opt_alloc_centers_[peer_idx], ed.opt_alloc_center_hulls_[host_idx],
          ed.opt_alloc_center_hulls_[peer_idx]);
      ++resent;
    }
    if (resent > 0) {
      *last_send_time = now;
      ++(*retry_count);
      if (fp && fp->debug_opt_output_) {
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "[Meeting Opt] Host " << self_id << " retry "
                                  << (wait_commit_ack ? "commit" : "proposal") << " round "
                                  << *retry_count << "/" << fp->opt_retry_max_
                                  << " resent=" << resent);
      }
    }
  } else if (!wait_commit_ack) {
    optDebugWarn(fp,
        "[Meeting Opt Debug] wait session no retry: can_retry=" + std::to_string(int(can_retry)) +
            " dt=" + std::to_string(now - *last_send_time) +
            " retry_interval=" + std::to_string(fp->opt_retry_interval_));
  }

  optDebugWarn(fp, wait_commit_ack ? "[Meeting Opt Debug] optTimer return: waiting commit acks" :
                                     "[Meeting Opt Debug] optTimer return: waiting opt responses");
}

}  // namespace c2_expl
