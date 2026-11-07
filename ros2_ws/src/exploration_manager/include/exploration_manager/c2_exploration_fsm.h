#ifndef _C2_EXPLORATION_FSM_H_
#define _C2_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <exploration_manager/expl_data.h>
#include <plan_manage/plan_container.hpp>
#include <exploration_manager/copertura_camera.h>
#include <exploration_manager/copertura_visita.h>
#include <c2x_msgs/msg/bspline.hpp>
#include <c2x_msgs/msg/drone_state.hpp>
#include <c2x_msgs/msg/exploration_ready.hpp>
#include <c2x_msgs/msg/meeting_opt.hpp>
#include <c2x_msgs/msg/meeting_opt_response.hpp>

#include <algorithm>
#include <atomic>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <functional>
#include <unordered_set>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace c2_expl {
class FastPlannerManager;
class C2ExplorationManager;
class PlanningVisualization;
class CommunicationGraph;
class HGrid;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

// Per-UAV exploration FSM. Alongside the single-drone plan/execute loop it runs
// the decentralized coordination layer of the paper: a periodic DroneState
// broadcast, the MeetingOpt commit-style task dispatch
// (Proposal -> Commit -> Finalize, with Cancel/rollback), and a trajectory
// exchange feeding the optimizer's inter-drone collision-avoidance term. All of
// those only ever talk to peers currently inside r_comm, as reported by
// CommunicationGraph.
class C2ExplorationFSM {

public:
  C2ExplorationFSM();
  ~C2ExplorationFSM();

  void init(rclcpp::Node::SharedPtr node);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static constexpr int8_t kMeetingOptProposal = 0;
  static constexpr int8_t kMeetingOptCommit = 1;
  static constexpr int8_t kMeetingOptCancel = 2;
  static constexpr int8_t kMeetingOptFinalize = 3;
  static constexpr int8_t kMeetingOptProposalResponse = 0;
  static constexpr int8_t kMeetingOptCommitAck = 1;
  static constexpr int32_t kMeetingOptAccept = 1;
  static constexpr int32_t kMeetingOptReject = 2;

  using MeetingOptPub = rclcpp::Publisher<c2x_msgs::msg::MeetingOpt>::SharedPtr;
  using MeetingOptResPub = rclcpp::Publisher<c2x_msgs::msg::MeetingOptResponse>::SharedPtr;

  /* workflow functions */
  int getId();
  int callExplorationPlanner();
  void visualize(int content);
  void drawFrontierVisualization();
  void drawPlanningVisualization();
  void clearVisMarker();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback();
  void frontierCallback();
  void triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void safetyCallback();
  void crashCheckCallback();
  // Fine esplorazione: rientro alla partenza se lontani, altrimenti atterraggio.
  void concludiEsplorazione(const char* motivo);
  void progressCheckCallback();
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /* Swarm and MeetingOpt callbacks */
  void droneStateTimerCallback();
  void droneStateMsgCallback(const c2x_msgs::msg::DroneState::SharedPtr msg);
  void optTimerCallback();
  void optMsgCallback(const c2x_msgs::msg::MeetingOpt::SharedPtr msg);
  void optResMsgCallback(const c2x_msgs::msg::MeetingOptResponse::SharedPtr msg);

  void swarmTrajCallback(const c2x_msgs::msg::Bspline::SharedPtr msg);
  void swarmTrajTimerCallback();
  // Resolve a detected trajectory conflict with one peer, by drone-id priority.
  void handleSwarmConflict(const int peer_id, const string& caller);

  /* helper functions */
  static int resolveCenterGridId(
      const shared_ptr<HGrid>& hgrid, const DroneState& drone_state, const int center_idx);
  static std::string idsToStr(const vector<int>& ids);
  static void centersToGridIds(const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers,
      vector<int>& grid_ids, bool dedup);
  static void centersToCenterIds(const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers,
      vector<int>& center_ids, bool dedup);
  static void centersToCenterGridIds(const shared_ptr<HGrid>& hgrid,
      const vector<Vector3d>& centers, vector<int>& center_grid_ids);
  static void centersToCenterHulls(const shared_ptr<HGrid>& hgrid, const vector<Vector3d>& centers,
      vector<vector<Vector3d>>& center_hulls);
  static void preserveCenterHulls(const shared_ptr<HGrid>& hgrid,
      const vector<Vector3d>& old_centers, const vector<int>& old_center_ids,
      const vector<int>& old_center_grid_ids, const vector<vector<Vector3d>>& old_center_hulls,
      const vector<Vector3d>& new_centers, const vector<int>& new_center_ids,
      const vector<int>& new_center_grid_ids, vector<vector<Vector3d>>& new_center_hulls);
  static void packHullsToMsg(const vector<vector<Vector3d>>& hulls, vector<int32_t>& offsets,
      vector<geometry_msgs::msg::Point>& points);
  static void unpackHullsFromMsg(const vector<int32_t>& offsets,
      const vector<geometry_msgs::msg::Point>& points, vector<vector<Vector3d>>& hulls);
  static void uniqueIds(const vector<int>& in, vector<int>& out);
  static void applyAllocationToState(const shared_ptr<HGrid>& hgrid, const vector<int>& ids,
      const vector<Vector3d>& centers, const vector<vector<Vector3d>>& center_hulls,
      DroneState& state);
  static void msgPointsToCenters(
      const vector<geometry_msgs::msg::Point>& points, vector<Vector3d>& centers);
  static void centersToMsgPoints(
      const vector<Vector3d>& centers, vector<geometry_msgs::msg::Point>& points);
  static void fillMeetingOptPayload(const vector<int>& ego_ids, const vector<int>& other_ids,
      const vector<Vector3d>& ego_centers, const vector<Vector3d>& other_centers,
      const vector<vector<Vector3d>>& ego_hulls, const vector<vector<Vector3d>>& other_hulls,
      c2x_msgs::msg::MeetingOpt& opt);
  static void clearOptPendingLock(ExplorationData& ed);
  static bool hasOptPendingFromHost(
      const ExplorationData& ed, const int host_id, const double stamp);
  static bool hasAppliedUnconfirmedTx(
      const ExplorationData& ed, const int host_id, const double stamp);
  static void clearAppliedUnconfirmedTx(ExplorationData& ed);
  static void rollbackAppliedUnconfirmedTx(ExplorationData& ed, const int self_id);
  static void clearHostOptSession(ExplorationData& ed);
  static void collectStateTaskGridIds(
      const DroneState& st, const shared_ptr<HGrid>& hgrid, vector<int>& out);
  static void clearPeerRejectBackoff(ExplorationData& ed, const int peer_id);
  static double registerPeerRejectBackoff(
      ExplorationData& ed, const shared_ptr<FSMParam>& fp, const int peer_id, const double now);
  static bool containsId(const vector<int>& ids, const int id);
  static bool pushUniqueId(vector<int>& ids, const int id);
  static void forEachRemoteParticipant(const vector<int>& participant_ids, const int self_id,
      const std::function<void(size_t, int)>& fn);
  static bool findParticipantIndex(
      const vector<int>& participant_ids, const int drone_id, size_t& idx);
  static bool isCenterSetResolvableOnLocalHGrid(const shared_ptr<HGrid>& hgrid,
      const vector<geometry_msgs::msg::Point>& points, const double tol, int& invalid_count);
  void optDebugWarn(const shared_ptr<FSMParam>& fp, const std::string& msg);
  void optDebugTrace(const shared_ptr<FSMParam>& fp, const std::string& msg);
  static void publishRepeated(
      const MeetingOptPub& pub, const c2x_msgs::msg::MeetingOpt& msg, const int repeat_send_num);
  static void publishRepeated(const MeetingOptResPub& pub,
      const c2x_msgs::msg::MeetingOptResponse& msg, const int repeat_send_num);
  static int getMeetingOptRepeatNum(const shared_ptr<FSMParam>& fp, const int8_t phase);
  static int getMeetingOptResRepeatNum(const shared_ptr<FSMParam>& fp, const int8_t phase);
  static void sendMeetingOptAdaptive(const MeetingOptPub& opt_pub, const shared_ptr<FSMParam>& fp,
      const int from_id, const int to_id, const double stamp, const int8_t phase);
  static void sendMeetingOptAdaptive(const MeetingOptPub& opt_pub, const shared_ptr<FSMParam>& fp,
      const int from_id, const int to_id, const double stamp, const int8_t phase,
      const vector<int>& ego_ids, const vector<int>& other_ids, const vector<Vector3d>& ego_centers,
      const vector<Vector3d>& other_centers, const vector<vector<Vector3d>>& ego_hulls,
      const vector<vector<Vector3d>>& other_hulls);
  static void sendMeetingOptResponseAdaptive(const MeetingOptResPub& opt_res_pub,
      const shared_ptr<FSMParam>& fp, const int from_id, const int to_id, const double stamp,
      const int8_t phase, const int32_t status);
  void handleHostWaitSession(ExplorationData& ed, const shared_ptr<FSMParam>& fp,
      const MeetingOptPub& opt_pub, const int self_id, const double now,
      const bool wait_commit_ack);

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<C2ExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;
  shared_ptr<CommunicationGraph> comm_graph_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, frontier_timer_;
  // Own timer, in input_cb_group_ below (not the default group) - crashCheckCallback only
  // reads fd_->odom_orient_, so it doesn't need the exclusion frontier_mutex_/the default
  // group provide, and it must NOT share the default group with frontier_timer_: that
  // timer's callback can legitimately run for 10s+ on a cluttered map, and a crash needs
  // to be caught the instant it happens, not whenever that computation next yields.
  rclcpp::TimerBase::SharedPtr crash_check_timer_;
  // No-progress watchdog, see progressCheckCallback().
  rclcpp::TimerBase::SharedPtr progress_timer_;
  // Own reentrant group, separate from the default group exec_timer_/safety_timer_/
  // frontier_timer_ live in: those timers' callbacks can legitimately run for
  // seconds on a cluttered map (grid tour / ConnectivityGraph reconstruction), and
  // if odom/trigger shared that default group they'd be starved for the same
  // duration - the FSM would keep replanning from a stale odom_pos_ snapshot.
  rclcpp::CallbackGroup::SharedPtr input_cb_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  // Conferma di prontezza: il drone DICHIARA di essere pronto invece di
  // lasciarlo dedurre dai log al banco di prova.
  rclcpp::Publisher<c2x_msgs::msg::ExplorationReady>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
  void publishReadiness();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // Guards expl_manager_'s frontier/grid-tour state (updateFrontierStruct,
  // findGlobalTourOfGrid, and HGrid's last-cost-matrix center tables) against concurrent
  // access from triggerCallback (input_cb_group_) and everything in the default group
  // (frontierCallback and the MeetingOpt handlers), all of which touch the same state.
  // On a cluttered real scene that computation can take several seconds per cycle and
  // re-fires back to back, so without this the default group starves trigger_sub_
  // indefinitely.
  // RICORSIVO. Con std::mutex semplice il blocco aggiunto attorno alla
  // pianificazione si autobloccava: callExplorationPlanner lo prende, e alcuni
  // percorsi che portano fin li' lo tengono gia' - risultato, zero cicli di
  // pianificazione su tutti i droni e stallo totale, intercettato dal
  // rilevatore del banco di prova dopo 90 s.
  std::recursive_mutex frontier_mutex_;

  // Istante del via e periodo di grazia in cui una mappa ancora vuota non viene
  // scambiata per "niente da esplorare". Sostituisce il riscaldamento fisso di
  // 30 s che lo script imponeva prima di ogni run.
  rclcpp::Time trigger_time_{ 0, 0, RCL_ROS_TIME };
  // Ultimo conteggio di frontiere calcolato, per il rilevatore di non
  // convergenza globale: e' un numero GLOBALE (mappa condivisa), quindi se
  // resta fermo a lungo lo sciame intero non sta avanzando.
  // Ritrovamento: flag PROPRIO, distinto da go_back_ (rientro a base).
  // Tenerli separati e' deliberato - confonderli e' esattamente l'errore che
  // ha fatto fallire il primo tentativo, perche' go_back_ disattiva la
  // ricerca di frontiere e le sue uscite non valevano per questo caso.
  bool rdv_active_ = false;
  // Rientro alla partenza a esplorazione conclusa. Flag proprio, con la sua
  // scadenza: un drone fermo dove ha finito puo' ostruire una strettoia agli
  // altri, e per un impiego reale il rientro serve comunque.
  bool returning_ = false;
  Eigen::Vector3d return_target_ = Eigen::Vector3d::Zero();
  // Quota del rientro: sopra il disordine a livello del suolo, dove il
  // pianificatore trova strada. Sotto box_max_z (2.5) con margine.
  double return_altitude_ = 1.8;
  double return_deadline_ = 0.0;
  Eigen::Vector3d rdv_target_ = Eigen::Vector3d::Zero();
  double rdv_deadline_ = 0.0;

  int frontiers_seen_ = -1;
  int conv_last_frontiers_ = -2;
  rclcpp::Time conv_last_change_{ 0, 0, RCL_ROS_TIME };
  double min_explore_time_ = 15.0;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;
  // Fine missione normale: il ponte porta il drone sulla verticale esatta del
  // punto di decollo e poi atterra (land_pub_ resta per "atterra qui e subito").
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr home_land_pub_;
  // STATO FINALE NON REVERSIBILE: crash, volo interrotto da PX4, atterraggio
  // gia' chiesto. transitState viene chiamata da piu' gruppi di callback, e
  // un FSMCallback rimasto dentro una pianificazione lunga, al ritorno,
  // riportava in PLAN/PUB_TRAJ un drone che il controllo di crash aveva
  // appena chiuso in FINISH. Con questo flag da FINISH non si esce piu'.
  std::atomic<bool> finale_{ false };
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr flight_terminated_sub_;
  // Prontezza al via subordinata al ponte: decollo e giro iniziale conclusi
  // (fsm.wait_flight_ready, topic volo_pronto). Senza, il via poteva arrivare
  // con il drone ancora a terra o in salita.
  // Anticipo della ripianificazione in corso: fsm.replan_time di norma,
  // fsm.replan_time_collision (corto) quando si scansa un ostacolo.
  double anticipo_corrente_ = 0.2;
  double replan_time_collisione_ = 0.2;
  // Vero se lo stato di partenza del piano in corso e' letto sulla traiettoria
  // precedente (drone in moto lungo il piano), falso se e' quello misurato.
  bool partenza_da_traiettoria_ = false;
  // Istante in cui e' stato letto lo stato misurato da cui parte il piano
  // (quando partenza_da_traiettoria_ e' falso): e' l'origine dei tempi vera
  // della traiettoria nuova, vedi callExplorationPlanner.
  rclcpp::Time istante_stato_misurato_{ 0, 0, RCL_ROS_TIME };
  // Pianificazione in corso chiesta dal controllo anti-collisione (la
  // traiettoria in corso non e' piu' sicura).
  bool per_collisione_ = false;
  // Traiettoria in corso = frenata d'emergenza (vedi callExplorationPlanner):
  // il controllo anti-collisione la lascia finire.
  std::atomic<bool> frenata_{ false };
  // Almeno una traiettoria e' stata mandata al server.
  bool traiettoria_pubblicata_ = false;
  static constexpr int PIANO_IN_COLLISIONE = 101;
  rclcpp::Time riprova_dopo_{ 0, 0, RCL_ROS_TIME };
  int piani_in_collisione_ = 0;  // per la diagnostica nei log
  // Copia della traiettoria in corso fatta prima di pianificare: serve a
  // scartare un piano arrivato troppo tardi (vedi callExplorationPlanner) e a
  // rimettere al suo posto quella che il drone sta ancora seguendo.
  LocalTrajData traj_prima_;
  bool traj_prima_valida_ = false;
  static constexpr int PIANO_SCADUTO = 100;

  // COMPLETAMENTO FOTOGRAFICO (vedi copertura_camera.h): finite le frontiere,
  // il drone va a inquadrare con la telecamera le superfici della mappa 3D
  // rimaste grigie, prima di rientrare.
  std::unique_ptr<CoperturaCamera> copertura_;
  std::mutex copertura_mutex_;
  rclcpp::TimerBase::SharedPtr copertura_timer_;
  bool completamento_attivo_ = true;   // fsm.photo_completion
  double completamento_durata_ = 300.0;  // fsm.photo_completion_time
  bool completamento_ = false;         // fase in corso
  bool completamento_fatto_ = false;
  double completamento_scadenza_ = 0.0;
  Eigen::Vector3d meta_foto_ = Eigen::Vector3d::Zero();
  double yaw_foto_ = 0.0;
  Eigen::Vector3d zona_foto_ = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> zone_scartate_;
  bool in_sosta_foto_ = false;
  rclcpp::Time arrivo_foto_;
  int zone_fatte_ = 0;
  int tentativi_foto_ = 0;   // tratti pianificati verso il punto di vista corrente
  bool prossimaZonaFoto();
  // VISITA (vedi copertura_visita.h): nella stessa fase, PRIMA delle foto, il
  // drone va nelle zone di spazio percorribile in cui non e' mai passato.
  std::unique_ptr<CoperturaVisita> visita_;
  bool visita_attiva_ = true;          // fsm.visit_completion
  bool zona_di_visita_ = false;        // la meta corrente e' una visita (niente sosta)
  // Tetto di tempo per la zona di visita corrente (tempo dello stack): vedi FSMCallback.
  double scadenza_zona_ = 0.0;
  int zone_visitate_ = 0;
  bool prossimaZonaVisita();
  // SCIAME, fase di visita (ottobre 2026). La visita era per drone: ognuno segnava
  // visitato solo dove passava lui e sceglieva le zone come se fosse solo, quindi con
  // tre droni ciascuno avrebbe rifatto da se' tutta la visita, andando spesso tutti
  // verso le stesse zone. Dai DroneState dei compagni (20 Hz) si tengono qui posizione,
  // fase della missione e meta di visita: le loro posizioni segnano visitato anche per
  // questo drone (copertura_timer_) e le zone si spartiscono (prossimaZonaVisita).
  // Scritto dalla callback dei DroneState (gruppo di serie), letto dal timer della
  // copertura (input_cb_group_): da qui il mutex.
  struct PariVisita {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    double recv = -1.0;   // istante locale di ricezione [s], <0 = mai ricevuto
    int fase = 0;         // come DroneState.fase
    bool ha_meta = false;
    Eigen::Vector3d meta = Eigen::Vector3d::Zero();
  };
  std::vector<PariVisita> pari_visita_;
  std::mutex pari_mutex_;
  // Sottoscrizione dedicata ai DroneState dei compagni per la visita, in input_cb_group_: quella
  // principale (gruppo di serie) resta ferma durante le pianificazioni lunghe e i messaggi
  // intanto si perdono (coda di 10), lasciando buchi nella pianta della visita proprio dove i
  // compagni sono passati (vedi init).
  rclcpp::Subscription<c2x_msgs::msg::DroneState>::SharedPtr pari_visita_sub_;
  // Fase della missione di questo drone, come DroneState.fase.
  int faseMissione() const;
  bool prossimaZona();                 // visita, poi foto
  bool puntoDiVistaFoto(const ZonaNonInquadrata& z, Eigen::Vector3d& vp, double& yaw);
  bool spazioLibero(const Eigen::Vector3d& p, double raggio);
  void chiudiCompletamento(const char* perche);
  bool attendi_volo_pronto_ = false;
  std::atomic<bool> volo_pronto_{ false };
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr volo_pronto_sub_;
  // Tetto al tempo di esplorazione (secondi dal via, 0 = nessun tetto): oltre,
  // si rientra e si atterra con la mappa raccolta fin li'. Garantisce che la
  // missione finisca anche se il criterio sulle frontiere non converge.
  double max_explore_time_ = 0.0;
  rclcpp::Publisher<c2x_msgs::msg::Bspline>::SharedPtr bspline_pub_;

  // Swarm state
  rclcpp::Publisher<c2x_msgs::msg::DroneState>::SharedPtr drone_state_pub_;
  MeetingOptPub opt_pub_;
  MeetingOptResPub opt_res_pub_;
  rclcpp::Publisher<c2x_msgs::msg::Bspline>::SharedPtr swarm_traj_pub_;
  rclcpp::Subscription<c2x_msgs::msg::DroneState>::SharedPtr drone_state_sub_;
  rclcpp::Subscription<c2x_msgs::msg::MeetingOpt>::SharedPtr opt_sub_;
  rclcpp::Subscription<c2x_msgs::msg::MeetingOptResponse>::SharedPtr opt_res_sub_;
  rclcpp::Subscription<c2x_msgs::msg::Bspline>::SharedPtr swarm_traj_sub_;
  rclcpp::TimerBase::SharedPtr drone_state_timer_, opt_timer_, swarm_traj_timer_;
};

}  // namespace c2_expl

#endif
