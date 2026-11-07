#ifndef _C2_EXPLORATION_MANAGER_H_
#define _C2_EXPLORATION_MANAGER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <c2x_msgs/srv/solve_tsp.hpp>
#include <Eigen/Eigen>
#include <map>
#include <set>
#include <memory>
#include <vector>
#include <limits>

#include <active_perception/hgrid.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::vector;

namespace c2_expl {
class EDTEnvironment;
class SDFMap;
class CommunicationGraph;
class FastPlannerManager;
// class UniformGrid;
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;
struct DroneState;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED, NO_GRID };

class C2ExplorationManager {
public:
  C2ExplorationManager();
  ~C2ExplorationManager();

  void initialize(rclcpp::Node::SharedPtr node);

  int planExploreMotion(
      const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);

  int planTrajToView(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
      const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw);

  int updateFrontierStruct(const Eigen::Vector3d& pos);
  // Mette in coda per l'archiviazione le frontiere che questo drone tiene ma non
  // riesce a eliminare. Restituisce quante ne ha accodate.
  int banHeldFrontiers();
  // Archivia le frontiere il cui punto di vista e' irraggiungibile da pos.
  int banUnreachableFrontiers(const Eigen::Vector3d& pos);
  // Prossima tappa verso casa: casa stessa se raggiungibile, altrimenti il
  // punto piu' avanzato raggiungibile sulla congiungente. Falso se nemmeno
  // il 20% del tragitto e' percorribile.
  bool findReturnWaypoint(const Vector3d& pos, const Vector3d& home, Vector3d& out);

  struct AllocationRequest {
    vector<Eigen::Vector3d> drone_positions;
    vector<Eigen::Vector3d> drone_velocities;
    vector<int> drone_ids;
    vector<int> grid_ids;
    const vector<DroneState>* drone_states = nullptr;
    // Non-hull centers are protected by connectivity graph node id.
    vector<int> blocked_center_node_ids;
    // Split hull tasks are protected by hull geometry.
    vector<vector<Vector3d>> blocked_center_hulls;
  };

  struct AllocationResult {
    vector<vector<Vector3d>> centers;
    vector<vector<vector<Vector3d>>> center_hulls;
  };

  void allocateTasks(const AllocationRequest& request, AllocationResult& result);

  // Vero se la griglia e' spartita a strisce fra i droni (vedi findGlobalTourOfGrid): in quel
  // caso la negoziazione MeetingOpt non serve e non deve riassegnare celle fuori striscia.
  bool spartizioneAStrisce() { return calcolaStrisce(); }
  // Drone a cui spetta la striscia che contiene p (-1 se la griglia non e' spartita).
  int proprietarioStriscia(const Eigen::Vector3d& p);

  // Find optimal tour visiting unknown grid
  bool findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, vector<int>& ids, vector<vector<int>>& others,
      bool init = false);

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<CommunicationGraph> comm_graph_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<HGrid> hgrid_;
  shared_ptr<SDFMap> sdf_map_;
  // shared_ptr<UniformGrid> uniform_grid_;

private:
  static constexpr int kLkhCostScale = 100;
  static constexpr int kLkhPrecision = 1;
  static constexpr int kLkhMaxEdgeWeight =
      std::numeric_limits<int>::max() / 2 / kLkhPrecision - 1;
  static constexpr double kLkhFallbackBlockedCost = 1000.0;

  struct AllocationCandidateSet {
    Eigen::MatrixXd mat;
    vector<int> center_grid_ids;
    vector<Vector3d> center_positions;
    vector<vector<Vector3d>> center_hulls;
    vector<int> center_ids;
    vector<int> center_types;
    int full_center_num = 0;
  };

  // Find optimal coordinated tour for quadrotor swarm
  void findGridAndFrontierPath(const Vector3d& cur_pos, const Vector3d& cur_vel,
      const Vector3d& cur_yaw, vector<int>& grid_ids, vector<int>& ftr_ids);

  void shortenPath(vector<Vector3d>& path);

  void findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
      const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos, vector<int>& ids);

  bool buildAllocationCandidateSet(
      const AllocationRequest& request, AllocationCandidateSet& candidates);

  static int toLkhEdgeWeight(const double raw_cost, int& sanitized_count);
  static void parseMultiTours(
      const vector<int>& ids, const int drone_num, const int dimension, vector<vector<int>>& tours);
  static bool pointInsideConvexHull2D(
      const vector<Vector3d>& hull, const Vector3d& p, const double tol = 0.25);
  static Vector3d hullCentroid(const vector<Vector3d>& hull);
  static bool hasExplicitSplitCenter(
      const vector<int>& candidate_indices, const vector<vector<Vector3d>>& center_hulls);
  static bool candidateBlockedByHull(
      const Vector3d& center, const vector<vector<Vector3d>>& blocked_center_hulls);
  static void filterMeetingOptCentersByParticipantTasks(const shared_ptr<HGrid>& hgrid,
      const vector<DroneState>& participant_states, const vector<int>& center_grid_ids,
      const vector<Vector3d>& center_positions, const vector<int>& center_ids,
      const vector<int>& center_types, const vector<vector<Vector3d>>& center_hulls,
      vector<int>& selected_indices);
  static void collectAssignedHullTasks(const shared_ptr<HGrid>& hgrid,
      const vector<DroneState>* states, vector<HGrid::HullTask>& hull_tasks);
  static void keepCenterSubset(const vector<int>& keep_indices, const int drone_num,
      Eigen::MatrixXd& mat, vector<int>& center_grid_ids, vector<Vector3d>& center_positions,
      vector<vector<Vector3d>>& center_hulls, vector<int>& center_ids, vector<int>& center_types);

  // Frontiere visitate senza risultato: chiave = baricentro quantizzato a 10 cm,
  // valore = (celle viste l'ultima volta, tentativi consecutivi senza calo).
  // Una frontiera puo' avere punti di vista validi e restare comunque
  // inosservabile - celle in intercapedini piu' strette del velivolo, viste
  // solo di scorcio - e il campionamento dei punti di vista non ha modo di
  // accorgersene. Solo l'esito della visita lo rivela.
  // SPARTIZIONE A STRISCE (sciame, ottobre 2026): proprietario di ogni cella della griglia,
  // calcolato una volta dalle partenze di tutti i droni (vedi calcolaStrisce).
  bool calcolaStrisce();
  std::map<int, int> strisce_;   // cella -> drone_id
  bool strisce_pronte_ = false;
  // Centri delle celle (per trovare la cella di un punto: la griglia e' regolare, quindi e'
  // quella col centro piu' vicino).
  std::vector<std::pair<int, Eigen::Vector3d>> centri_celle_;
  // Vero se p cade in una cella su cui un compagno con uno stato recente sta lavorando (la
  // tiene nelle sue grid_ids_): con le strisce, quella frontiera e' sua.
  bool inCellaDiUnCompagno(const Eigen::Vector3d& p);
  // Bersaglio della pianificazione precedente: serve a non cambiare idea a
  // ogni ciclo (vedi l'isteresi in planExploreMotion).
  Eigen::Vector3d last_target_avg_ = Eigen::Vector3d::Zero();
  bool has_last_target_ = false;
  // Vero se il modello di visibilita' copre 360 gradi in orizzontale.
  bool omni_yaw_ = false;
  // Modalita' dell'ambiente, dal rilevatore in cloud_transform_bridge.
  // Si parte da chiuso: e' il caso conservativo, coordina di piu'.
  bool closed_mode_ = true;
  double fp_state_fresh_ = 1.2;
  // Celle possedute al ciclo precedente, per l'isteresi sulla proprieta'.
  std::set<int> owned_last_;
  // Pari con cui si e' calcolata l'ultima ripartizione. Serve a distinguere
  // 'e' comparso qualcuno di nuovo' (si rinegozia) da 'e' scomparso qualcuno'
  // (si tiene il proprio): con un collegamento ottico intermittente e' la
  // differenza fra coordinarsi e rubarsi il lavoro a vicenda.
  std::set<int> peers_last_;
  // Ogni quanto si puo' ricalcolare la ripartizione: senza un minimo di
  // permanenza l'assegnazione oscilla a ogni ciclo.
  double repartition_interval_ = 3.0;
  double last_repartition_ = 0.0;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr env_sub_;

  struct FrontierProgress {
    size_t min_cells;      // minimo mai raggiunto
    int strikes;           // selezioni senza nuovi progressi, DOPO essersi avvicinati
    bool been_close;       // il velivolo si e' avvicinato abbastanza da poterla osservare
    Eigen::Vector3d avg;   // baricentro esatto: banFrontier confronta su questo
  };
  std::map<std::tuple<int, int, int>, FrontierProgress> visit_strikes_;
  // Archiviazioni decise durante la pianificazione e applicate in
  // updateFrontierStruct: mutare frontiers_ dal percorso di pianificazione
  // corrompe l'heap (altri thread iterano la stessa lista).
  std::vector<Eigen::Vector3d> pending_bans_;

  rclcpp::Node::SharedPtr node_;
  // Dedicated node+executor for the blocking LKH service calls, spun via
  // rclcpp::spin_until_future_complete() from inside callSolveTSP(). Kept
  // separate from node_ (which the main MultiThreadedExecutor spins) so the
  // blocking wait never depends on the main executor's callback-group
  // scheduling - this is the standard documented pattern for calling a
  // service synchronously from within another callback.
  rclcpp::Node::SharedPtr tsp_client_node_;
  shared_ptr<EDTEnvironment> edt_environment_;
  rclcpp::Client<c2x_msgs::srv::SolveTSP>::SharedPtr tsp_client_, acvrp_client_;

public:
  typedef shared_ptr<C2ExplorationManager> Ptr;
};

}  // namespace c2_expl

#endif
