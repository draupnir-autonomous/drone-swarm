// #include <fstream>
#include <exploration_manager/c2_exploration_manager.h>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include <active_perception/graph_node.h>
#include <active_perception/perception_utils.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/hgrid.h>

#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_env/communication_graph.h>
#include <plan_env/multi_map_manager.h>

#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace Eigen;

namespace c2_expl {

namespace {
// LKH service call, replacing ROS1's blocking ros::ServiceClient::call().
// Requires the exploration_node to spin with a MultiThreadedExecutor, since
// this blocks the calling (FSM timer) thread while a *different* executor
// thread processes the incoming service response.
bool callSolveTSP(rclcpp::Node::SharedPtr spin_node,
    rclcpp::Client<c2x_msgs::srv::SolveTSP>::SharedPtr client, uint8_t prob) {
  if (!client || !client->service_is_ready()) return false;
  auto req = std::make_shared<c2x_msgs::srv::SolveTSP::Request>();
  req->prob = prob;
  auto future = client->async_send_request(req);
  // No timeout, matching ROS1's blocking ros::ServiceClient::call() exactly:
  // LKH's runtime scales with problem size (RUNS=1, but larger grids can
  // still take tens of seconds), and there's no safe universal cap.
  return rclcpp::spin_until_future_complete(spin_node, future) == rclcpp::FutureReturnCode::SUCCESS;
}

// CommunicationGraph/HGrid (constructed just above in initialize()) already
// declare several of the "exploration.*" names this function also reads
// (drone_id, vm, am); declare_parameter throws on a repeat, unlike ROS1's
// idempotent nh.param.
template <typename T>
T declareIfNew(rclcpp::Node::SharedPtr node, const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) node->declare_parameter(name, default_value);
  return node->get_parameter(name).get_value<T>();
}
}  // namespace

C2ExplorationManager::C2ExplorationManager() {
}

C2ExplorationManager::~C2ExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void C2ExplorationManager::initialize(rclcpp::Node::SharedPtr node) {
  node_ = node;
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(node_);

  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;

  // Initialize communication graph with environment
  comm_graph_.reset(new CommunicationGraph(edt_environment_, node_));

  // Set communication graph directly to multi-map manager
  if (sdf_map_->getMultiMapManager()) {
    sdf_map_->getMultiMapManager()->setCommunicationGraph(comm_graph_);
  }

  frontier_finder_.reset(new FrontierFinder(edt_environment_, node_));
  // uniform_grid_.reset(new UniformGrid(edt_environment_, node_));
  hgrid_.reset(new HGrid(edt_environment_, node_));
  // view_finder_.reset(new ViewFinder(edt_environment_, node_));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  // Se il modello di visibilita' copre tutto l'orizzonte, l'imbardata non
  // cambia nulla di cio' che il sensore vede, e sceglierne una diversa da quella
  // attuale costa solo tempo: planTrajToView impone alla traiettoria un minimo
  // pari all'angolo diviso la velocita' di imbardata. In quel caso si mantiene
  // l'orientamento corrente.
  const double left = declareIfNew(node_, "perception_utils.lidar_left_angle", -1.0);
  const double right = declareIfNew(node_, "perception_utils.lidar_right_angle", -1.0);
  omni_yaw_ = (left > 3.0 && right > 3.0);
  if (omni_yaw_)
    RCLCPP_WARN(node_->get_logger(),
        "Sensore omnidirezionale in orizzontale: l'imbardata non incide sulla visibilita', "
        "si mantiene quella corrente per non pagare il tempo di rotazione");

  fp_state_fresh_ = declareIfNew(node_, "fsm.state_fresh_timeout", 1.2);
  // Modalita' dell'ambiente pubblicata dal rilevatore (ritorni verso l'alto).
  env_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "environment/closed", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr m) {
        if (m->data != closed_mode_)
          RCLCPP_WARN(node_->get_logger(), "Modalita' esplorazione: %s",
              m->data ? "CHIUSA (ripartizione esclusiva)" : "APERTA (comportamento invariato)");
        closed_mode_ = m->data;
      });

  ep_->refine_local_ = declareIfNew(node_, "exploration.refine_local", true);
  ep_->refined_num_ = declareIfNew(node_, "exploration.refined_num", -1);
  ep_->refined_radius_ = declareIfNew(node_, "exploration.refined_radius", -1.0);
  ep_->top_view_num_ = declareIfNew(node_, "exploration.top_view_num", -1);
  ep_->max_decay_ = declareIfNew(node_, "exploration.max_decay", -1.0);
  ep_->tsp_dir_ = declareIfNew(node_, "exploration.tsp_dir", string("null"));
  ep_->mtsp_dir_ = declareIfNew(node_, "exploration.mtsp_dir", string("null"));
  ep_->relax_time_ = declareIfNew(node_, "exploration.relax_time", 1.0);
  ep_->drone_num_ = declareIfNew(node_, "exploration.drone_num", 1);
  ep_->drone_id_ = declareIfNew(node_, "exploration.drone_id", 1);
  ep_->init_plan_num_ = declareIfNew(node_, "exploration.init_plan_num", 2);
  ep_->prob_type_ = declareIfNew(node_, "exploration.prob_type", 1);
  ep_->center_switch_penalty_ = declareIfNew(node_, "exploration.center_switch_penalty", 0.0);
  ep_->center_stay_bonus_ = declareIfNew(node_, "exploration.center_stay_bonus", 0.0);
  ep_->partizione_strisce_ = declareIfNew(node_, "partitioning.strisce", true);
  ep_->partenze_ = declareIfNew(node_, "exploration.partenze", std::vector<double>{});

  ed_->swarm_state_.resize(ep_->drone_num_);
  ed_->meeting_opt_stamps_.resize(ep_->drone_num_);
  ed_->meeting_opt_res_stamps_.resize(ep_->drone_num_);
  ed_->meeting_opt_commit_res_stamps_.resize(ep_->drone_num_);
  ed_->opt_peer_backoff_until_.resize(ep_->drone_num_);
  ed_->opt_peer_reject_streak_.resize(ep_->drone_num_);
  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
    ed_->swarm_state_[i].recv_stamp_ = 0.0;
    ed_->meeting_opt_stamps_[i] = 0.0;
    ed_->meeting_opt_res_stamps_[i] = 0.0;
    ed_->meeting_opt_commit_res_stamps_[i] = 0.0;
    ed_->opt_peer_backoff_until_[i] = 0.0;
    ed_->opt_peer_reject_streak_[i] = 0;
  }
  planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);

  ViewNode::vm_ = declareIfNew(node_, "exploration.vm", -1.0);
  ViewNode::am_ = declareIfNew(node_, "exploration.am", -1.0);
  ViewNode::yd_ = declareIfNew(node_, "exploration.yd", -1.0);
  ViewNode::ydd_ = declareIfNew(node_, "exploration.ydd", -1.0);
  ViewNode::w_dir_ = declareIfNew(node_, "exploration.w_dir", -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(node_, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  // The TSP/ACVRP clients live on their own dedicated node+executor (see the
  // header comment on tsp_client_node_) rather than node_, which the main
  // MultiThreadedExecutor spins - findGlobalTourOfGrid/allocateTasks call
  // these synchronously from inside a timer callback, and routing the
  // blocking wait through the same node/executor that owns that timer
  // proved to deadlock in practice (the response never got processed even
  // with a separate callback group and extra executor threads).
  tsp_client_node_ = std::make_shared<rclcpp::Node>(
      "exploration_tsp_client_" + to_string(ep_->drone_id_));
  tsp_client_ = tsp_client_node_->create_client<c2x_msgs::srv::SolveTSP>(
      "/solve_tsp_" + to_string(ep_->drone_id_));
  acvrp_client_ = tsp_client_node_->create_client<c2x_msgs::srv::SolveTSP>(
      "/solve_acvrp_" + to_string(ep_->drone_id_));

  // Swarm
  for (auto& state : ed_->swarm_state_) {
    state.stamp_ = 0.0;
    state.recv_stamp_ = 0.0;
    state.recent_interact_time_ = 0.0;
    state.recent_attempt_time_ = 0.0;
  }
  ed_->last_grid_ids_ = {};
  ed_->reallocated_ = true;
  ed_->meeting_opt_stamp_ = 0.0;
  ed_->opt_wait_start_time_ = 0.0;
  ed_->opt_last_send_time_ = 0.0;
  ed_->opt_retry_count_ = 0;
  ed_->opt_commit_wait_start_time_ = 0.0;
  ed_->opt_commit_last_send_time_ = 0.0;
  ed_->opt_commit_retry_count_ = 0;
  ed_->opt_preferred_gate_host_id_ = -1;
  ed_->opt_preferred_gate_since_ = 0.0;
  ed_->opt_pending_host_id_ = -1;
  ed_->opt_pending_stamp_ = 0.0;
  ed_->opt_pending_recv_time_ = 0.0;
  ed_->opt_commit_apply_pending_ = false;
  ed_->opt_commit_apply_host_id_ = -1;
  ed_->opt_commit_apply_stamp_ = 0.0;
  ed_->opt_commit_apply_time_ = 0.0;
  ed_->opt_commit_backup_valid_ = false;
  ed_->wait_opt_response_ = false;
  ed_->wait_opt_commit_ack_ = false;
  ed_->plan_num_ = 0;
}

int C2ExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  rclcpp::Time t1 = node_->now();
  auto t2 = t1;

  // Do global and local tour planning and retrieve the next viewpoint

  ed_->frontier_tour_.clear();
  Vector3d next_pos;
  double next_yaw;
  vector<int> grid_ids, frontier_ids;
  // Indice della frontiera effettivamente scelta, valorizzato da ogni ramo: serve
  // solo alla diagnostica in fondo alla funzione. -1 = nessuna scelta.
  int diag_fid = -1;
  findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids);

  if (grid_ids.empty()) {
    // PROVATO E SCARTATO (campagna M1, 4 run): far prendere al drone senza
    // celle la frontiera piu' vicina di cui e' il piu' prossimo NON ha ridotto
    // l'immobilita' - 47% contro 48% di riferimento, invariata - e ha raddoppiato
    // i ribaltamenti, 2 su 4 contro 4 su 16. Il ripiego si attivava (69 volte),
    // quindi il meccanismo funzionava.
    //
    // Cosa insegna, ed e' piu' utile del codice: le soste lunghe NON sono causate
    // dalla mancanza di un bersaglio. Il drone un bersaglio se lo prendeva, e
    // restava fermo lo stesso. Quindi la diagnosi "sta fermo perche' non ha
    // lavoro assegnato" e' sbagliata o incompleta, e va cercata altrove la
    // ragione per cui un drone con un obiettivo valido non avanza.
    // Verificato inoltre che i ribaltamenti aggiuntivi NON sono collisioni fra
    // droni: le fermate per vicinanza non discriminano fra run pulite e no.
    return NO_GRID;

  } else if (frontier_ids.size() == 0) {
    // Detect whether current task token is hull-based (for logging and fallback behavior).
    bool next_task_is_hull = false;
    const auto& ego_state = ed_->swarm_state_[ep_->drone_id_ - 1];
    if (ed_->region_tour_.size() > 1 && !ego_state.center_positions_.empty()) {
      int best_task_idx = -1;
      double best_d2 = std::numeric_limits<double>::infinity();
      for (int i = 0; i < static_cast<int>(ego_state.center_positions_.size()); ++i) {
        const double d2 = (ego_state.center_positions_[i] - ed_->region_tour_[1]).squaredNorm();
        if (d2 < best_d2) {
          best_d2 = d2;
          best_task_idx = i;
        }
      }
      if (best_task_idx >= 0 && best_task_idx < static_cast<int>(ego_state.center_hulls_.size()) &&
          !ego_state.center_hulls_[best_task_idx].empty()) {
        next_task_is_hull = true;
      }
    }
    if (next_task_is_hull) {
      RCLCPP_WARN(node_->get_logger(),
          "[Hull->Frontier] No frontier in current hull task, fallback to nearest frontier around "
          "task center");
    }

    // No frontier from task-constrained extraction, find the frontier nearest to the current task
    // center.
    Eigen::Vector3d grid_center = ed_->region_tour_[1];

    double min_cost = 100000;
    int min_cost_id = -1;
    // Con le strisce si escludono le frontiere nelle celle su cui un compagno sta lavorando
    // (vedi inCellaDiUnCompagno); se non resta nulla si torna a considerarle tutte.
    std::vector<bool> altrui(ed_->points_.size(), false);
    size_t n_altrui = 0;
    if (strisce_pronte_)
      for (size_t i = 0; i < ed_->averages_.size() && i < altrui.size(); ++i)
        if (inCellaDiUnCompagno(ed_->averages_[i])) { altrui[i] = true; ++n_altrui; }
    if (n_altrui == altrui.size()) std::fill(altrui.begin(), altrui.end(), false);
    for (int i = 0; i < ed_->points_.size(); ++i) {
      if (altrui[i]) continue;
      vector<Eigen::Vector3d> path;
      double cost = ViewNode::computeCost(
          grid_center, ed_->averages_[i], 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      if (ViewNode::isUnreachableCost(cost)) {
        // RCLCPP_ERROR(node_->get_logger(), "No frontier in assigned grid, frontier %d
        // unreachable", i);
        continue;
      }
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_id = i;
      }
    }
    // Nessuna frontiera raggiungibile dal CENTRO DELLA CELLA: si riprova dalla
    // posizione del DRONE. Il centro della cella e' un punto geometrico, non un
    // posto dove il velivolo si trova: se cade in una sacca o contro un muro,
    // ogni frontiera risulta irraggiungibile da li' anche quando il drone ci
    // arriva benissimo. Il ramo qui sotto allora scartava la cella e la
    // pianificazione falliva in ciclo - blocchi di 354 s (run V10_GPS 1) e di
    // 616 s (BASE_LIO 4), che l'archiviazione per fallimenti consecutivi non
    // risolveva perche' dal drone quelle frontiere sono raggiungibili.
    if (min_cost_id < 0) {
      for (int i = 0; i < static_cast<int>(ed_->points_.size()); ++i) {
        if (altrui[i]) continue;
        vector<Eigen::Vector3d> path;
        double cost = ViewNode::computeCost(
            pos, ed_->points_[i], 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
        if (ViewNode::isUnreachableCost(cost)) continue;
        if (cost < min_cost) {
          min_cost = cost;
          min_cost_id = i;
        }
      }
      if (min_cost_id >= 0)
        RCLCPP_WARN(node_->get_logger(),
            "Nessuna frontiera raggiungibile dal centro della cella: scelta la piu' vicina al drone");
    }
    if (min_cost_id < 0) {
      int bad_grid = grid_ids.empty() ? -1 : grid_ids.front();
      if (bad_grid >= 0) {
        RCLCPP_WARN(node_->get_logger(),
            "No reachable frontier found from assigned grid center, temporarily drop grid %d",
            bad_grid);

        auto& state = ed_->swarm_state_[ep_->drone_id_ - 1];
        for (auto it = state.grid_ids_.begin(); it != state.grid_ids_.end();) {
          if (*it == bad_grid) {
            it = state.grid_ids_.erase(it);
          } else {
            ++it;
          }
        }
        for (auto it = ed_->last_grid_ids_.begin(); it != ed_->last_grid_ids_.end();) {
          if (*it == bad_grid) {
            it = ed_->last_grid_ids_.erase(it);
          } else {
            ++it;
          }
        }
        ed_->reallocated_ = true;
      } else {
        RCLCPP_WARN(node_->get_logger(), "No reachable frontier found from assigned grid center");
      }
      return FAIL;
    }
    next_pos = ed_->points_[min_cost_id];
    next_yaw = ed_->yaws_[min_cost_id];
    diag_fid = min_cost_id;
  } else if (frontier_ids.size() == 1) {
    // Single frontier, find the min cost viewpoint for it
    ed_->refined_ids_ = { frontier_ids[0] };
    ed_->unrefined_points_ = { ed_->points_[frontier_ids[0]] };
    ed_->n_points_.clear();
    vector<vector<double>> n_yaws;
    frontier_finder_->getViewpointsInfo(
        pos, { frontier_ids[0] }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);
    // getViewpointsInfo SALTA le frontiere senza punti di vista (o con indice
    // fuori lista), e il codice qui sotto leggeva n_points_[0] senza
    // controllare: puntatore nullo e crash di exploration_node (run V7_LIO 3,
    // "Address not mapped to object [(nil)]" in questa funzione). Si ripiega sul
    // punto di vista principale della frontiera, se c'e'; altrimenti si
    // dichiara fallita la pianificazione, che e' gestita.
    if (ed_->n_points_.empty() || ed_->n_points_[0].empty() || n_yaws.empty() ||
        n_yaws[0].size() != ed_->n_points_[0].size()) {
      if (frontier_ids[0] < 0 || frontier_ids[0] >= static_cast<int>(ed_->points_.size()) ||
          frontier_ids[0] >= static_cast<int>(ed_->yaws_.size())) {
        RCLCPP_WARN(node_->get_logger(),
            "Frontiera %d senza punti di vista disponibili: pianificazione fallita", frontier_ids[0]);
        return FAIL;
      }
      ed_->n_points_ = { { ed_->points_[frontier_ids[0]] } };
      n_yaws = { { ed_->yaws_[frontier_ids[0]] } };
    }

    // Directly select min cost viewpoint from top N candidates
    double min_cost = 100000;
    int min_cost_id = -1;
    vector<Vector3d> tmp_path;
    for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
      if (edt_environment_->sdf_map_->getOccupancy(ed_->n_points_[0][i]) == SDFMap::OCCUPIED) {
        continue;
      }
      auto tmp_cost = ViewNode::computeCost(
          pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
      if (tmp_cost < min_cost) {
        min_cost = tmp_cost;
        min_cost_id = i;
      }
    }
    if (min_cost_id < 0) {
      // All top-N candidates are currently occupied (map updated since they were
      // generated) - using index 0 as a fallback here would silently hand astar_
      // an occupied goal, which fails every time and livelocks the FSM retrying
      // the same unreachable viewpoint forever. Fail instead, same contract as
      // the multi-frontier branch above.
      RCLCPP_WARN(node_->get_logger(), "No reachable viewpoint for single frontier %d", frontier_ids[0]);
      return FAIL;
    }
    next_pos = ed_->n_points_[0][min_cost_id];
    next_yaw = n_yaws[0][min_cost_id];
    diag_fid = frontier_ids[0];

    ed_->refined_points_ = { next_pos };
    ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
  } else {
    // Multiple frontiers, directly use the first frontier's top viewpoint
    next_pos = ed_->points_[frontier_ids[0]];
    next_yaw = ed_->yaws_[frontier_ids[0]];
    diag_fid = frontier_ids[0];

    ed_->refined_points_ = { next_pos };
    ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
  }

  // ISTERESI SUL BERSAGLIO - "finisci quello che hai cominciato".
  //
  // Il tour delle frontiere viene ricalcolato a ogni ciclo, e i cluster si
  // fondono e si dividono mentre la mappa si aggiorna: l'ordine dell'ATSP
  // cambia, la "prima frontiera" diventa un'altra e il drone inverte la rotta.
  // Misurato sui log: il bersaglio cambiava nel 38% delle pianificazioni (una
  // ogni ~3 s) e solo il 6-8% degli inseguimenti arrivava entro 3 m dalla meta.
  // I droni abbandonavano il 92% dei bersagli prima di raggiungerli - ed e' per
  // questo che l'esplorazione non convergeva, non perche' le frontiere fossero
  // inosservabili. L'assegnazione delle griglie invece era gia' stabile (2-3% di
  // cambi), quindi l'instabilita' nasce tutta qui.
  // Un'isteresi analoga esiste gia' un livello sopra, sui centri di griglia
  // (exploration.center_switch_penalty / center_stay_bonus); mancava qui.
  if (has_last_target_ && (pos - last_target_avg_).norm() > 2.0) {
    int keep = -1;
    double best = 1.0;  // stessa frontiera se il baricentro non si e' spostato oltre 1 m
    for (int i = 0; i < static_cast<int>(ed_->averages_.size()); ++i) {
      const double d = (ed_->averages_[i] - last_target_avg_).norm();
      if (d < best) {
        best = d;
        keep = i;
      }
    }
    if (keep >= 0 && keep < static_cast<int>(ed_->points_.size())) {
      next_pos = ed_->points_[keep];
      next_yaw = ed_->yaws_[keep];
      diag_fid = keep;
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    } else {
      has_last_target_ = false;  // sparita: si accetta la nuova scelta
    }
  }
  if (diag_fid >= 0 && diag_fid < static_cast<int>(ed_->averages_.size())) {
    last_target_avg_ = ed_->averages_[diag_fid];
    has_last_target_ = true;
  }

  // Sensore omnidirezionale: qualunque imbardata vede le stesse celle, quindi si
  // tiene quella attuale e il vincolo di tempo sulla rotazione sparisce.
  if (omni_yaw_) next_yaw = yaw[0];

  // std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;
  ed_->next_pos_ = next_pos;
  ed_->next_yaw_ = next_yaw;

  // Diagnostica dell'impallamento: quando un drone ricicla la stessa frontiera
  // all'infinito serve sapere DOVE stanno quelle celle, non solo che esistono.
  // Una frontiera schiacciata contro il pavimento, il soffitto o il bordo della
  // exploration box e' irraggiungibile per costruzione, e va riconosciuta come
  // tale invece che inseguita.
  // Archiviazione delle frontiere inosservabili, basata sull'ESITO della visita.
  //
  // Il campionamento dei punti di vista non basta a riconoscerle: in
  // cubicle_office ci sono intercapedini larghe 30 cm fra due muri, dove il
  // velivolo (0.5 m) non puo' entrare. Il punto di vista viene campionato fuori,
  // in spazio aperto, ed e' perfettamente valido; il drone ci va, non riesce a
  // vedere dentro la fessura, e la frontiera resta identica. Per sempre: dopo 40
  // minuti di volo le frontiere attive erano ancora ~20 per drone e non
  // scendevano, mentre le dormienti salivano oltre 50.
  // Quindi: se siamo praticamente arrivati al punto di vista e le celle non
  // calano, si segna un tentativo fallito; dopo tre, la frontiera viene
  // archiviata come dormiente e si passa oltre. Non e' un timeout - e' una
  // conclusione tratta dall'evidenza, e vale per qualunque causa di
  // irraggiungibilita', non solo per questa.
  if (diag_fid >= 0 && diag_fid < static_cast<int>(ed_->averages_.size()) &&
      diag_fid < static_cast<int>(ed_->frontiers_.size())) {
    const Eigen::Vector3d& avg = ed_->averages_[diag_fid];
    // Chiave quantizzata a 1 m, non a 10 cm: il baricentro si sposta man mano
    // che le celle si liberano, e con una griglia fine ogni selezione creava una
    // chiave nuova azzerando il conteggio. A 1 m la frontiera resta la stessa
    // per decine di selezioni (verificato sui log).
    const auto key = std::make_tuple(static_cast<int>(std::round(avg[0])),
        static_cast<int>(std::round(avg[1])), static_cast<int>(std::round(avg[2])));
    const size_t now_cells = ed_->frontiers_[diag_fid].size();
    // Il velivolo deve essersi AVVICINATO prima che un fallimento conti. Senza
    // questa condizione si archivierebbero frontiere legittime e lontane: mentre
    // ci si viaggia verso il conteggio celle non cala, quindi accumulerebbero
    // strike senza colpa. Il primo tentativo usava "sono entro 1.5 m dal punto
    // di vista al momento di pianificare", che si verifica nello 0.7% delle
    // pianificazioni - il drone pianifica quando ha appena finito una
    // traiettoria, quindi quasi sempre lontano dal prossimo bersaglio. Qui si
    // registra invece se ci si e' avvicinati in QUALCHE momento.
    const bool close_now = (pos - avg).norm() < 5.0;
    auto it = visit_strikes_.find(key);
    if (it != visit_strikes_.end()) {
      // Il baricentro si sposta mentre le celle si liberano, e banFrontier lo
      // confronta esattamente: va tenuto aggiornato a ogni selezione.
      it->second.avg = avg;
    }
    if (it == visit_strikes_.end()) {
      visit_strikes_[key] = { now_cells, 0, close_now, avg };
    } else if (now_cells < it->second.min_cells) {
      it->second.min_cells = now_cells;   // si guadagna ancora terreno
      it->second.strikes = 0;
      it->second.been_close |= close_now;
    } else if (!(it->second.been_close |= close_now)) {
      // Ancora mai avvicinati: non e' colpa della frontiera se non cala.
    } else if (++it->second.strikes >= 10) {
      // Solo REGISTRATA qui: l'archiviazione vera avviene in
      // updateFrontierStruct, che e' il punto dove il codice gia' modifica le
      // liste di frontiere. Rimuoverla da qui corrompeva l'heap.
      RCLCPP_WARN(node_->get_logger(),
          "Frontiera a (%.2f %.2f %.2f) ferma a %zu celle per 10 selezioni dopo essersi "
          "avvicinati (minimo raggiunto %zu): da archiviare",
          avg[0], avg[1], avg[2], now_cells, it->second.min_cells);
      pending_bans_.push_back(avg);
      visit_strikes_.erase(it);
    }
  }

  {
    const int fid = diag_fid;
    if (fid >= 0 && fid < static_cast<int>(ed_->frontiers_.size())) {
      const auto& cells = ed_->frontiers_[fid];
      Vector3d bmin = Vector3d::Constant(1e9), bmax = Vector3d::Constant(-1e9);
      for (const auto& c : cells)
        for (int k = 0; k < 3; ++k) {
          bmin[k] = min(bmin[k], c[k]);
          bmax[k] = max(bmax[k], c[k]);
        }
      // Le celle di frontiera sono celle LIBERE confinanti con lo SCONOSCIUTO: a
      // non essere mai osservato e' il vicino, non la cella stessa. Il riquadro
      // dei vicini sconosciuti dice quindi dove sta davvero la zona che il
      // sensore non raggiunge mai.
      Vector3d umin = Vector3d::Constant(1e9), umax = Vector3d::Constant(-1e9);
      int unk_num = 0, unk_enclosed = 0;
      auto* map = edt_environment_->sdf_map_.get();
      for (const auto& c : cells) {
        Eigen::Vector3i ci;
        map->posToIndex(c, ci);
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz) {
              if (dx == 0 && dy == 0 && dz == 0) continue;
              const Eigen::Vector3i ni(ci[0] + dx, ci[1] + dy, ci[2] + dz);
              if (map->getOccupancy(ni) != SDFMap::UNKNOWN) continue;
              Vector3d np;
              map->indexToPos(ni, np);
              for (int k = 0; k < 3; ++k) {
                umin[k] = min(umin[k], np[k]);
                umax[k] = max(umax[k], np[k]);
              }
              ++unk_num;
              // Discrimina fra "buco di campionamento" e "volume inesplorato":
              // una cella ignota stretta fra due celle LIBERE lungo la verticale
              // e' una striscia lasciata dal passo angolare del lidar, non spazio
              // da visitare. Solo nel primo caso sarebbe lecito riempirla.
              auto probe = [&](int step) {
                for (int s = 1; s <= 5; ++s) {
                  const int occ = map->getOccupancy(Eigen::Vector3i(ni[0], ni[1], ni[2] + s * step));
                  if (occ != SDFMap::UNKNOWN) return occ;
                }
                return static_cast<int>(SDFMap::UNKNOWN);
              };
              if (probe(1) == SDFMap::FREE && probe(-1) == SDFMap::FREE) ++unk_enclosed;
            }
      }
      RCLCPP_WARN(node_->get_logger(),
          "[StuckDiag] pos(%.2f %.2f %.2f) goal(%.2f %.2f %.2f) d=%.2f attive=%zu dormienti=%zu "
          "tgt_celle=%zu tgt_box[(%.2f %.2f %.2f)-(%.2f %.2f %.2f)] "
          "ignoti=%d racchiusi=%d ign_box[(%.2f %.2f %.2f)-(%.2f %.2f %.2f)] vol=%.1f",
          pos[0], pos[1], pos[2], next_pos[0], next_pos[1], next_pos[2], (next_pos - pos).norm(),
          ed_->frontiers_.size(), ed_->dead_frontiers_.size(), cells.size(), bmin[0], bmin[1],
          bmin[2], bmax[0], bmax[1], bmax[2], unk_num, unk_enclosed, umin[0], umin[1], umin[2], umax[0], umax[1],
          umax[2], edt_environment_->sdf_map_->getExploredVolume());
    }
  }

  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    return FAIL;
  }

  double total = (node_->now() - t2).seconds();
  RCLCPP_INFO(node_->get_logger(), "Total time: %lf", total);
  // RCLCPP_ERROR_EXPRESSION(node_->get_logger(), total > 0.1, "Total time too long!!!");

  return SUCCEED;
}

int C2ExplorationManager::planTrajToView(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw) {

  // Plan trajectory (position and yaw) to the next viewpoint
  auto t1 = node_->now();

  // Compute time lower bound of yaw and use in trajectory generation
  double diff0 = next_yaw - yaw[0];
  double diff1 = fabs(diff0);
  double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

  // Generate trajectory of x,y,z
  bool goal_unknown = (edt_environment_->sdf_map_->getOccupancy(next_pos) == SDFMap::UNKNOWN);
  // bool start_unknown = (edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN);
  bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;
  // Ricerca "paziente": se scade il tempo normale si riprova con 0.15 s prima di
  // concludere che il punto di vista non e' raggiungibile (vedi
  // Astar::searchPatient).
  if (planner_manager_->path_finder_->searchPatient(pos, next_pos, 0.15, true) != Astar::REACH_END) {
    int occ = edt_environment_->sdf_map_->getOccupancy(next_pos);
    int infl = edt_environment_->sdf_map_->getInflateOccupancy(next_pos);
    bool in_box = edt_environment_->sdf_map_->isInBox(next_pos);
    RCLCPP_ERROR(node_->get_logger(), "No path to next viewpoint");
    // Fallimento VERO (la ricerca paziente non e' scaduta per tempo): il punto
    // di vista non si raggiunge da qui, quindi non va riproposto (vedi
    // FrontierFinder::blacklistViewpoint).
    if (!planner_manager_->path_finder_->last_timed_out_) {
      frontier_finder_->blacklistViewpoint(next_pos);
      RCLCPP_WARN(node_->get_logger(),
          "[PuntiDiVista] (%.2f %.2f %.2f) irraggiungibile: escluso dai punti di vista",
          next_pos[0], next_pos[1], next_pos[2]);
    }
    RCLCPP_WARN(node_->get_logger(),
        "Path fail detail: start(%.2f %.2f %.2f) goal(%.2f %.2f %.2f) occ=%d infl=%d "
        "in_box=%d",
        pos[0], pos[1], pos[2], next_pos[0], next_pos[1], next_pos[2], occ, infl, in_box ? 1 : 0);
    return FAIL;
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);

  // PROVATO E SCARTATO - concatenare qui le 2 tappe successive del giro
  // (ed_->chain_points_) per allungare il tratto pianificato.
  //
  // L'idea era che i tratti sono corti - ~0.35 m, mezzo secondo - e finiscono a
  // velocita' nulla, quindi la crociera non viene mai raggiunta. Campagna da 4
  // run: velocita' in moto 0.62 m/s contro 0.73 di riferimento, tempo fermo 63%
  // contro 50%, durate 257/307/457/637 s contro una mediana di 277 s.
  // Il perche' e' misurato: il tempo di PIANIFICAZIONE e' raddoppiato (mediana
  // 0.112 s contro 0.05 s, 611 s complessivi in 4 run). L'A* e l'ottimizzazione
  // costano piu' che linearmente nella lunghezza del percorso, e il drone sta
  // fermo mentre pianifica. Allungare i piani si paga piu' di quanto renda -
  // stesso esito delle soglie di ripianificazione e dell'anticipo, provati prima.
  ed_->kino_path_.clear();

  const double radius_far = 7.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close || optimistic) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;
    // std::cout << "Close goal." << std::endl;
    if (ed_->plan_num_ < ep_->init_plan_num_) {
      ed_->plan_num_++;
      RCLCPP_WARN(node_->get_logger(), "init plan.");
    }
  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    // std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    // std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      return FAIL;
    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
    RCLCPP_ERROR(node_->get_logger(), "Lower bound not satified!");

  double traj_plan_time = (node_->now() - t1).seconds();

  t1 = node_->now();
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (node_->now() - t1).seconds();
  RCLCPP_INFO(node_->get_logger(), "\033[1m\033[38;5;214mTraj plan time: %lf, yaw plan time: %lf\033[0m",
      traj_plan_time, yaw_time);

  return SUCCEED;
}

// Archivia le frontiere che questo drone ha selezionato ma non e' riuscito a
// eliminare, chiamata dal guardiano di stallo dell'FSM.
//
// Colma una lacuna del bando ordinario: quello richiede been_close, cioe' che il
// velivolo si sia avvicinato almeno una volta. La condizione esiste per non
// archiviare frontiere lontane e legittime, ma lascia scoperto proprio il caso
// che blocca la convergenza - una frontiera a cui NON si riesce mai ad
// arrivare non accumula strike e resta viva per sempre. Osservato sulla
// campagna ODOM: 1-2 frontiere residue tenute da un drone fermo, gli altri due
// senza griglie assegnate che si arrendono dopo 13 tentativi a vuoto, e la run
// che brucia fino al tetto dei 470 s (3 run su 5).
//
// Il segnale usato qui e' piu' forte della distanza: 72 s senza alcun guadagno
// di volume. L'archiviazione e' reversibile - searchFrontiers rimette in gioco
// le dormienti appena la mappa cambia nel loro riquadro - quindi non si perde
// spazio esplorabile se arrivano informazioni nuove.
// Frontiere senza un punto di vista raggiungibile dal velivolo.
//
// Il ramo "No reachable frontier found from assigned grid center" di
// planExploreMotion fallisce PRIMA di scegliere una frontiera, quindi non
// registra nessun tentativo in visit_strikes_: la frontiera irraggiungibile non
// viene mai archiviata ne' dal conteggio dei tentativi ne' dal guardiano dei
// progressi (banHeldFrontiers svuota visit_strikes_, che qui e' vuoto).
// Misurato nella run BASE_LIO 4 di office_small: UNA frontiera rimasta, 727
// pianificazioni fallite, 638 s di volo invece di ~100.
// L'FSM chiama questa funzione dopo una lunga serie di fallimenti consecutivi:
// si archiviano le frontiere il cui punto di vista non e' raggiungibile da dove
// il drone si trova adesso.
int C2ExplorationManager::banUnreachableFrontiers(const Vector3d& pos) {
  int queued = 0;
  const size_t n = std::min(ed_->points_.size(), ed_->averages_.size());
  // Qui si decide di ARCHIVIARE: un tempo scaduto non deve valere come
  // "irraggiungibile", quindi la ricerca ha piu' tempo del solito.
  const double normale = ViewNode::astar_->max_search_time_;
  ViewNode::astar_->max_search_time_ = std::max(normale, 0.15);
  for (size_t i = 0; i < n; ++i) {
    vector<Eigen::Vector3d> path;
    const double cost =
        ViewNode::computeCost(pos, ed_->points_[i], 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    if (ViewNode::isUnreachableCost(cost)) {
      pending_bans_.push_back(ed_->averages_[i]);
      ++queued;
    }
  }
  ViewNode::astar_->max_search_time_ = normale;
  return queued;
}

int C2ExplorationManager::banHeldFrontiers() {
  int queued = 0;
  for (const auto& entry : visit_strikes_) {
    pending_bans_.push_back(entry.second.avg);
    ++queued;
  }
  visit_strikes_.clear();
  return queued;
}

// Prossima tappa verso casa, con degrado graduale.
//
// Il rientro puntava direttamente alla partenza e, se la ricerca di percorso
// falliva, non si muoveva affatto: su 10 rientri ne arrivavano 5, gli altri
// scadevano dopo un minuto e atterravano dov'erano. Un rientro o riesce del
// tutto o non parte, ed e' il modo peggiore di fallire.
//
// Qui si tenta prima casa; se non c'e' via, si cercano punti intermedi sulla
// congiungente, dal piu' avanzato al piu' arretrato, e si prende il primo
// raggiungibile. Il drone si avvicina a tappe invece di rinunciare, e a ogni
// ciclo riprova da capo - quindi man mano che la mappa si completa e che si
// sposta, la strada che prima non c'era puo' comparire.
bool C2ExplorationManager::findReturnWaypoint(
    const Vector3d& pos, const Vector3d& home, Vector3d& out) {
  auto raggiungibile = [&](const Vector3d& meta) {
    if (!sdf_map_->isInBox(meta)) return false;
    if (sdf_map_->getInflateOccupancy(meta) == 1) return false;
    return planner_manager_->path_finder_->searchPatient(pos, meta, 0.15, true) == Astar::REACH_END;
  };

  if (raggiungibile(home)) { out = home; return true; }

  const Vector3d dir = home - pos;
  const double dist = dir.norm();
  if (dist < 1e-3) { out = home; return true; }
  // Tappe dall'80% al 20% del tragitto: sotto il 20% il guadagno non vale il
  // ciclo di pianificazione.
  for (double f = 0.8; f >= 0.19; f -= 0.2) {
    Vector3d cand = pos + dir * f;
    cand.z() = home.z();
    if (raggiungibile(cand)) {
      out = cand;
      return true;
    }
  }
  return false;
}

int C2ExplorationManager::updateFrontierStruct(const Eigen::Vector3d& pos) {

  auto t1 = node_->now();
  auto t2 = t1;
  ed_->views_.clear();
  ed_->views1_.clear();
  ed_->views2_.clear();

  // Archiviazione differita delle frontiere inosservabili.
  //
  // La decisione viene presa in planExploreMotion, ma NON puo' essere applicata
  // li': rimuovere un elemento da frontiers_ dal percorso di pianificazione
  // invalida i riferimenti sotto i piedi di chi sta iterando la stessa lista in
  // un altro thread. E' successo davvero - 2 segfault su 3 run, con lo stack in
  // getPathForTour -> searchPath -> free() -> malloc_printerr, cioe' heap
  // corrotto. Qui invece siamo nella funzione che gia' ricostruisce le liste di
  // frontiere, serializzata come tutte le altre mutazioni.
  for (const auto& avg : pending_bans_) {
    if (frontier_finder_->banFrontier(avg))
      RCLCPP_WARN(node_->get_logger(),
          "Frontiera a (%.2f %.2f %.2f) archiviata: nessun progresso da dove il velivolo puo' "
          "arrivare",
          avg[0], avg[1], avg[2]);
  }
  pending_bans_.clear();

  // Punti di vista diventati irraggiungibili con l'aggiornamento della mappa:
  // vedi FrontierFinder::revalidateViewpoints (causa dei blocchi "No path to
  // next viewpoint").
  const int rese_dormienti = frontier_finder_->revalidateViewpoints();
  if (rese_dormienti > 0)
    RCLCPP_WARN(node_->get_logger(),
        "[PuntiDiVista] %d frontiere senza piu' un punto di vista raggiungibile: archiviate",
        rese_dormienti);

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();

  double frontier_time = (node_->now() - t1).seconds();
  t1 = node_->now();

  // Find viewpoints (x,y,z,yaw) for all clusters; find the informative ones
  frontier_finder_->computeFrontiersToVisit();

  // Retrieve the updated info
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);

  frontier_finder_->getTopViewpointsInfo(
      pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->viewpoints_);

  for (int i = 0; i < ed_->points_.size(); ++i) {
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

    vector<Vector3d> v1, v2;
    frontier_finder_->percep_utils_->setPose(ed_->points_[i], ed_->yaws_[i]);
    frontier_finder_->percep_utils_->getFOV(v1, v2);
    ed_->views1_.insert(ed_->views1_.end(), v1.begin(), v1.end());
    ed_->views2_.insert(ed_->views2_.end(), v2.begin(), v2.end());
  }

  if (ed_->frontiers_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No coverable frontier.");
    return 0;
  }

  double view_time = (node_->now() - t1).seconds();

  t1 = node_->now();
  frontier_finder_->updateFrontierCostMatrix();

  double mat_time = (node_->now() - t1).seconds();
  double total_time = frontier_time + view_time + mat_time;
  RCLCPP_INFO(node_->get_logger(),
      "\033[1m\033[38;5;214m[Drone %d] frontier t: %lf, viewpoint t: %lf, mat t: %lf, total "
      "t: %lf\033[0m",
      ep_->drone_id_, frontier_time, view_time, mat_time, total_time);

  return ed_->frontiers_.size();
}

void C2ExplorationManager::findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids) {
  auto t1 = node_->now();

  // Select nearby drones according to their states' stamp
  vector<Eigen::Vector3d> positions = { cur_pos };
  // vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0) };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // Partitioning-based tour planning
  vector<int> ego_ids;  // hgrid sequence.
  vector<vector<int>> other_ids;
  // Coarse-grained selection of which grids to visit and in what order.
  if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids)) {
    grid_ids = {};
    return;
  }
  grid_ids = ego_ids;

  double grid_time = (node_->now() - t1).seconds();

  // Frontier-based single drone tour planning
  // Restrict frontier within the first visited grid
  t1 = node_->now();

  vector<int> ftr_ids;
  bool used_center_hull = false;
  bool used_center_grid = false;
  int best_task_idx = -1;

  // If current task token is a split hull, frontier tour must use frontiers in that hull.
  auto& ego_state = ed_->swarm_state_[ep_->drone_id_ - 1];
  if (ed_->region_tour_.size() > 1 && !ego_state.center_positions_.empty()) {
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(ego_state.center_positions_.size()); ++i) {
      const double d2 = (ego_state.center_positions_[i] - ed_->region_tour_[1]).squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        best_task_idx = i;
      }
    }

    if (best_task_idx >= 0 && best_task_idx < static_cast<int>(ego_state.center_hulls_.size()) &&
        !ego_state.center_hulls_[best_task_idx].empty()) {
      const auto& task_hull = ego_state.center_hulls_[best_task_idx];
      for (int i = 0; i < static_cast<int>(ed_->averages_.size()); ++i) {
        if (pointInsideConvexHull2D(task_hull, ed_->averages_[i])) {
          ftr_ids.push_back(i);
        }
      }
      used_center_hull = true;
    }
  }

  if (!used_center_hull && ed_->region_tour_.size() > 1) {
    int center_grid_id = -1;
    if (hgrid_->getGridIdByCenterPos(ed_->region_tour_[1], center_grid_id)) {
      hgrid_->getFrontiersInGrid(vector<int>{ center_grid_id }, ftr_ids);
      used_center_grid = true;
    }
  }
  // For hull task, keep hull-only frontier set. For grid task, fallback to grid-level collection.
  if (!used_center_hull && ftr_ids.empty()) {
    // Con le strisce: frontiere di TUTTE le celle proprie. getFrontiersInGrid guarda solo la
    // prima dell'elenco, e quasi sempre restituiva zero: allora planExploreMotion prendeva la
    // frontiera piu' vicina FRA TUTTE e il drone usciva dalla sua striscia (a tre droni su
    // cubicle_office, "Find frontier tour, 0 involved" 132 volte e droni che si
    // attraversavano la mappa a vicenda). Con un drone solo resta il comportamento validato.
    if (strisce_pronte_)
      hgrid_->getFrontiersInGrids(ego_ids, ftr_ids);
    else
      hgrid_->getFrontiersInGrid(ego_ids, ftr_ids);
  }
  RCLCPP_INFO(node_->get_logger(),
      "\033[1m\033[38;5;117mFind frontier tour, %d involved------------\033[0m",
      (int)ftr_ids.size());
  if (used_center_hull) {
    RCLCPP_INFO(node_->get_logger(), "[Hull->Frontier] hull-aligned frontier extraction enabled");
  }
  if (used_center_grid) {
    RCLCPP_INFO(node_->get_logger(), "[Grid->Frontier] center-aligned frontier extraction enabled");
  }

  // Consider next grid in frontier tour planning
  Eigen::Vector3d grid_pos;
  double grid_yaw;
  vector<Eigen::Vector3d> grid_pos_vec;
  if (ed_->region_tour_.size() > 1) {
    grid_pos_vec = { ed_->region_tour_[1] };
  } else if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
    grid_pos_vec = { grid_pos };
  }

  if (!ftr_ids.empty()) {
    findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
  } else {
    // No frontier; keep frontier_ids empty.
    frontier_ids.clear();
  }
  double ftr_time = (node_->now() - t1).seconds();
  RCLCPP_INFO(node_->get_logger(), "Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
}

void C2ExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && rclcpp::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Degenerate case (start essentially coincides with goal, e.g. right after
  // takeoff before the drone has moved): upstream's own "at least three
  // points" guard below assumes short_tour.size() reached 2, but a
  // single-point A* path (start==goal) never grows past 1. Left unguarded,
  // this reaches PolynomialTraj::waypointsTraj with a single waypoint and
  // segfaults (0-length duration/derivative vectors it doesn't expect).
  if (short_tour.size() == 1) {
    // Nudge rather than duplicate: an exact-duplicate second point would
    // give planExploreTraj a zero-length/zero-duration segment, which hangs
    // (dt=0 makes its "for (ts=0; ts<=duration; ts+=dt)" sampling loop never
    // advance) rather than crashing.
    short_tour.push_back(short_tour.front() + Eigen::Vector3d(0, 0, 0.05));
  }

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void C2ExplorationManager::allocateTasks(
    const AllocationRequest& request, AllocationResult& result) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  const auto& positions = request.drone_positions;
  const auto& drone_ids = request.drone_ids;
  const auto& grid_ids = request.grid_ids;
  const auto* drone_states = request.drone_states;
  const auto& blocked_center_node_ids = request.blocked_center_node_ids;
  const auto& blocked_center_hulls = request.blocked_center_hulls;
  auto& all_centers = result.centers;
  auto& all_center_hulls = result.center_hulls;

  all_centers.clear();
  const int drone_num = positions.size();
  all_centers.resize(drone_num);
  all_center_hulls.clear();
  all_center_hulls.resize(drone_num);

  if (grid_ids.empty() || drone_num == 0) {
    return;
  }

  AllocationCandidateSet candidates;
  if (!buildAllocationCandidateSet(request, candidates)) {
    return;
  }

  const int full_center_num = candidates.full_center_num;
  Eigen::MatrixXd use_mat = candidates.mat;
  std::vector<int> use_center_grid_ids = candidates.center_grid_ids;
  std::vector<Eigen::Vector3d> use_center_positions = candidates.center_positions;
  std::vector<vector<Vector3d>> use_center_hulls = candidates.center_hulls;
  std::vector<int> use_center_ids = candidates.center_ids;
  std::vector<int> use_center_types = candidates.center_types;
  int use_center_num = static_cast<int>(use_center_positions.size());
  int blocked_filtered_num = use_center_num;

  // Restrict MeetingOpt candidates to centers that still correspond to participant-owned tasks.
  // Split tasks match by hull; normal tasks match by center id / grid id.
  if (drone_states != nullptr &&
      drone_states->size() == static_cast<size_t>(drone_num) && !use_center_positions.empty()) {
    std::vector<int> owned_center_idx;
    filterMeetingOptCentersByParticipantTasks(
        hgrid_, *drone_states, use_center_grid_ids, use_center_positions, use_center_ids,
        use_center_types, use_center_hulls, owned_center_idx);

    // TUTTO LO SCIAME ALL'INCONTRO (ottobre 2026). Il filtro serve a non ridistribuire compiti
    // di droni ASSENTI dall'incontro (collegamento limitato): si possono scambiare solo quelli
    // dei partecipanti. Ma riconosce i compiti dei partecipanti solo se i loro id di cella
    // combaciano con i centri di questo drone, e in volo spesso non combaciano: a tre droni
    // tutti collegati su office_small, 30 incontri su 33 finivano qui e nessuno rialloca
    // nulla, mentre i tre droni rivendicavano 13-14 celle su 16 a testa (374 avvisi di
    // "lavoro duplicato" in una run). Se all'incontro c'e' l'INTERO sciame non ci sono compiti
    // di terzi da proteggere: si ripartiscono tutti i centri candidati.
    if (owned_center_idx.empty() && drone_num == static_cast<int>(ep_->drone_num_)) {
      owned_center_idx.resize(use_center_positions.size());
      for (int i = 0; i < static_cast<int>(owned_center_idx.size()); ++i) owned_center_idx[i] = i;
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[MeetingOpt] filtro sui compiti vuoto ma all'incontro c'e' tutto lo sciame: "
          "ripartisco tutti i " << use_center_positions.size() << " centri");
    }
    if (owned_center_idx.empty()) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[MeetingOpt] No candidate centers survive participant-task filtering, pooled_centers="
              << use_center_positions.size());
      return;
    }

    if (owned_center_idx.size() < use_center_positions.size()) {
      // Keep the cost matrix rows/cols aligned with the filtered center metadata.
      keepCenterSubset(owned_center_idx, drone_num, use_mat, use_center_grid_ids,
          use_center_positions, use_center_hulls, use_center_ids, use_center_types);
    }
    blocked_filtered_num = static_cast<int>(use_center_positions.size());

    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
        "[MeetingOpt] participant-task center filter: pooled=" << full_center_num
                                                            << " blocked_filtered="
                                                            << blocked_filtered_num
                                                            << " owned="
                                                            << use_center_positions.size());
    use_center_num = static_cast<int>(use_center_positions.size());
  }

  // Drop centers that are already blocked by another participant's center id or split-task hull.
  if (!blocked_center_node_ids.empty() || !blocked_center_hulls.empty()) {
    std::vector<int> sel_center_idx;
    sel_center_idx.reserve(use_center_num);
    unordered_set<int> blocked_node_id_set(
        blocked_center_node_ids.begin(), blocked_center_node_ids.end());
    for (int i = 0; i < use_center_num; ++i) {
      const int cid = (i < static_cast<int>(use_center_ids.size())) ? use_center_ids[i] : -1;
      const bool has_hull =
          (i < static_cast<int>(use_center_hulls.size()) && !use_center_hulls[i].empty());
      if (!has_hull && cid >= 0 && blocked_node_id_set.find(cid) != blocked_node_id_set.end())
        continue;
      if (candidateBlockedByHull(use_center_positions[i], blocked_center_hulls)) continue;
      sel_center_idx.push_back(i);
    }
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
        "[MeetingOpt] center filter: full=" << full_center_num << " blocked_node_ids="
                                             << blocked_center_node_ids.size() << " blocked_hulls="
                                             << blocked_center_hulls.size()
                                             << " remain=" << sel_center_idx.size());
    if (sel_center_idx.empty()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
          "[MeetingOpt] No candidate centers after blocked-center filtering");
      return;
    }
    if (sel_center_idx.size() < static_cast<size_t>(use_center_num)) {
      // Apply the same subset to both the matrix and the center metadata before ACVRP.
      keepCenterSubset(sel_center_idx, drone_num, use_mat, use_center_grid_ids,
          use_center_positions, use_center_hulls, use_center_ids, use_center_types);
    }
    use_center_num = static_cast<int>(use_center_positions.size());
  } else {
    blocked_filtered_num = use_center_num;
  }

  if (use_center_positions.empty()) {
    return;
  }

  // Anti-oscillation on center ownership:
  // Penalize reassigning a center to a non-owner, optionally reward owner retention.
  if (ep_->center_switch_penalty_ > 1e-6 || ep_->center_stay_bonus_ > 1e-6) {
    vector<int> row_drone_ids(drone_num, -1);
    if (drone_ids.size() == static_cast<size_t>(drone_num)) {
      row_drone_ids = drone_ids;
    } else {
      for (int i = 0; i < drone_num; ++i) row_drone_ids[i] = i + 1;
    }

    unordered_map<int, int> center_owner;
    center_owner.reserve(use_center_ids.size() * 2);
    for (int did = 0; did < static_cast<int>(ed_->swarm_state_.size()); ++did) {
      const int drone_id = did + 1;
      const auto& state = ed_->swarm_state_[did];
      for (int i = 0; i < static_cast<int>(state.center_ids_.size()); ++i) {
        if (i < static_cast<int>(state.center_hulls_.size()) && !state.center_hulls_[i].empty()) {
          continue;
        }
        const int cid = state.center_ids_[i];
        if (cid < 0) continue;
        auto it = center_owner.find(cid);
        if (it == center_owner.end()) {
          center_owner[cid] = drone_id;
        } else if (it->second != drone_id) {
          it->second = -1;
        }
      }
    }

    int penalized_edges = 0;
    int rewarded_edges = 0;
    for (int i = 0; i < drone_num; ++i) {
      const int drone_id = row_drone_ids[i];
      if (drone_id <= 0) continue;
      for (int j = 0; j < use_center_num; ++j) {
        if (j >= static_cast<int>(use_center_ids.size())) continue;
        if (j < static_cast<int>(use_center_hulls.size()) && !use_center_hulls[j].empty()) continue;
        const int cid = use_center_ids[j];
        if (cid < 0) continue;
        auto it = center_owner.find(cid);
        if (it == center_owner.end()) continue;
        const int owner = it->second;
        if (owner <= 0) continue;

        const int row = 1 + i;
        const int col = 1 + drone_num + j;
        double adjusted = use_mat(row, col);
        if (owner == drone_id) {
          if (ep_->center_stay_bonus_ > 1e-6) {
            adjusted = std::max(0.0, adjusted - ep_->center_stay_bonus_);
            ++rewarded_edges;
          }
        } else {
          adjusted += ep_->center_switch_penalty_;
          ++penalized_edges;
        }
        use_mat(row, col) = adjusted;
      }
    }

    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
        "[MeetingOpt] ownership hysteresis applied: switch_penalty="
            << ep_->center_switch_penalty_ << " stay_bonus=" << ep_->center_stay_bonus_
            << " penalized_edges=" << penalized_edges << " rewarded_edges=" << rewarded_edges);
  }

  if (use_center_num == 1) {
    const auto& pt = use_center_positions.front();
    int best_idx = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < drone_num; ++i) {
      vector<Eigen::Vector3d> path;
      double cost =
          ViewNode::computeCost(positions[i], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      if (cost < best_cost) {
        best_cost = cost;
        best_idx = i;
      }
    }
    all_centers[best_idx] = { pt };
    all_center_hulls[best_idx] = { use_center_hulls.front() };
    return;
  }

  const int center_num = use_center_num;
  const int dimension = use_mat.rows();

  int capacity = 0;
  vector<int> demands(center_num, 0);
  if (ep_->prob_type_ == 2) {
    const double demand_scale = 0.1;
    const int base_demand = 1;

    int total_demand = 0;
    int max_demand = 0;

    unordered_map<int, int> grid_to_index;
    grid_to_index.reserve(grid_ids.size());
    for (int i = 0; i < static_cast<int>(grid_ids.size()); ++i) {
      grid_to_index[grid_ids[i]] = i;
    }

    vector<int> grid_demand(grid_ids.size(), 0);
    for (int i = 0; i < static_cast<int>(grid_ids.size()); ++i) {
      int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
      int d = 0;
      if (unum > 0) {
        d = (int)std::ceil(unum * demand_scale);
        d = std::max(d, 1);
      }
      d += base_demand;
      grid_demand[i] = d;
    }

    unordered_map<int, int> grid_center_counts;
    grid_center_counts.reserve(center_num);
    for (int gid : use_center_grid_ids) {
      grid_center_counts[gid] += 1;
    }

    for (int c = 0; c < center_num; ++c) {
      int gid = use_center_grid_ids[c];
      auto it = grid_to_index.find(gid);
      if (it == grid_to_index.end()) {
        demands[c] = 1;
        total_demand += demands[c];
        max_demand = std::max(max_demand, demands[c]);
        continue;
      }
      int gidx = it->second;
      int cnt = std::max(1, grid_center_counts[gid]);
      int per = (int)std::ceil(static_cast<double>(grid_demand[gidx]) / cnt);
      per = std::max(per, 1);
      demands[c] = per;
      total_demand += per;
      max_demand = std::max(max_demand, per);
    }

    const double slack = 0.10;
    capacity = (int)std::ceil((double)total_demand / (double)drone_num * (1.0 + slack));
    capacity = std::max(capacity, max_demand);
    RCLCPP_WARN(node_->get_logger(), "ACVRP total_demand=%d, cap=%d, drones=%d", total_demand,
        capacity, drone_num);
  }

  ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : allocation\n";

  if (ep_->prob_type_ == 1)
    file << "TYPE : ATSP\n";
  else if (ep_->prob_type_ == 2)
    file << "TYPE : ACVRP\n";

  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

  if (ep_->prob_type_ == 2) {
    file << "CAPACITY : " + to_string(capacity) + "\n";
    file << "VEHICLES : " + to_string(drone_num) + "\n";
  }

  file << "EDGE_WEIGHT_SECTION\n";
  int sanitized_cost_cnt = 0;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = toLkhEdgeWeight(use_mat(i, j), sanitized_cost_cnt);
      file << int_cost << " ";
    }
    file << "\n";
  }
  if (sanitized_cost_cnt > 0) {
    RCLCPP_WARN(node_->get_logger(), "[MeetingOpt] Sanitized %d invalid/overflow cost entries before LKH",
        sanitized_cost_cnt);
  }

  if (ep_->prob_type_ == 2) {
    file << "DEMAND_SECTION\n";
    file << "1 0\n";
    for (int i = 0; i < drone_num; ++i) {
      file << to_string(i + 2) + " 0\n";
    }
    for (int i = 0; i < center_num; ++i) {
      file << to_string(i + 2 + drone_num) + " " + to_string(demands[i]) + "\n";
    }
    file << "DEPOT_SECTION\n";
    file << "1\n";
    file << "EOF";
  }

  file.close();

  file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
  if (ep_->prob_type_ == 1) {
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINMAX\n";
    file << "SEED = 1\n";
    file << "TRACE_LEVEL = 0\n";
  } else if (ep_->prob_type_ == 2) {
    file << "TRACE_LEVEL = 1\n";
    file << "SEED = 0\n";
  }
  file << "PRECISION = " << kLkhPrecision << "\n";
  file << "RUNS = 1\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  if (!callSolveTSP(tsp_client_node_, acvrp_client_, 3)) {
    RCLCPP_ERROR(node_->get_logger(), "Fail to solve ACVRP.");
    return;
  }

  ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  vector<vector<int>> tours;
  parseMultiTours(ids, drone_num, dimension, tours);

  for (const auto& tr : tours) {
    if (tr.empty()) continue;
    int drone_token = tr.front();
    if (drone_token <= 0 || drone_token > drone_num) continue;
    int drone_idx = drone_token - 1;
    unordered_set<int> seen;
    for (size_t i = 1; i < tr.size(); ++i) {
      int id = tr[i];
      int center_idx = id - 1 - drone_num;
      if (center_idx >= 0 && center_idx < static_cast<int>(use_center_positions.size())) {
        if (seen.insert(center_idx).second) {
          all_centers[drone_idx].push_back(use_center_positions[center_idx]);
          all_center_hulls[drone_idx].push_back(use_center_hulls[center_idx]);
        }
      }
    }
  }
}

bool C2ExplorationManager::buildAllocationCandidateSet(
    const AllocationRequest& request, AllocationCandidateSet& candidates) {
  vector<HGrid::HullTask> hull_tasks;
  collectAssignedHullTasks(hgrid_, request.drone_states, hull_tasks);

  if (!hull_tasks.empty()) {
    hgrid_->getAllocationCostMatrix(request.drone_positions, request.drone_velocities,
        request.grid_ids, candidates.mat, &hull_tasks);
  } else {
    hgrid_->getAllocationCostMatrix(
        request.drone_positions, request.drone_velocities, request.grid_ids, candidates.mat);
  }

  const auto& center_grid_ids = hgrid_->getLastCostMatrixCenterGridIds();
  const auto& center_positions = hgrid_->getLastCostMatrixCenterPositions();
  const auto& center_hulls = hgrid_->getLastCostMatrixCenterHulls();
  const auto& center_ids = hgrid_->getLastCostMatrixCenterIds();
  const auto& center_types = hgrid_->getLastCostMatrixCenterTypes();

  candidates.full_center_num = static_cast<int>(center_grid_ids.size());
  if (candidates.full_center_num == 0) {
    return false;
  }

  candidates.center_grid_ids.clear();
  candidates.center_positions.clear();
  candidates.center_hulls.clear();
  candidates.center_ids.clear();
  candidates.center_types.clear();
  candidates.center_grid_ids.reserve(candidates.full_center_num);
  candidates.center_positions.reserve(candidates.full_center_num);
  candidates.center_hulls.reserve(candidates.full_center_num);
  candidates.center_ids.reserve(candidates.full_center_num);
  candidates.center_types.reserve(candidates.full_center_num);

  for (int i = 0; i < candidates.full_center_num; ++i) {
    candidates.center_grid_ids.push_back(center_grid_ids[i]);
    candidates.center_positions.push_back(center_positions[i]);
    if (i < static_cast<int>(center_hulls.size())) {
      candidates.center_hulls.push_back(center_hulls[i]);
    } else {
      candidates.center_hulls.emplace_back();
    }
    if (i < static_cast<int>(center_ids.size())) {
      candidates.center_ids.push_back(center_ids[i]);
    } else {
      candidates.center_ids.push_back(-1);
    }
    if (i < static_cast<int>(center_types.size())) {
      candidates.center_types.push_back(center_types[i]);
    } else {
      candidates.center_types.push_back(HGrid::FREE_ACTIVE_CENTER);
    }
  }

  return true;
}

bool C2ExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init) {
  rclcpp::Clock clock(RCL_ROS_TIME);

  RCLCPP_INFO(node_->get_logger(), "\033[32mFind grid tour---------------\033[0m");

  auto t1 = node_->now();

  auto& state = ed_->swarm_state_[ep_->drone_id_ - 1];
  vector<int> grid_ids = state.grid_ids_;
  int pre_grid_size = grid_ids.size();

  // print grid_ids
  std::cout << "\033[1;96mgrid_ids: ";
  for (auto id : grid_ids) std::cout << id << " ";
  std::cout << "\033[0m" << std::endl;

  // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

  vector<int> first_ids, second_ids;
  hgrid_->inputFrontiers(ed_->averages_);

  hgrid_->updateGridData(ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_,
      first_ids, second_ids, positions[0]);
  const double tg_griglia = (node_->now() - t1).seconds();  // [TempiGiro]
  state.grid_ids_ = grid_ids;

  // ---- SPARTIZIONE A STRISCE (sciame, ottobre 2026) ----
  //
  // La ripartizione per vicinanza qui sotto (modalita' chiusa) non e' mai stata esclusiva
  // davvero: ogni drone si sconta del 25% le celle che gia' tiene e all'avvio le tiene tutte,
  // e con i droni a 3-4 m l'uno dall'altro le distanze dalle celle si equivalgono, quindi
  // ciascuno continua a ritenersi padrone delle stesse celle. A tre droni tutti collegati su
  // office_small: 374-632 avvisi di "lavoro duplicato" a run, esplorazione finita in 46-74 s
  // contro i 38 del drone singolo. E all'aperto (parco, campo) non era nemmeno attiva: li'
  // restava solo MeetingOpt, che in volo riassegna di rado. Qui la proprieta' di ogni cella
  // dipende SOLO dalla griglia (identica su tutti) e dalle partenze dei droni (dal launch):
  // le celle si ordinano lungo la linea delle partenze e si dividono in blocchi contigui di
  // pari numero, uno per drone nell'ordine delle partenze. Ogni drone calcola la stessa
  // spartizione, senza messaggi ne' isteresi, e la striscia non cambia mai. Chi ha finito
  // la propria striscia va sulla cella ancora da esplorare piu' vicina che nessun compagno
  // tiene (lo stesso ripiego della modalita' chiusa, sotto). partitioning.strisce:=false (o
  // nessuna partenza dal launch) torna alla ripartizione di prima.
  if (calcolaStrisce()) {
    // LAVORO = celle con frontiere ATTIVE, non le celle "rilevanti". Una cella resta
    // rilevante finche' vi resta spazio ignoto raggiungibile, anche se nessuna frontiera attiva
    // ci cade (le altre sono dormienti: punti di vista non validi o archiviate). Tenendo
    // quelle, il drone restava padrone di celle in cui non poteva fare nulla, trovava 0
    // frontiere nelle sue celle (~100 volte per drone in una run su cubicle_office) e
    // ripiegava sulla frontiera piu' vicina ovunque fosse, anche a 15 m: 22 s per cella contro
    // i 12 del drone singolo. Con le frontiere attive come lavoro, finita la propria striscia
    // scatta subito l'aiuto mirato qui sotto.
    std::vector<int> relevant;
    {
      std::vector<int> tutte;
      hgrid_->getAllIds(tutte);
      for (int g : tutte)
        if (hgrid_->frontiereInCella(g) > 0) relevant.push_back(g);
    }
    std::vector<int> mine;
    for (int g : relevant) {
      const auto it = strisce_.find(g);
      if (it != strisce_.end() && it->second == ep_->drone_id_) mine.push_back(g);
    }
    // AIUTO A STRISCIA FINITA: si prende la cella con frontiere attive piu' vicina fra quelle
    // che nessun compagno tiene, e solo se fra tutti i compagni con uno stato recente questo
    // drone e' il PIU' VICINO a lei (calcolo identico su ognuno: un aiutante per cella). Mai
    // una cella gia' tenuta da un compagno. Prima chi finiva la propria striscia prendeva la
    // libera piu' vicina e, se non ce n'erano, condivideva quella di un compagno: a tre droni su
    // cubicle_office due aiutanti sceglievano la stessa cella nello stesso istante, cambiavano
    // bersaglio ogni secondo e si sono trovati a 0.21 m l'uno dall'altro (run 1153). Senza
    // nulla da prendere il drone resta senza celle e l'FSM conclude l'esplorazione (visita
    // della propria striscia), vedi il ramo IDLE.
    if (mine.empty() && !relevant.empty()) {
      std::set<int> prese_dai_pari;
      std::vector<Eigen::Vector3d> altri;
      const double ora = node_->now().seconds();
      for (int k = 0; k < static_cast<int>(ed_->swarm_state_.size()); ++k) {
        if (k + 1 == ep_->drone_id_) continue;
        const auto& ps = ed_->swarm_state_[k];
        if (ps.recv_stamp_ <= 0.0 || ora - ps.recv_stamp_ > std::max(fp_state_fresh_, 1.0)) continue;
        for (int g : ps.grid_ids_) prese_dai_pari.insert(g);
        // Per il "piu' vicino" contano solo i compagni ancora in esplorazione (con celle): chi
        // ha concluso svuota la sua lista e non prende piu' celle.
        if (!ps.grid_ids_.empty()) altri.push_back(ps.pos_);
      }
      int libera = -1;
      double d_libera = std::numeric_limits<double>::infinity();
      for (int g : relevant) {
        if (prese_dai_pari.count(g)) continue;
        Eigen::Vector3d c;
        if (!hgrid_->getGridCenter(g, c)) continue;
        const double d = (c - positions[0]).head<2>().norm();
        bool piu_vicino = true;
        for (const auto& a : altri)
          if ((c - a).head<2>().norm() < d) { piu_vicino = false; break; }
        if (piu_vicino && d < d_libera) { d_libera = d; libera = g; }
      }
      if (libera >= 0) {
        mine.push_back(libera);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), clock, 2000,
            "[Strisce] Drone %d: striscia finita, aiuto sulla cella %d a %.1f m", ep_->drone_id_,
            libera, d_libera);
      }
    }
    // Nessuna frontiera attiva nella propria striscia e nulla da aiutare, ma spazio ancora
    // IGNOTO nella striscia: ci si va, le frontiere compariranno strada facendo. Solo quando
    // non resta nemmeno quello la lista resta vuota e l'FSM conclude. Senza questo passo, a
    // inizio missione il drone 2 (la cui striscia non aveva ancora frontiere attive) ha chiuso
    // l'esplorazione dopo 7 s ed e' rimasto a terra per tutto il resto (run 1234).
    if (mine.empty()) {
      std::vector<int> rilevanti;
      hgrid_->getRelevantIds(rilevanti);
      for (int g : rilevanti) {
        const auto it = strisce_.find(g);
        if (it != strisce_.end() && it->second == ep_->drone_id_) mine.push_back(g);
      }
    }
    RCLCPP_INFO_THROTTLE(node_->get_logger(), clock, 3000,
        "[Strisce] Drone %d: %zu celle con frontiere attive, tengo %zu", ep_->drone_id_,
        relevant.size(), mine.size());
    grid_ids = mine;
    state.grid_ids_ = grid_ids;
    owned_last_.clear();
    owned_last_.insert(grid_ids.begin(), grid_ids.end());
  } else
  // ---- MODALITA' CHIUSA: ripartizione esclusiva e decentralizzata ----
  //
  // Difetto misurato senza questo blocco: tutti e tre i droni tenevano
  // contemporaneamente quasi tutte le celle (14, 16 e 15 su 16) per l'intera
  // run, lavorando sovrapposti, e quando la propria copia si svuotava nessuno
  // gliene passava altre: si ritiravano lasciando la mappa incompleta.
  //
  // Il principio evita una negoziazione. Due droni in contatto conoscono
  // entrambi le posizioni, quindi possono calcolare LA STESSA ripartizione in
  // modo indipendente e arrivare allo stesso risultato: l'esclusivita' diventa
  // una proprieta' del calcolo invece che di un protocollo, e non ci sono
  // messaggi da perdere ne' stati da riconciliare. Ogni cella va al drone in
  // contatto piu' vicino al suo centro, con l'id come spareggio deterministico.
  // Isolato, un drone non vede pari e tiene tutto: corretto, perche' per quanto
  // ne sa e' solo. Fedele allo spirito decentralizzato: si decide solo con chi
  // e' dentro r_comm.
  if (closed_mode_) {
    // L'insieme da ripartire e' quello delle celle RILEVANTI, non l'unione di
    // quelle gia' possedute.
    //
    // Prima filtravo il possesso: se l'insieme posseduto si svuotava non restava
    // nulla da ripartire, e le celle che diventano rilevanti PIU' TARDI - perche'
    // una frontiera vi compare dopo - non entravano mai nel giro. Misurato: i
    // droni ricevevano zero celle mentre ne esistevano 7 rilevanti, 5 con
    // frontiere, e si arrendevano lasciando la mappa incompleta.
    // Cosi' invece la proprieta' e' una funzione pura di (celle rilevanti,
    // posizioni dei pari in contatto, possesso precedente per l'isteresi):
    // deterministica, ricalcolata a ogni ciclo, e si auto-ripara.
    std::vector<int> relevant;
    hgrid_->getRelevantIds(relevant);
    // Dominio della ripartizione: la griglia INTERA, non le sole celle
    // localmente rilevanti. La rilevanza dipende dalla mappa di ciascun drone,
    // quindi partizionandoci sopra i tre partizionavano insiemi diversi e
    // l'esclusivita' non poteva valere. La griglia e' statica e identica per
    // tutti: come dominio rende il calcolo riproducibile su ogni drone.
    // Cio' che e' rilevante torna a filtrare il LAVORO (vedi 'mine' piu' sotto),
    // non la proprieta'.
    std::vector<int> domain;
    hgrid_->getAllIds(domain);
    const std::set<int> rel_set(relevant.begin(), relevant.end());

    // STABILITA': la ripartizione non si ricalcola a ogni pianificazione.
    // Ricalcolandola sempre, l'insieme assegnato puo' cambiare a ogni ciclo e il
    // drone insegue un bersaglio che si sposta - misurato: 762 cicli di
    // pianificazione su un drone contro 92 di un altro, squilibrio 8.3 e il
    // watchdog scattato 12 volte. Si tiene l'assegnazione precedente per almeno
    // repartition_interval, filtrata su cio' che e' ancora rilevante; si
    // ricalcola subito, invece, se si e' rimasti senza celle - la' l'attesa
    // costerebbe lavoro fermo.
    // SI RINEGOZIA QUANDO SI INCONTRA QUALCUNO, NON QUANDO LO SI PERDE.
    //
    // E' l'asimmetria che rende il coordinamento compatibile con un collegamento
    // intermittente - il caso reale di un link ottico, che un muro interrompe.
    // Misurato su una run: 165 interruzioni/ripristini di collegamento e 165
    // sovrapposizioni di celle. Il meccanismo: perdendo di vista un compagno il
    // drone ricalcolava la ripartizione fra i soli rimasti e, con la regola
    // "isolato tengo tutto", si prendeva anche le celle di chi non vedeva piu' -
    // che nel frattempo ci stava lavorando.
    // Ma perdere il contatto NON e' un'informazione sulle celle altrui: e'
    // assenza di informazione. Quindi:
    //   * compare un pari mai visto in questo assetto -> si rinegozia, c'e'
    //     informazione nuova da dividere;
    //   * scompare un pari -> si tiene il proprio e non si tocca il suo;
    //   * si resta senza celle -> si ricalcola comunque, perche' aspettare
    //     costerebbe lavoro fermo.
    std::set<int> peers_now;
    {
      const double now_fresh = node_->now().seconds();
      for (int id : ed_->neighbor_ids_) {
        if (id < 1 || id > static_cast<int>(ed_->swarm_state_.size()) || id == ep_->drone_id_)
          continue;
        if (fp_state_fresh_ > 1e-6 &&
            now_fresh - ed_->swarm_state_[id - 1].recv_stamp_ > fp_state_fresh_)
          continue;
        peers_now.insert(id);
      }
    }
    bool nuovo_incontro = false;
    for (int id : peers_now)
      if (!peers_last_.count(id)) nuovo_incontro = true;

    const double now_rep = node_->now().seconds();
    const bool tieni = !owned_last_.empty() && !nuovo_incontro &&
        now_rep - last_repartition_ < repartition_interval_;
    if (tieni) {
      std::set<int> rel(relevant.begin(), relevant.end());
      std::vector<int> keep;
      for (int g : owned_last_)
        if (rel.count(g)) keep.push_back(g);
      if (!keep.empty()) {
        grid_ids = keep;
        state.grid_ids_ = grid_ids;
        owned_last_.clear();
        owned_last_.insert(keep.begin(), keep.end());
        // salta il ricalcolo di questo ciclo
        relevant.clear();
      }
    }
    if (!relevant.empty()) {
      last_repartition_ = now_rep;
      if (nuovo_incontro)
        RCLCPP_INFO(node_->get_logger(),
            "[Incontro] Drone %d rinegozia: %zu pari in vista", ep_->drone_id_, peers_now.size());
    }
    peers_last_ = peers_now;

    std::vector<std::pair<int, Eigen::Vector3d>> peers;
    peers.emplace_back(ep_->drone_id_, positions[0]);
    const double now_s = node_->now().seconds();
    for (int id : ed_->neighbor_ids_) {
      if (id < 1 || id > static_cast<int>(ed_->swarm_state_.size()) || id == ep_->drone_id_)
        continue;
      const auto& ps = ed_->swarm_state_[id - 1];
      // Solo stati freschi: una posizione vecchia porterebbe a cedere celle a un
      // drone che non e' piu' li'.
      if (fp_state_fresh_ > 1e-6 && now_s - ps.recv_stamp_ > fp_state_fresh_) continue;
      peers.emplace_back(id, ps.pos_);
    }

    if (relevant.empty()) {
      // assegnazione mantenuta da questo ciclo, nulla da fare
    } else if (peers.size() == 1) {
      // Isolato: prende tutto cio' che e' rilevante. E' la risposta corretta,
      // perche' per quanto ne sa e' solo.
      grid_ids = relevant;
    } else if (!relevant.empty()) {
      auto distTo = [&](int pi, int g) {
        Eigen::Vector3d c;
        if (!hgrid_->getGridCenter(g, c)) return std::numeric_limits<double>::infinity();
        return (peers[pi].second - c).norm();
      };

      // Proprietario uscente di ogni cella, visto allo STESSO modo da tutti.
      //
      // L'isteresi era applicata solo a se' stessi: ogni drone si scontava del
      // 25% le celle che gia' teneva, ma non riconosceva lo stesso sconto ai
      // vicini per le loro. Cosi' A, che tiene g, se la aggiudicava grazie allo
      // sconto; B calcolava la stessa ripartizione col costo pieno e, se anche
      // solo un po' piu' vicino, se la aggiudicava a sua volta. Entrambi
      // rivendicavano g, e siccome lo sconto ne sposta poche alla volta la
      // sovrapposizione risultava di 1-2 celle - misurate 595 volte in 4 run,
      // con tutte le coppie di droni coinvolte e a connettivita' piena, quindi
      // NON per frammentazione radio.
      // PROVATA E SCARTATA (campagna SIM, 4 run): applicare lo sconto a chiunque
      // sia il proprietario NON ha eliminato le sovrapposizioni - sono passate da
      // 595 a 747 - e ha peggiorato le durate, tutte e quattro le run al tetto di
      // tempo (407-437 s contro 137-437). Rendere le celle piu' appiccicose al
      // proprietario riduce il ribilanciamento senza rendere coerenti i calcoli.
      //
      // Il motivo per cui non poteva funzionare: la ripartizione opera su
      // `relevant`, e la rilevanza di una cella e' calcolata sulla mappa LOCALE
      // (raggiungibile e ancora esplorabile). Le mappe sono condivise ma non
      // istantaneamente identiche, quindi due droni partizionano insiemi di celle
      // diversi. Rendere simmetrica l'isteresi non basta: bisognerebbe rendere
      // concorde l'insieme di partenza, oppure far decidere l'assegnazione a uno
      // solo e comunicarla - cioe' cambiare protocollo, non un coefficiente.
      // Il calcolo di incumbent resta qui perche' serve a quella strada.
      std::map<int, int> incumbent;
      for (int g : owned_last_) incumbent[g] = ep_->drone_id_;
      for (const auto& pr : peers) {
        if (pr.first == ep_->drone_id_) continue;
        if (pr.first < 1 || pr.first > static_cast<int>(ed_->swarm_state_.size())) continue;
        for (int g : ed_->swarm_state_[pr.first - 1].grid_ids_) {
          auto it_inc = incumbent.find(g);
          if (it_inc == incumbent.end() || pr.first < it_inc->second) incumbent[g] = pr.first;
        }
      }

      // PROVATO E SCARTATO - pesare le celle per le frontiere che contengono,
      // invece di contarle tutte uguali, nel bilanciatore al punto 2.
      //
      // Il ragionamento era buono e resta valido: il dominio e' tutta la
      // griglia, a meta' run gran parte delle celle e' gia' esplorata, e
      // dividere 25 celle in 9/8/8 non dice nulla su chi ha ancora lavoro. Se
      // le poche celle vive cadono nella quota di un drone, gli altri restano a
      // mani vuote e vanno in IDLE - ed e' il pezzo piu' grosso del tempo fermo
      // misurato, il 35% e' "ho finito, aspetto i compagni".
      // Campagna da 4 run (3 valutate, la quarta interrotta perche' l'esito era
      // gia' deciso): finiti=0/3 in TUTTE, durate al tetto di 700 s (677/618/677)
      // contro 3/3 e 257-637 s. Zero conclusioni su nove attese.
      // Il perche' e' misurato: l'insieme di celle tenuto da ogni drone cambia
      // nel 67-83% dei ricalcoli. Il peso dipende dalle frontiere, che cambiano
      // a ogni aggiornamento di mappa; il bilanciatore sposta celle diverse ogni
      // volta e il drone insegue un'assegnazione che si muove. Non esaurisce mai
      // il proprio settore, quindi non conclude mai. Con il collegamento ottico
      // e' peggio: ogni nuovo incontro forza un ricalcolo, e prima ne usciva un
      // risultato quasi identico, ora uno diverso.
      // Morale per il prossimo tentativo: lo squilibrio NON si corregge rendendo
      // la ripartizione piu' reattiva. Serve qualcosa che non destabilizzi
      // l'assegnazione - p.es. dare lavoro a chi e' rimasto a secco senza
      // toccare l'assegnazione degli altri.

      // 1) assegnazione per vicinanza, con isteresi sul proprietario uscente.
      std::map<int, int> owner;
      std::vector<int> load(peers.size(), 0);
      for (int g : domain) {
        int best = 0;
        double best_cost = std::numeric_limits<double>::infinity();
        const auto it_inc = incumbent.find(g);
        const int inc_id = (it_inc == incumbent.end()) ? -1 : it_inc->second;
        (void)inc_id;  // vedi la nota sull'isteresi simmetrica, provata e scartata
        for (size_t i = 0; i < peers.size(); ++i) {
          double cost = distTo(int(i), g);
          if (peers[i].first == ep_->drone_id_ && owned_last_.count(g)) cost *= 0.75;
          if (cost < best_cost - 1e-6 ||
              (std::fabs(cost - best_cost) <= 1e-6 && peers[i].first < peers[best].first)) {
            best_cost = cost;
            best = int(i);
          }
        }
        owner[g] = best;
        ++load[best];
      }

      // 2) ribilanciamento: senza, un drone che ha finito il suo settore resta
      // fermo, perche' su ogni cella rimasta c'e' sempre qualcuno piu' vicino di
      // lui per quanto carico sia. Si sposta dal piu' carico al piu' scarico la
      // cella che costa meno al ricevente, finche' la differenza scende a uno.
      for (int iter = 0; iter < 64; ++iter) {
        int hi = 0, lo = 0;
        for (size_t i = 0; i < peers.size(); ++i) {
          if (load[i] > load[hi]) hi = int(i);
          if (load[i] < load[lo]) lo = int(i);
        }
        if (load[hi] - load[lo] <= 1) break;
        int move_g = -1;
        double best_extra = std::numeric_limits<double>::infinity();
        for (const auto& kv : owner) {
          if (kv.second != hi) continue;
          const double extra = distTo(lo, kv.first) - distTo(hi, kv.first);
          if (extra < best_extra) {
            best_extra = extra;
            move_g = kv.first;
          }
        }
        if (move_g < 0) break;
        owner[move_g] = lo;
        --load[hi];
        ++load[lo];
      }

      std::vector<int> mine;
      for (const auto& kv : owner)
        if (peers[kv.second].first == ep_->drone_id_ && rel_set.count(kv.first))
          mine.push_back(kv.first);

      // RIPIEGO: mai restare a mani vuote se del lavoro esiste.
      //
      // La ripartizione equalizza il NUMERO di celle sulla griglia INTERA, quindi
      // la proprieta' si spalma su celle in gran parte gia' esplorate. Le poche
      // celle ancora vive stanno vicine fra loro e possono finire tutte nella
      // quota di uno o due droni: il terzo si ritrova proprietario solo di celle
      // morte, il filtro su rel_set qui sopra lo lascia con zero, planExploreMotion
      // torna NO_GRID e l'FSM va in IDLE.
      // Misurato su office_small, 4 run: succede in 118 ripartizioni su 555 (21%),
      // e in 51 di quelle c'erano 3 o 4 celle rilevanti - con tre droni ce ne
      // sarebbe stata una per ciascuno. E' il motivo per cui i droni stanno fermi
      // NELLA PRIMA META' della run, dove l'ozio vale il 21-40% del tempo mentre
      // c'e' ancora tutto da esplorare (negli ultimi due decili scende a zero).
      //
      // Il ripiego prende la cella rilevante piu' vicina, chiunque ne sia il
      // proprietario. Non tocca la funzione cella->proprietario, che ogni drone
      // continua a calcolare identica: e' una decisione puramente locale che
      // scatta SOLO quando questo drone starebbe fermo comunque. Al massimo due
      // droni lavorano la stessa cella per un ciclo - che e' meglio di uno fermo.
      // Il ribilanciamento vero PROVATO E SCARTATO stava a monte, nel pesare le
      // celle per le frontiere: destabilizzava la ripartizione (cambiava nel
      // 67-83% dei ricalcoli) e nessun drone concludeva piu'. La lezione era di
      // non rendere la ripartizione piu' reattiva, ed e' quello che si evita qui.
      // Si sceglie la piu' vicina FRA QUELLE CHE NESSUN COMPAGNO STA LAVORANDO.
      //
      // Prendere la piu' vicina e basta - PROVATO, campagna RIPIEGO da 4 run - fa
      // centro sul bersaglio meccanico e sbaglia l'esito: le ripartizioni a vuoto
      // vanno da 21% a 0, l'ozio dal 33% al 16%, il volo dal 40% al 54%, ma le
      // durate peggiorano (497/527 s contro 478/498) e compaiono 2 crash su 4 run
      // contro 0 su 4. I droni volano di piu' e non esplorano di piu'.
      // Il perche' e' misurato: le celle ancora vive sono poche e stanno vicine
      // fra loro, quindi "la piu' vicina" e' la stessa per tutti e i droni si
      // ammassano dove qualcuno lavora gia' - 476 rilevamenti sotto 0.8 m, 103
      // sotto 0.5 m, minimo 0.36 m, dentro l'ingombro delle eliche. Il lavoro
      // viene duplicato invece che aggiunto, e ci si va addosso.
      // E' la seconda volta che questa classe di misura da' questa firma: la M1,
      // che faceva lo stesso a livello di frontiera, "ha raddoppiato i
      // ribaltamenti, 2 su 4 contro 4 su 16".
      // Le celle di ogni drone viaggiano gia' nello stato di sciame, quindi
      // l'informazione per evitarsi c'e': si scarta cio' che e' gia' preso e si
      // ripiega sulla piu' vicina in assoluto solo se non resta altro.
      //
      // ESITO (campagna RIPIEGO2, 4 run su office_small) - tutti i criteri
      // dichiarati prima della prova sono stati superati, ed e' il guadagno piu'
      // grande della sessione:
      //   durate    247/267/297/347 s, mediana 282 contro 393 del riferimento (-28%)
      //   completamento 4/4 a 3/3, crash 0 (RIPIEGO ne aveva 2 su 4)
      //   ozio 15% contro 33%, volo 52% contro 40%
      //   droni sotto 0.5 m fra loro: 28 contro 103
      // Conferma la lettura del difetto: il volo in piu' non era inutile, era
      // mal diretto. Mandare il drone fermo DOVE NON C'E' GIA' QUALCUNO converte
      // lo stesso tempo di volo in copertura invece che in duplicazione, e toglie
      // insieme la causa dei ribaltamenti. In 44 attivazioni su 53 una cella
      // libera esisteva davvero: bastava guardarla.
      if (mine.empty() && !relevant.empty()) {
        std::set<int> prese_dai_pari;
        const double now_busy = node_->now().seconds();
        for (const auto& pr : peers) {
          if (pr.first == ep_->drone_id_) continue;
          if (pr.first < 1 || pr.first > static_cast<int>(ed_->swarm_state_.size())) continue;
          const auto& ps = ed_->swarm_state_[pr.first - 1];
          // Solo stati freschi: una lista vecchia terrebbe occupate celle libere.
          if (fp_state_fresh_ > 1e-6 && now_busy - ps.recv_stamp_ > fp_state_fresh_) continue;
          for (int g : ps.grid_ids_) prese_dai_pari.insert(g);
        }
        int best_g = -1, ripiego_g = -1;
        double best_d = std::numeric_limits<double>::infinity();
        double ripiego_d = std::numeric_limits<double>::infinity();
        for (int g : relevant) {
          const double d = distTo(0, g);  // peers[0] e' sempre questo drone
          if (d < ripiego_d) {
            ripiego_d = d;
            ripiego_g = g;
          }
          if (prese_dai_pari.count(g)) continue;
          if (d < best_d) {
            best_d = d;
            best_g = g;
          }
        }
        const bool libera = best_g >= 0;
        if (!libera) {
          best_g = ripiego_g;
          best_d = ripiego_d;
        }
        if (best_g >= 0) {
          mine.push_back(best_g);
          RCLCPP_INFO_THROTTLE(node_->get_logger(), clock, 2000,
              "[Ripiego] Drone %d senza celle con %zu rilevanti (%zu gia' prese): va sulla %d a "
              "%.1fm%s",
              ep_->drone_id_, relevant.size(), prese_dai_pari.size(), best_g, best_d,
              libera ? "" : " - nessuna libera, la condivide");
        }
      }

      std::string carichi;
      for (size_t i = 0; i < load.size(); ++i)
        carichi += std::to_string(peers[i].first) + ":" + std::to_string(load[i]) + " ";
      RCLCPP_INFO_THROTTLE(node_->get_logger(), clock, 3000,
          "[Chiuso] %zu droni in contatto, %zu celle su %zu della griglia, %zu rilevanti: tengo %zu (carichi %s)",
          peers.size(), owner.size(), domain.size(), relevant.size(), mine.size(), carichi.c_str());

      grid_ids = mine;
    }

    state.grid_ids_ = grid_ids;
    owned_last_.clear();
    owned_last_.insert(grid_ids.begin(), grid_ids.end());

    // CONTROLLO DI COERENZA DELLA RIPARTIZIONE (solo diagnostica).
    //
    // La ripartizione e' esclusiva per costruzione, ma solo FRA I DRONI CHE SI
    // SENTONO: ognuno la calcola per conto proprio sui vicini che ha in quel
    // momento. Se due droni non si vedono - o si vedono con stati vecchi -
    // possono rivendicare la stessa cella e lavorarci entrambi, sprecando meta'
    // dello sforzo senza che niente lo segnali. E' successo davvero con la radio
    // a 10 m, dove i droni 1 e 3 calcolavano due ripartizioni indipendenti dello
    // stesso spazio.
    // Qui non si corregge nulla: si rende rumoroso un guasto altrimenti muto.
    for (int id : ed_->neighbor_ids_) {
      if (id < 1 || id > static_cast<int>(ed_->swarm_state_.size()) || id == ep_->drone_id_)
        continue;
      const auto& peer = ed_->swarm_state_[id - 1];
      vector<int> collisione;
      for (int g : grid_ids)
        if (std::find(peer.grid_ids_.begin(), peer.grid_ids_.end(), g) != peer.grid_ids_.end())
          collisione.push_back(g);
      if (!collisione.empty()) {
        std::string elenco;
        for (int g : collisione) elenco += std::to_string(g) + " ";
        RCLCPP_WARN(node_->get_logger(),
            "[Coerenza] Drone %d e drone %d rivendicano entrambi %zu celle (%s): lavoro duplicato",
            ep_->drone_id_, id, collisione.size(), elenco.c_str());
      }
    }
  }

  if (grid_ids.empty()) {
    // Perche' nessuna griglia: si stampa lo stato di TUTTE le celle invece di
    // dedurlo. isRelevant richiede raggiungibile E stato esplorabile, e finora
    // ho corretto le due condizioni una alla volta indovinando quale fosse
    // chiusa - due volte di fila senza effetto.
    RCLCPP_WARN(node_->get_logger(), "Empty dominance. %s", hgrid_->gridSummary().c_str());
    ed_->region_tour_.clear();
    return false;
  }

  const auto& assigned_grids = state.grid_ids_;
  const auto& assigned_hulls = state.center_hulls_;
  vector<int> assigned_center_grid_ids = state.center_grid_ids_;
  if (assigned_center_grid_ids.size() < state.center_positions_.size()) {
    assigned_center_grid_ids.resize(state.center_positions_.size(), -1);
  }
  for (int i = 0; i < static_cast<int>(state.center_positions_.size()); ++i) {
    if (i < static_cast<int>(assigned_center_grid_ids.size()) &&
        assigned_center_grid_ids[i] >= 0)
      continue;
    int gid = -1;
    if (hgrid_->getGridIdByCenterPos(state.center_positions_[i], gid, 1.0)) {
      assigned_center_grid_ids[i] = gid;
    }
  }
  if (assigned_center_grid_ids != state.center_grid_ids_) {
    state.center_grid_ids_ = assigned_center_grid_ids;
  }

  vector<HGrid::HullTask> hull_tasks;
  const int hull_task_num = std::min(static_cast<int>(assigned_center_grid_ids.size()),
      static_cast<int>(state.center_hulls_.size()));
  hull_tasks.reserve(hull_task_num);
  for (int i = 0; i < hull_task_num; ++i) {
    if (assigned_center_grid_ids[i] < 0 || state.center_hulls_[i].empty()) continue;
    HGrid::HullTask task;
    task.grid_id = assigned_center_grid_ids[i];
    task.hull = state.center_hulls_[i];
    hull_tasks.push_back(std::move(task));
  }

  // Build the cost matrix.
  Eigen::MatrixXd mat_full;
  if (!hull_tasks.empty()) {
    hgrid_->getTourCostMatrix(positions, velocities, grid_ids, mat_full, &hull_tasks);
  } else {
    hgrid_->getTourCostMatrix(positions, velocities, grid_ids, mat_full);
  }

  const double tg_matrice = (node_->now() - t1).seconds();  // [TempiGiro]
  const auto& full_center_grid_ids = hgrid_->getLastCostMatrixCenterGridIds();
  const auto& full_center_positions = hgrid_->getLastCostMatrixCenterPositions();
  const auto& full_center_hulls = hgrid_->getLastCostMatrixCenterHulls();
  const auto& full_center_ids = hgrid_->getLastCostMatrixCenterIds();
  const auto& full_center_types = hgrid_->getLastCostMatrixCenterTypes();
  Eigen::MatrixXd mat = mat_full;
  vector<int> center_grid_ids(full_center_grid_ids.begin(), full_center_grid_ids.end());
  vector<Vector3d> center_positions(full_center_positions.begin(), full_center_positions.end());

  const int task_num = std::max(
      static_cast<int>(state.center_positions_.size()),
      std::max(static_cast<int>(assigned_hulls.size()),
          static_cast<int>(assigned_center_grid_ids.size())));
  bool used_assigned_centers = false;
  if (task_num > 0) {
    unordered_map<int, int> assigned_grid_task_count;
    assigned_grid_task_count.reserve(task_num);
    for (int ti = 0; ti < task_num; ++ti) {
      const int gid = (ti < static_cast<int>(assigned_center_grid_ids.size()))
                          ? assigned_center_grid_ids[ti]
                          : -1;
      if (gid >= 0) {
        assigned_grid_task_count[gid] += 1;
      }
    }

    vector<int> selected_indices;
    vector<int> selected_task_indices;
    unordered_set<int> seen;
    selected_indices.reserve(task_num * 2);
    selected_task_indices.reserve(task_num * 2);

    unordered_map<int, vector<int>> grid_to_indices;
    grid_to_indices.reserve(full_center_grid_ids.size());
    for (int i = 0; i < static_cast<int>(full_center_grid_ids.size()); ++i) {
      grid_to_indices[full_center_grid_ids[i]].push_back(i);
    }

    auto chooseNearestByRef = [&](const vector<int>& candidates, const Vector3d& ref) {
      int best_idx = -1;
      double best_d2 = std::numeric_limits<double>::infinity();
      for (const int idx : candidates) {
        const double d2 = (full_center_positions[idx] - ref).squaredNorm();
        if (d2 < best_d2) {
          best_d2 = d2;
          best_idx = idx;
        }
      }
      return best_idx;
    };
    auto preferUnknownCenters = [&](const vector<int>& candidates) {
      vector<int> unknowns;
      unknowns.reserve(candidates.size());
      for (const int idx : candidates) {
        if (idx >= 0 && idx < static_cast<int>(full_center_types.size()) &&
            full_center_types[idx] == HGrid::UNKNOWN_ACTIVE_CENTER) {
          unknowns.push_back(idx);
        }
      }
      return unknowns.empty() ? candidates : unknowns;
    };

    int split_task_num = 0;
    int split_task_matched = 0;
    int split_task_finished = 0;
    unordered_set<int> split_locked_grids;
    split_locked_grids.reserve(task_num);

    // Task-token driven update: split tasks are solely tracked by their assigned hull.
    for (int ti = 0; ti < task_num; ++ti) {
      const bool has_hull = (ti < static_cast<int>(assigned_hulls.size()) &&
                             !assigned_hulls[ti].empty());
      const int task_gid = (ti < static_cast<int>(assigned_center_grid_ids.size()))
                               ? assigned_center_grid_ids[ti]
                               : -1;

      if (has_hull) {
        ++split_task_num;
        vector<int> contain_indices;
        contain_indices.reserve(4);
        for (int idx = 0; idx < static_cast<int>(full_center_grid_ids.size()); ++idx) {
          if (seen.find(idx) != seen.end()) continue;
          if (pointInsideConvexHull2D(assigned_hulls[ti], full_center_positions[idx])) {
            contain_indices.push_back(idx);
          }
        }

        if (!contain_indices.empty()) {
          const vector<int> preferred = preferUnknownCenters(contain_indices);
          const int matched_idx = chooseNearestByRef(preferred, hullCentroid(assigned_hulls[ti]));
          if (matched_idx >= 0 && seen.insert(matched_idx).second) {
            selected_indices.push_back(matched_idx);
            selected_task_indices.push_back(ti);
            ++split_task_matched;
          }
          if (task_gid >= 0) split_locked_grids.insert(task_gid);
        } else {
          // Hull task completion criterion:
          // if neither active UNKNOWN nor active FREE center falls inside this hull, this task is
          // finished.
          ++split_task_finished;
        }
        continue;
      }

      // Non-split task: still represented by center-grid token.
      if (task_gid < 0) continue;
      if (split_locked_grids.find(task_gid) != split_locked_grids.end()) {
        // This grid already has a split-hull token; do not re-expand by grid token.
        continue;
      }
      auto it = grid_to_indices.find(task_gid);
      if (it == grid_to_indices.end()) continue;

      vector<int> cands;
      cands.reserve(it->second.size());
      for (const int idx : it->second) {
        if (seen.find(idx) == seen.end()) cands.push_back(idx);
      }
      if (cands.empty()) continue;
      const vector<int> preferred = preferUnknownCenters(cands);
      const bool grid_has_split_centers = hasExplicitSplitCenter(it->second, full_center_hulls);

      const bool can_expand_grid_token =
          (assigned_grid_task_count.count(task_gid) > 0 &&
           assigned_grid_task_count.at(task_gid) == 1 &&
           !grid_has_split_centers);
      if (can_expand_grid_token && preferred.size() > 1) {
        for (const int idx : preferred) {
          if (idx >= 0 && seen.insert(idx).second) {
            selected_indices.push_back(idx);
            selected_task_indices.push_back(ti);
          }
        }
        continue;
      }

      int selected = preferred.front();
      if (preferred.size() > 1 && ti < static_cast<int>(state.center_positions_.size())) {
        selected = chooseNearestByRef(preferred, state.center_positions_[ti]);
      }
      if (selected >= 0 && seen.insert(selected).second) {
        selected_indices.push_back(selected);
        selected_task_indices.push_back(ti);
      }
    }

    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
        "[CenterFilter] Drone " << ep_->drone_id_ << " split_tasks=" << split_task_num
                                 << " split_matched=" << split_task_matched << " split_finished="
                                 << split_task_finished
                                 << " split_locked_grids=" << split_locked_grids.size());

    if (!selected_indices.empty()) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 1000,
          "[CenterFilter] Drone " << ep_->drone_id_ << " assigned_grids=" << assigned_grids.size()
                                   << " assigned_hulls=" << assigned_hulls.size()
                                   << " selected_centers=" << selected_indices.size()
                                   << " full_centers=" << full_center_ids.size());
      const int drone_num = positions.size();
      const int sel_num = selected_indices.size();
      const int dimen = 1 + drone_num + sel_num;
      mat = Eigen::MatrixXd::Zero(dimen, dimen);
      // Copy depot/drone blocks
      for (int i = 0; i < 1 + drone_num; ++i) {
        for (int j = 0; j < 1 + drone_num; ++j) {
          mat(i, j) = mat_full(i, j);
        }
      }
      // Copy depot/drone <-> center and center <-> center blocks
      for (int i = 0; i < sel_num; ++i) {
        int full_i = 1 + drone_num + selected_indices[i];
        for (int j = 0; j < 1 + drone_num; ++j) {
          mat(j, 1 + drone_num + i) = mat_full(j, full_i);
          mat(1 + drone_num + i, j) = mat_full(full_i, j);
        }
        for (int j = 0; j < sel_num; ++j) {
          int full_j = 1 + drone_num + selected_indices[j];
          mat(1 + drone_num + i, 1 + drone_num + j) = mat_full(full_i, full_j);
        }
      }

      center_grid_ids.clear();
      center_positions.clear();
      center_grid_ids.reserve(sel_num);
      center_positions.reserve(sel_num);
      vector<int> selected_center_ids;
      vector<int> selected_center_grid_ids;
      vector<vector<Vector3d>> selected_center_hulls;
      selected_center_ids.reserve(sel_num);
      selected_center_grid_ids.reserve(sel_num);
      selected_center_hulls.reserve(sel_num);
      for (int i = 0; i < sel_num; ++i) {
        const int idx = selected_indices[i];
        const int ti = selected_task_indices[i];
        if (idx < 0 || idx >= static_cast<int>(full_center_grid_ids.size())) continue;
        center_grid_ids.push_back(full_center_grid_ids[idx]);
        center_positions.push_back(full_center_positions[idx]);
        if (idx < static_cast<int>(full_center_ids.size())) {
          selected_center_ids.push_back(full_center_ids[idx]);
        } else {
          selected_center_ids.push_back(-1);
        }
        if (ti < static_cast<int>(assigned_hulls.size()) && !assigned_hulls[ti].empty()) {
          // Preserve split-task hull token instead of replacing by refreshed full hull.
          selected_center_hulls.push_back(assigned_hulls[ti]);
        } else if (idx < static_cast<int>(full_center_hulls.size())) {
          selected_center_hulls.push_back(full_center_hulls[idx]);
        } else {
          selected_center_hulls.emplace_back();
        }
        if (ti < static_cast<int>(assigned_center_grid_ids.size()) &&
            assigned_center_grid_ids[ti] >= 0) {
          selected_center_grid_ids.push_back(assigned_center_grid_ids[ti]);
        } else {
          selected_center_grid_ids.push_back(full_center_grid_ids[idx]);
        }
      }
      state.center_positions_ = center_positions;
      state.center_hulls_ = selected_center_hulls;
      state.center_ids_ = selected_center_ids;
      state.center_grid_ids_ = selected_center_grid_ids;
    } else {
      RCLCPP_WARN(node_->get_logger(),
          "[ATSP] Assigned centers not found from grid/hull after refresh, skip this round");
      // No assigned center survives after refresh:
      // clear stale grid/hull task tokens so finished hull tasks are retired immediately.
      state.center_ids_.clear();
      state.center_positions_.clear();
      state.center_hulls_.clear();
      state.center_grid_ids_.clear();
      indices.clear();
      ed_->region_tour_.clear();
      ed_->reallocated_ = false;
      return false;
    }
    used_assigned_centers = true;
  }

  int center_num = center_grid_ids.size();
  if (used_assigned_centers && center_num == 0) {
    // Strict ownership mode: do not expand from grid ids when assigned grid/hull cannot be matched.
    RCLCPP_WARN(node_->get_logger(),
        "[ATSP] No matched assigned center by grid/hull after refresh, skip this planning round");
    indices.clear();
    ed_->region_tour_.clear();
    ed_->reallocated_ = false;
    return false;
  }
  if (center_num <= 1) {
    ed_->region_tour_.clear();
    ed_->region_tour_.push_back(positions[0]);
    if (!center_grid_ids.empty()) {
      indices = { center_grid_ids.front() };
      if (!center_positions.empty()) {
        ed_->region_tour_.push_back(center_positions.front());
      }
    } else {
      indices = grid_ids;
      if (used_assigned_centers && !center_positions.empty()) {
        for (const auto& c : center_positions) {
          ed_->region_tour_.push_back(c);
        }
      } else {
        for (auto gid : indices) {
          ed_->region_tour_.push_back(hgrid_->getCenter(gid));
        }
      }
    }
    ed_->last_grid_ids_ = indices;
    ed_->reallocated_ = false;
    RCLCPP_WARN(node_->get_logger(), "[ATSP] Skip solver: <=1 center");
    return true;
  }

  double mat_time = (node_->now() - t1).seconds();

  // Find optimal path through ATSP
  t1 = node_->now();
  const int dimension = mat.rows();
  const int drone_num = 1;
  if (dimension < 3) {
    indices = grid_ids;
    ed_->region_tour_.clear();
    ed_->region_tour_.push_back(positions[0]);
    if (used_assigned_centers && !center_positions.empty()) {
      for (const auto& c : center_positions) {
        ed_->region_tour_.push_back(c);
      }
    } else {
      for (auto gid : indices) {
        ed_->region_tour_.push_back(hgrid_->getCenter(gid));
      }
    }
    ed_->last_grid_ids_ = indices;
    ed_->reallocated_ = false;
    return true;
  }

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  int sanitized_cost_cnt = 0;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = toLkhEdgeWeight(mat(i, j), sanitized_cost_cnt);
      file << int_cost << " ";
    }
    file << "\n";
  }
  if (sanitized_cost_cnt > 0) {
    RCLCPP_WARN(node_->get_logger(), "[GridTour] Sanitized %d invalid/overflow cost entries before LKH",
        sanitized_cost_cnt);
  }
  file.close();

  // Create par file
  file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
  // "\n"; file << "MTSP_MAX_SIZE = "
  //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "PRECISION = " << kLkhPrecision << "\n";
  file << "RUNS = 1\n";
  file << "SEED = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  const double tg_prima_lkh = (node_->now() - t1).seconds();  // [TempiGiro]
  if (!callSolveTSP(tsp_client_node_, tsp_client_, 2)) {
    RCLCPP_ERROR(node_->get_logger(), "Fail to solve ATSP.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
      "[TempiGiro] griglia %.3f s, matrice dei costi %.3f s, preparazione %.3f s, LKH %.3f s",
      tg_griglia, tg_matrice - tg_griglia, tg_prima_lkh - tg_matrice,
      (node_->now() - t1).seconds() - tg_prima_lkh);
  // Read results
  t1 = node_->now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  others.resize(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      others[tours[i][0] - 2].insert(
          others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
    }
  }
  bool fallback_to_grid_ids = false;
  if (indices.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[ATSP] Empty tour parsed, fallback to grid_ids");
    indices = grid_ids;
    fallback_to_grid_ids = true;
  }
  vector<int> center_order;
  if (!fallback_to_grid_ids) {
    center_order.reserve(indices.size());
    for (int id : indices) {
      int center_idx = id - 1 - drone_num;
      if (center_idx >= 0 && center_idx < static_cast<int>(center_positions.size())) {
        center_order.push_back(center_idx);
      }
    }
    for (auto& id : indices) {
      id -= 1 + drone_num;
      if (id >= 0 && id < static_cast<int>(center_grid_ids.size())) {
        id = center_grid_ids[id];
      }
    }
    for (auto& other : others) {
      for (auto& id : other) {
        id -= 1 + drone_num;
        if (id >= 0 && id < static_cast<int>(center_grid_ids.size())) {
          id = center_grid_ids[id];
        }
      }
    }
  }

  std::cout << "\033[1;32mGrid tour: ";
  if (!fallback_to_grid_ids) {
    unordered_set<int> seen;
    vector<int> unique_indices;
    for (int id : indices) {
      if (seen.insert(id).second) unique_indices.push_back(id);
    }
    indices.swap(unique_indices);
    for (auto& other : others) {
      seen.clear();
      vector<int> unique_other;
      for (int id : other) {
        if (seen.insert(id).second) unique_other.push_back(id);
      }
      other.swap(unique_other);
    }
  }
  for (auto& id : indices) {
    std::cout << id << " ";
  }
  std::cout << "\033[0m" << std::endl;

  ed_->region_tour_.clear();
  ed_->region_tour_.push_back(positions[0]);
  if (!center_order.empty()) {
    for (int center_idx : center_order) {
      if (center_idx >= 0 && center_idx < static_cast<int>(center_positions.size())) {
        ed_->region_tour_.push_back(center_positions[center_idx]);
      }
    }
  } else {
    unordered_map<int, int> first_center_idx;
    first_center_idx.reserve(center_grid_ids.size());
    for (int i = 0; i < static_cast<int>(center_grid_ids.size()); ++i) {
      if (first_center_idx.find(center_grid_ids[i]) == first_center_idx.end()) {
        first_center_idx[center_grid_ids[i]] = i;
      }
    }
    for (int gid : indices) {
      auto it = first_center_idx.find(gid);
      if (it != first_center_idx.end()) {
        ed_->region_tour_.push_back(center_positions[it->second]);
      } else {
        ed_->region_tour_.push_back(hgrid_->getCenter(gid));
      }
    }
  }

  grid_ids = indices;

  ed_->last_grid_ids_ = grid_ids;
  ed_->reallocated_ = false;


  return true;
}

void C2ExplorationManager::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices) {
  // if (ftr_ids.empty()) {
  //   indices.clear();
  //   return;
  // }

  indices.clear();

  // Filter out stale/invalid frontier ids propagated from grid cache.
  vector<int> valid_ftr_ids;
  valid_ftr_ids.reserve(ftr_ids.size());
  unordered_set<int> unique_ids;
  for (const int id : ftr_ids) {
    if (id < 0 || id >= static_cast<int>(ed_->frontiers_.size())) {
      RCLCPP_WARN(node_->get_logger(), "[FrontierTour] Skip invalid frontier id %d (frontier size: %zu)",
          id, ed_->frontiers_.size());
      continue;
    }
    if (unique_ids.insert(id).second) {
      valid_ftr_ids.push_back(id);
    }
  }
  if (valid_ftr_ids.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[FrontierTour] No valid frontier id remains after filtering");
    return;
  }

  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
  Eigen::MatrixXd mat;
  // getSwarmCostMatrix costs between frontiers.
  frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, valid_ftr_ids, grid_pos, mat);
  const int dimension = mat.rows();
  if (dimension < 3) {
    indices = valid_ftr_ids;
    return;
  }

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  int sanitized_cost_cnt = 0;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = toLkhEdgeWeight(mat(i, j), sanitized_cost_cnt);
      file << int_cost << " ";
    }
    file << "\n";
  }
  if (sanitized_cost_cnt > 0) {
    RCLCPP_WARN(node_->get_logger(),
        "[FrontierTour] Sanitized %d invalid/overflow cost entries before LKH", sanitized_cost_cnt);
  }
  file.close();

  // Create par file
  const int drone_num = 1;

  file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
  file << "MTSP_MAX_SIZE = "
       << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "PRECISION = " << kLkhPrecision << "\n";
  file << "RUNS = 1\n";
  file << "SEED = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  if (!callSolveTSP(tsp_client_node_, tsp_client_, 1)) {
    RCLCPP_ERROR(node_->get_logger(), "Fail to solve ATSP.");
    return;
  }

  // Read results
  ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
  if (!fin.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "[FrontierTour] Failed to open tour file for parsing");
    return;
  }
  string res;
  vector<int> ids;
  bool found_tour_section = false;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) {
      found_tour_section = true;
      break;
    }
  }
  if (!found_tour_section) {
    RCLCPP_ERROR(node_->get_logger(), "[FrontierTour] TOUR_SECTION not found in mtsp tour file");
    fin.close();
    return;
  }
  while (getline(fin, res)) {
    int id = 0;
    try {
      id = stoi(res);
    } catch (const std::exception&) {
      continue;
    }
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour
  vector<vector<int>> tours;
  parseMultiTours(ids, drone_num, dimension, tours);

  for (const auto& tour : tours) {
    if (tour.empty() || tour.front() != 1) continue;
    for (int i = 1; i < static_cast<int>(tour.size()); ++i) {
      const int local_id = tour[i] - (1 + drone_num);
      if (!grid_pos.empty() && local_id == static_cast<int>(valid_ftr_ids.size())) {
        // Virtual "next grid" node used by TSP refinement.
        continue;
      }
      if (local_id < 0 || local_id >= static_cast<int>(valid_ftr_ids.size())) {
        RCLCPP_WARN(node_->get_logger(), "[FrontierTour] Skip parsed frontier index %d (valid size: %zu)",
            local_id, valid_ftr_ids.size());
        continue;
      }
      indices.push_back(valid_ftr_ids[local_id]);
    }
  }

  if (indices.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[FrontierTour] Empty parsed tour, fallback to filtered frontier ids");
    indices = valid_ftr_ids;
  }

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
  if (!grid_pos.empty()) {
    ed_->frontier_tour_.push_back(grid_pos[0]);
  }

  // ed_->other_tours_.clear();
  // for (int i = 1; i < positions.size(); ++i) {
  //   ed_->other_tours_.push_back({});
  //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
  // }
}

bool C2ExplorationManager::calcolaStrisce() {
  if (strisce_pronte_) return true;
  const int n = static_cast<int>(ep_->drone_num_);
  if (!ep_->partizione_strisce_ || n < 2 || static_cast<int>(ep_->partenze_.size()) < 2 * n)
    return false;
  std::vector<int> dominio;
  hgrid_->getAllIds(dominio);
  if (dominio.empty()) return false;
  // Direzione della linea delle partenze: dalla prima alla piu' lontana da lei.
  const Eigen::Vector2d p0(ep_->partenze_[0], ep_->partenze_[1]);
  Eigen::Vector2d dir = Eigen::Vector2d::Zero();
  for (int i = 1; i < n; ++i) {
    const Eigen::Vector2d pi(ep_->partenze_[2 * i], ep_->partenze_[2 * i + 1]);
    if ((pi - p0).norm() > dir.norm()) dir = pi - p0;
  }
  if (dir.norm() < 0.5) return false;   // partenze coincidenti: niente strisce
  dir.normalize();
  const Eigen::Vector2d trasv(-dir.y(), dir.x());
  struct Cella { int id; double lungo, trasversale; };
  std::vector<Cella> celle;
  centri_celle_.clear();
  for (int g : dominio) {
    Eigen::Vector3d c;
    if (!hgrid_->getGridCenter(g, c)) continue;
    centri_celle_.emplace_back(g, c);
    // arrotondate al centimetro: l'ordine non deve dipendere dagli errori di arrotondamento
    celle.push_back({ g, std::round(100.0 * dir.dot(c.head<2>())) / 100.0,
        std::round(100.0 * trasv.dot(c.head<2>())) / 100.0 });
  }
  if (celle.size() < static_cast<size_t>(n)) return false;
  std::sort(celle.begin(), celle.end(), [](const Cella& a, const Cella& b) {
    if (a.lungo != b.lungo) return a.lungo < b.lungo;
    if (a.trasversale != b.trasversale) return a.trasversale < b.trasversale;
    return a.id < b.id;
  });
  std::vector<std::pair<double, int>> droni;
  for (int i = 0; i < n; ++i)
    droni.emplace_back(dir.dot(Eigen::Vector2d(ep_->partenze_[2 * i], ep_->partenze_[2 * i + 1])),
        i + 1);
  std::sort(droni.begin(), droni.end());
  const size_t m = celle.size();
  size_t k = 0;
  std::string riepilogo;
  for (int r = 0; r < n; ++r) {
    const size_t quante = m / n + (static_cast<size_t>(r) < m % n ? 1 : 0);
    riepilogo += "drone " + std::to_string(droni[r].second) + ":";
    for (size_t j = 0; j < quante && k < m; ++j, ++k) {
      strisce_[celle[k].id] = droni[r].second;
      riepilogo += " " + std::to_string(celle[k].id);
    }
    riepilogo += r + 1 < n ? "; " : "";
  }
  strisce_pronte_ = true;
  RCLCPP_WARN(node_->get_logger(),
      "[Strisce] spartizione della griglia (%zu celle) lungo la linea delle partenze "
      "(%.2f, %.2f): %s", m, dir.x(), dir.y(), riepilogo.c_str());
  return true;
}

int C2ExplorationManager::proprietarioStriscia(const Eigen::Vector3d& p) {
  if (!calcolaStrisce() || centri_celle_.empty()) return -1;
  int cella = -1;
  double d_min = std::numeric_limits<double>::infinity();
  for (const auto& kv : centri_celle_) {
    const double d = (kv.second - p).head<2>().squaredNorm();
    if (d < d_min) { d_min = d; cella = kv.first; }
  }
  const auto it = strisce_.find(cella);
  return it == strisce_.end() ? -1 : it->second;
}

bool C2ExplorationManager::inCellaDiUnCompagno(const Eigen::Vector3d& p) {
  if (!strisce_pronte_ || centri_celle_.empty()) return false;
  int cella = -1;
  double d_min = std::numeric_limits<double>::infinity();
  for (const auto& kv : centri_celle_) {
    const double d = (kv.second - p).head<2>().squaredNorm();
    if (d < d_min) { d_min = d; cella = kv.first; }
  }
  const double ora = node_->now().seconds();
  for (int k = 0; k < static_cast<int>(ed_->swarm_state_.size()); ++k) {
    if (k + 1 == ep_->drone_id_) continue;
    const auto& ps = ed_->swarm_state_[k];
    if (ps.recv_stamp_ <= 0.0 || ora - ps.recv_stamp_ > std::max(fp_state_fresh_, 1.0)) continue;
    if (std::find(ps.grid_ids_.begin(), ps.grid_ids_.end(), cella) != ps.grid_ids_.end())
      return true;
  }
  return false;
}

int C2ExplorationManager::toLkhEdgeWeight(const double raw_cost, int& sanitized_count) {
  double cost = raw_cost;
  if (!std::isfinite(cost) || cost < 0.0) {
    cost = kLkhFallbackBlockedCost;
    ++sanitized_count;
  }
  long long w = static_cast<long long>(std::llround(cost * kLkhCostScale));
  if (w < 0) {
    w = 0;
    ++sanitized_count;
  } else if (w > kLkhMaxEdgeWeight) {
    w = kLkhMaxEdgeWeight;
    ++sanitized_count;
  }
  return static_cast<int>(w);
}

void C2ExplorationManager::parseMultiTours(
    const vector<int>& ids, const int drone_num, const int dimension, vector<vector<int>>& tours) {
  tours.clear();
  vector<int> cur;
  vector<int> prefix;
  bool seen_salesman = false;
  for (int id : ids) {
    if (id <= 0) {
      continue;
    }
    if (id <= drone_num) {
      if (!cur.empty()) {
        tours.push_back(cur);
      }
      cur.clear();
      cur.push_back(id);
      seen_salesman = true;
      continue;
    }
    if (id >= dimension) {
      continue;
    }
    if (seen_salesman) {
      cur.push_back(id);
    } else {
      prefix.push_back(id);
    }
  }
  if (!cur.empty()) {
    tours.push_back(cur);
  }
  if (!prefix.empty() && !tours.empty()) {
    tours.back().insert(tours.back().end(), prefix.begin(), prefix.end());
  }
}

bool C2ExplorationManager::pointInsideConvexHull2D(
    const vector<Vector3d>& hull, const Vector3d& p, const double tol) {
  if (hull.empty()) return false;
  if (hull.size() == 1) {
    return (hull.front().head<2>() - p.head<2>()).norm() <= tol;
  }
  if (hull.size() == 2) {
    const auto a = hull[0].head<2>();
    const auto b = hull[1].head<2>();
    const auto ab = b - a;
    const double len = ab.norm();
    if (len < 1e-6) return (a - p.head<2>()).norm() <= tol;
    const double t = std::max(0.0, std::min(1.0, (p.head<2>() - a).dot(ab) / (len * len)));
    const auto proj = a + t * ab;
    return (proj - p.head<2>()).norm() <= tol;
  }

  int sign = 0;
  for (size_t i = 0; i < hull.size(); ++i) {
    const auto a = hull[i].head<2>();
    const auto b = hull[(i + 1) % hull.size()].head<2>();
    const auto e = b - a;
    const double el = e.norm();
    if (el < 1e-6) continue;
    const auto ap = p.head<2>() - a;
    const double signed_dist = (e.x() * ap.y() - e.y() * ap.x()) / el;
    if (std::abs(signed_dist) <= tol) continue;
    const int cur_sign = (signed_dist > 0.0) ? 1 : -1;
    if (sign == 0) {
      sign = cur_sign;
    } else if (sign != cur_sign) {
      return false;
    }
  }
  return true;
}

Vector3d C2ExplorationManager::hullCentroid(const vector<Vector3d>& hull) {
  Vector3d c = Vector3d::Zero();
  if (hull.empty()) return c;
  for (const auto& p : hull) c += p;
  c /= static_cast<double>(hull.size());
  return c;
}

bool C2ExplorationManager::hasExplicitSplitCenter(
    const vector<int>& candidate_indices, const vector<vector<Vector3d>>& center_hulls) {
  for (const int idx : candidate_indices) {
    if (idx >= 0 && idx < static_cast<int>(center_hulls.size()) && !center_hulls[idx].empty()) {
      return true;
    }
  }
  return false;
}

bool C2ExplorationManager::candidateBlockedByHull(
    const Vector3d& center, const vector<vector<Vector3d>>& blocked_center_hulls) {
  for (const auto& hull : blocked_center_hulls) {
    if (!hull.empty() && pointInsideConvexHull2D(hull, center)) return true;
  }
  return false;
}

void C2ExplorationManager::filterMeetingOptCentersByParticipantTasks(const shared_ptr<HGrid>& hgrid,
    const vector<DroneState>& participant_states, const vector<int>& center_grid_ids,
    const vector<Vector3d>& center_positions, const vector<int>& center_ids,
    const vector<int>& center_types, const vector<vector<Vector3d>>& center_hulls,
    vector<int>& selected_indices) {
  selected_indices.clear();
  if (!hgrid || participant_states.empty() || center_grid_ids.empty() || center_positions.empty()) {
    return;
  }

  struct NormalizedStateInfo {
    vector<int> task_gids;
    vector<int> extra_grid_ids;
    unordered_map<int, int> grid_task_count;
  };

  unordered_map<int, vector<int>> grid_to_indices;
  grid_to_indices.reserve(center_grid_ids.size());
  for (int i = 0; i < static_cast<int>(center_grid_ids.size()); ++i) {
    grid_to_indices[center_grid_ids[i]].push_back(i);
  }
  unordered_map<int, vector<int>> center_id_to_indices;
  center_id_to_indices.reserve(center_ids.size());
  for (int i = 0; i < static_cast<int>(center_ids.size()); ++i) {
    if (center_ids[i] >= 0) {
      center_id_to_indices[center_ids[i]].push_back(i);
    }
  }

  auto chooseNearestByRef = [&](const vector<int>& candidates, const Vector3d& ref) {
    int best_idx = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const int idx : candidates) {
      const double d2 = (center_positions[idx] - ref).squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        best_idx = idx;
      }
    }
    return best_idx;
  };

  auto preferUnknownCenters = [&](const vector<int>& candidates) {
    vector<int> unknowns;
    unknowns.reserve(candidates.size());
    for (const int idx : candidates) {
      if (idx >= 0 && idx < static_cast<int>(center_types.size()) &&
          center_types[idx] == HGrid::UNKNOWN_ACTIVE_CENTER) {
        unknowns.push_back(idx);
      }
    }
    return unknowns.empty() ? candidates : unknowns;
  };

  vector<NormalizedStateInfo> state_infos(participant_states.size());
  unordered_map<int, int> global_grid_task_count;
  global_grid_task_count.reserve(center_grid_ids.size() * 2);

  for (int si = 0; si < static_cast<int>(participant_states.size()); ++si) {
    const auto& state = participant_states[si];
    auto& info = state_infos[si];

    vector<int> assigned_center_grid_ids = state.center_grid_ids_;
    if (assigned_center_grid_ids.size() < state.center_positions_.size()) {
      assigned_center_grid_ids.resize(state.center_positions_.size(), -1);
    }
    for (int i = 0; i < static_cast<int>(state.center_positions_.size()); ++i) {
      if (i < static_cast<int>(assigned_center_grid_ids.size()) &&
          assigned_center_grid_ids[i] >= 0) {
        continue;
      }
      int gid = -1;
      if (hgrid->getGridIdByCenterPos(state.center_positions_[i], gid, 1.0)) {
        assigned_center_grid_ids[i] = gid;
      }
    }

    const int task_num = std::max(static_cast<int>(state.center_positions_.size()),
        std::max(static_cast<int>(state.center_hulls_.size()),
            static_cast<int>(assigned_center_grid_ids.size())));
    info.task_gids.resize(task_num, -1);
    unordered_set<int> covered_grid_tokens;
    covered_grid_tokens.reserve(task_num + state.grid_ids_.size());

    for (int ti = 0; ti < task_num; ++ti) {
      int task_gid = (ti < static_cast<int>(assigned_center_grid_ids.size()))
                         ? assigned_center_grid_ids[ti]
                         : -1;
      if (task_gid < 0 && ti < static_cast<int>(state.center_positions_.size())) {
        hgrid->getGridIdByCenterPos(state.center_positions_[ti], task_gid, 1.0);
      }
      info.task_gids[ti] = task_gid;
      if (task_gid < 0) continue;
      covered_grid_tokens.insert(task_gid);
      info.grid_task_count[task_gid] += 1;
      global_grid_task_count[task_gid] += 1;
    }

    for (const int gid : state.grid_ids_) {
      if (gid < 0 || covered_grid_tokens.find(gid) != covered_grid_tokens.end()) continue;
      info.extra_grid_ids.push_back(gid);
      info.grid_task_count[gid] += 1;
      global_grid_task_count[gid] += 1;
    }
  }

  unordered_set<int> seen;
  seen.reserve(center_positions.size());

  for (int si = 0; si < static_cast<int>(participant_states.size()); ++si) {
    const auto& state = participant_states[si];
    const auto& info = state_infos[si];
    const int task_num = info.task_gids.size();

    for (int ti = 0; ti < task_num; ++ti) {
      const bool has_ref_center = (ti < static_cast<int>(state.center_positions_.size()));
      const Vector3d ref_center =
          has_ref_center ? state.center_positions_[ti] : Vector3d::Zero();

      vector<Vector3d> task_hull;
      bool has_hull = false;
      if (ti < static_cast<int>(state.center_hulls_.size()) && !state.center_hulls_[ti].empty()) {
        task_hull = state.center_hulls_[ti];
        has_hull = true;
      } else if (has_ref_center) {
        has_hull = hgrid->getCenterHullByPos(ref_center, task_hull, 1.0) && !task_hull.empty();
      }

      const int task_gid = (ti < static_cast<int>(info.task_gids.size())) ? info.task_gids[ti] : -1;
      const int task_cid = (ti < static_cast<int>(state.center_ids_.size())) ? state.center_ids_[ti] : -1;
      const bool single_token_grid = (task_gid >= 0 &&
                                      info.grid_task_count.count(task_gid) > 0 &&
                                      info.grid_task_count.at(task_gid) == 1 &&
                                      global_grid_task_count.count(task_gid) > 0 &&
                                      global_grid_task_count.at(task_gid) == 1);

      if (has_hull) {
        vector<int> contain_indices;
        contain_indices.reserve(4);
        for (int idx = 0; idx < static_cast<int>(center_positions.size()); ++idx) {
          if (seen.find(idx) != seen.end()) continue;
          if (pointInsideConvexHull2D(task_hull, center_positions[idx])) {
            contain_indices.push_back(idx);
          }
        }

        if (contain_indices.empty()) continue;
        const vector<int> preferred = preferUnknownCenters(contain_indices);
        const Vector3d ref = has_ref_center ? ref_center : hullCentroid(task_hull);
        const int matched_idx = chooseNearestByRef(preferred, ref);
        if (matched_idx >= 0 && seen.insert(matched_idx).second) {
          selected_indices.push_back(matched_idx);
        }
        continue;
      }

      bool matched_exact_center = false;
      if (task_cid >= 0) {
        auto it = center_id_to_indices.find(task_cid);
        if (it != center_id_to_indices.end()) {
          vector<int> cands;
          cands.reserve(it->second.size());
          for (const int idx : it->second) {
            if (seen.find(idx) == seen.end()) cands.push_back(idx);
          }
          if (!cands.empty()) {
            const int matched_idx =
                (has_ref_center && cands.size() > 1) ? chooseNearestByRef(cands, ref_center) : cands.front();
            if (matched_idx >= 0 && seen.insert(matched_idx).second) {
              selected_indices.push_back(matched_idx);
              matched_exact_center = true;
            }
          }
        }
      }

      if (task_gid < 0) continue;
      auto it = grid_to_indices.find(task_gid);
      if (it == grid_to_indices.end()) continue;

      vector<int> cands;
      cands.reserve(it->second.size());
      for (const int idx : it->second) {
        if (seen.find(idx) == seen.end()) cands.push_back(idx);
      }
      if (cands.empty()) continue;

      const vector<int> preferred = preferUnknownCenters(cands);
      const bool grid_has_split_centers = hasExplicitSplitCenter(it->second, center_hulls);

      if (matched_exact_center) {
        continue;
      }

      if (single_token_grid && !grid_has_split_centers) {
        for (const int idx : preferred) {
          if (idx >= 0 && seen.insert(idx).second) {
            selected_indices.push_back(idx);
          }
        }
        continue;
      }

      int matched_idx = preferred.front();
      if (has_ref_center && preferred.size() > 1) {
        matched_idx = chooseNearestByRef(preferred, ref_center);
      }
      if (matched_idx >= 0 && seen.insert(matched_idx).second) {
        selected_indices.push_back(matched_idx);
      }
    }

    for (const int gid : info.extra_grid_ids) {
      if (gid < 0) continue;
      auto it = grid_to_indices.find(gid);
      if (it == grid_to_indices.end()) continue;

      vector<int> cands;
      cands.reserve(it->second.size());
      for (const int idx : it->second) {
        if (seen.find(idx) == seen.end()) cands.push_back(idx);
      }
      if (cands.empty()) continue;

      const vector<int> preferred = preferUnknownCenters(cands);
      const bool single_token_grid = (info.grid_task_count.count(gid) > 0 &&
                                      info.grid_task_count.at(gid) == 1 &&
                                      global_grid_task_count.count(gid) > 0 &&
                                      global_grid_task_count.at(gid) == 1);
      const bool grid_has_split_centers = hasExplicitSplitCenter(it->second, center_hulls);
      if (grid_has_split_centers) {
        // Bare grid tokens are not allowed to claim arbitrary sibling split parts.
        continue;
      }
      if (single_token_grid) {
        for (const int idx : preferred) {
          if (idx >= 0 && seen.insert(idx).second) {
            selected_indices.push_back(idx);
          }
        }
        continue;
      }

      const int matched_idx = preferred.front();
      if (matched_idx >= 0 && seen.insert(matched_idx).second) {
        selected_indices.push_back(matched_idx);
      }
    }
  }
}

void C2ExplorationManager::collectAssignedHullTasks(
    const shared_ptr<HGrid>& hgrid, const vector<DroneState>* states,
    vector<HGrid::HullTask>& hull_tasks) {
  hull_tasks.clear();
  if (!hgrid || states == nullptr) return;
  for (const auto& state : *states) {
    vector<int> assigned_center_grid_ids = state.center_grid_ids_;
    if (assigned_center_grid_ids.size() < state.center_positions_.size()) {
      assigned_center_grid_ids.resize(state.center_positions_.size(), -1);
    }
    for (int i = 0; i < static_cast<int>(state.center_positions_.size()); ++i) {
      if (assigned_center_grid_ids[i] >= 0) continue;
      int gid = -1;
      if (hgrid->getGridIdByCenterPos(state.center_positions_[i], gid, 1.0)) {
        assigned_center_grid_ids[i] = gid;
      }
    }

    const int task_num = std::min(static_cast<int>(assigned_center_grid_ids.size()),
        static_cast<int>(state.center_hulls_.size()));
    for (int i = 0; i < task_num; ++i) {
      if (assigned_center_grid_ids[i] < 0 || state.center_hulls_[i].empty()) continue;
      HGrid::HullTask task;
      task.grid_id = assigned_center_grid_ids[i];
      task.hull = state.center_hulls_[i];
      hull_tasks.push_back(std::move(task));
    }
  }
}

void C2ExplorationManager::keepCenterSubset(const vector<int>& keep_indices, const int drone_num, Eigen::MatrixXd& mat,
    vector<int>& center_grid_ids, vector<Vector3d>& center_positions,
    vector<vector<Vector3d>>& center_hulls, vector<int>& center_ids, vector<int>& center_types) {
  const int filtered_center_num = static_cast<int>(keep_indices.size());
  Eigen::MatrixXd filtered_mat =
      Eigen::MatrixXd::Zero(1 + drone_num + filtered_center_num, 1 + drone_num + filtered_center_num);
  for (int i = 0; i < 1 + drone_num; ++i) {
    for (int j = 0; j < 1 + drone_num; ++j) {
      filtered_mat(i, j) = mat(i, j);
    }
  }
  for (int i = 0; i < filtered_center_num; ++i) {
    const int src_i = 1 + drone_num + keep_indices[i];
    for (int j = 0; j < 1 + drone_num; ++j) {
      filtered_mat(j, 1 + drone_num + i) = mat(j, src_i);
      filtered_mat(1 + drone_num + i, j) = mat(src_i, j);
    }
    for (int j = 0; j < filtered_center_num; ++j) {
      const int src_j = 1 + drone_num + keep_indices[j];
      filtered_mat(1 + drone_num + i, 1 + drone_num + j) = mat(src_i, src_j);
    }
  }

  vector<int> filtered_center_grid_ids;
  vector<Vector3d> filtered_center_positions;
  vector<vector<Vector3d>> filtered_center_hulls;
  vector<int> filtered_center_ids;
  vector<int> filtered_center_types;
  filtered_center_grid_ids.reserve(filtered_center_num);
  filtered_center_positions.reserve(filtered_center_num);
  filtered_center_hulls.reserve(filtered_center_num);
  filtered_center_ids.reserve(filtered_center_num);
  filtered_center_types.reserve(filtered_center_num);
  for (const int idx : keep_indices) {
    filtered_center_grid_ids.push_back(center_grid_ids[idx]);
    filtered_center_positions.push_back(center_positions[idx]);
    filtered_center_hulls.push_back(center_hulls[idx]);
    filtered_center_ids.push_back(center_ids[idx]);
    filtered_center_types.push_back(center_types[idx]);
  }

  mat.swap(filtered_mat);
  center_grid_ids.swap(filtered_center_grid_ids);
  center_positions.swap(filtered_center_positions);
  center_hulls.swap(filtered_center_hulls);
  center_ids.swap(filtered_center_ids);
  center_types.swap(filtered_center_types);
}

}  // namespace c2_expl
