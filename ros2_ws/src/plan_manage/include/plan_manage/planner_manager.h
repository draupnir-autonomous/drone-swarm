#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>

#include <plan_env/edt_environment.h>

#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>

#include <plan_manage/plan_container.hpp>

#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace c2_expl {
// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  bool kinodynamicReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel,
      const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_pt,
      const Eigen::Vector3d& end_vel, const double& time_lb = -1);
  void planExploreTraj(const vector<Eigen::Vector3d>& tour, const Eigen::Vector3d& cur_vel,
      const Eigen::Vector3d& cur_acc, const double& time_lb = -1);
  bool planGlobalTraj(const Eigen::Vector3d& start_pos);

  void planYaw(const Eigen::Vector3d& start_yaw);
  void planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw, bool lookfwd,
      const double& relax_time);

  void initPlanModules(rclcpp::Node::SharedPtr node);
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

  bool checkTrajCollision(double& distance);
  // Stesso controllo di checkTrajCollision sulla traiettoria locale appena
  // pianificata, a partire dall'istante t_da in cui il server la prendera'.
  bool trajLiberaDa(double t_da);
  // Frenata in linea retta da (p, v) fino a fermarsi, con decelerazione
  // costante 'decel': sostituisce la traiettoria locale (posizione; l'imbardata
  // va pianificata a parte).
  void planStop(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double decel);
  // Traiettoria di riserva che percorre la spezzata 'punti' FERMANDOSI a ogni
  // vertice (profilo a trapezio, velocita' v, accelerazione a), al massimo per
  // lunghezza_max metri: non taglia gli angoli. Sostituisce la posizione della
  // traiettoria locale (l'imbardata va pianificata a parte).
  void planPolilinea(const vector<Eigen::Vector3d>& punti, double v, double a, double lunghezza_max);
  void calcNextYaw(const double& last_yaw, double& yaw);
  // Punteggio di un'imbardata in un punto della traiettoria (superfici non
  // ancora fotografate che la telecamera inquadrerebbe da li'). Se impostato,
  // planYawExplore orienta la telecamera verso di esse invece di guardare
  // sempre avanti: il lidar vede a 360 gradi, l'imbardata conta solo per lei.
  std::function<double(const Eigen::Vector3d&, double)> punteggio_imbardata_;

  // False when this drone's committed trajectory comes within
  // manager.swarm_clearance of the peer's, sweeping both forward in time.
  bool checkSwarmCollision(const int& drone_id);
  // Same check against every peer whose trajectory we currently hold. Returns
  // the id of the first conflicting drone, or -1 when clear.
  int firstSwarmConflict();
  // Vero se il compagno id ha pubblicato una traiettoria "ferma" (punti di controllo tutti
  // nello stesso posto): e' in attesa del via, in sosta o ha finito, cioe' non ripianifica.
  bool swarmTrajStatica(const int& id);

  PlanParameters pp_;
  LocalTrajData local_data_;
  // Guards local_data_'s trajectory fields (position_traj_/duration_/start_time_/...).
  // checkTrajCollision() runs from its own callback group specifically so it can never
  // be starved by a slow replan (see the FSM's crash_check_timer_ comment for the
  // incident that motivated this) - which means it can now run concurrently with the
  // planXxx()/updateTrajInfo() functions below that mutate local_data_ in place
  // (NonUniformBspline::setUniformBspline resizes its internal control-point matrix,
  // so a concurrent evaluateDeBoorT() read mid-resize is undefined behavior, not just a
  // stale-value read).
  std::mutex local_data_mutex_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  unique_ptr<Astar> path_finder_;
  unique_ptr<KinodynamicAstar> kino_path_finder_;

  SwarmData swarm_traj_data_;  // Data added for swarm trajectory planning
  // Guards swarm_traj_data_ for the same reason local_data_mutex_ guards
  // local_data_: the FSM writes peer trajectories from the default callback
  // group (swarmTrajCallback) while checkSwarmCollision reads them from
  // safetyCallback in the reentrant input group, and setUniformBspline resizes
  // the control-point matrix under the reader's feet.
  std::mutex swarm_traj_mutex_;
  // Minimum horizontal separation between two drones' trajectories, in metres.
  double swarm_clearance_ = 0.5;
  // Age past which a peer's last broadcast trajectory stops being trusted.
  double swarm_traj_max_age_ = 3.0;

private:
  /* main planning algorithms & modules */

  rclcpp::Node::SharedPtr node_;
  shared_ptr<SDFMap> sdf_map_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  void updateTrajInfo();
  void enforceDynamicLimits();
  // Primo punto della traiettoria (da t_da, entro raggio_max di percorso) che
  // entra nella fascia di gonfiaggio; vedi checkTrajCollision. Va chiamata con
  // local_data_mutex_ preso.
  bool collisioneDa(double t_da, double raggio_max, double& distanza, Eigen::Vector3d& dove);
  double dyn_limit_tol_ = 1.15;

  Eigen::MatrixXd paramLocalTraj(double start_t, double& dt, double& duration);

  // Heading planning

  // !SECTION stable

public:
  typedef shared_ptr<FastPlannerManager> Ptr;

private:
  unique_ptr<FrontierFinder> frontier_finder_;
  unique_ptr<HeadingPlanner> heading_planner_;
};
}  // namespace c2_expl

#endif
