// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

#include <algorithm>
#include <cmath>

namespace c2_expl {
// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {
}

FastPlannerManager::~FastPlannerManager() {
  std::cout << "des manager" << std::endl;
}

void FastPlannerManager::initPlanModules(rclcpp::Node::SharedPtr node) {
  node_ = node;
  /* read algorithm parameters */

  pp_.max_vel_ = node_->declare_parameter("manager.max_vel", -1.0);
  pp_.max_acc_ = node_->declare_parameter("manager.max_acc", -1.0);
  pp_.max_jerk_ = node_->declare_parameter("manager.max_jerk", -1.0);
  pp_.accept_vel_ = node_->declare_parameter("manager.accept_vel", pp_.max_vel_ + 0.5);
  pp_.accept_acc_ = node_->declare_parameter("manager.accept_acc", pp_.max_acc_ + 0.5);
  pp_.max_yawdot_ = node_->declare_parameter("manager.max_yawdot", -1.0);
  pp_.dynamic_ = node_->declare_parameter("manager.dynamic_environment", -1);
  pp_.local_traj_len_ = node_->declare_parameter("manager.local_segment_length", -1.0);
  pp_.ctrl_pt_dist = node_->declare_parameter("manager.control_points_distance", -1.0);
  pp_.bspline_degree_ = node_->declare_parameter("manager.bspline_degree", 3);
  pp_.min_time_ = node_->declare_parameter("manager.min_time", false);
  pp_.relax_time1_ = node_->declare_parameter("manager.relax_time1", 0.5);
  pp_.relax_time2_ = node_->declare_parameter("manager.relax_time2", 0.5);
  // Margine sui limiti dinamici prima di rallentare la traiettoria (vedi
  // enforceDynamicLimits): 1.15 = si tollera il 15% oltre max_vel/max_acc.
  dyn_limit_tol_ = node_->declare_parameter("manager.dyn_limit_tolerance", 1.15);
  // Horizontal separation checkSwarmCollision enforces between committed
  // trajectories. Was a hardcoded 0.5m, which is roughly the x500's own rotor
  // span - i.e. it only declared a conflict once the two drones were already
  // overlapping. Keep it BELOW optimization.swarm_safe_dist: the optimizer's
  // swarm term is what normally holds the drones apart, and this is the
  // backstop for when it failed. Set it above and the backstop fires while the
  // optimizer is satisfied, so the drones yield to each other endlessly.
  swarm_clearance_ = node_->declare_parameter("manager.swarm_clearance", 0.5);
  // A peer trajectory older than this is dropped rather than kept forever - see
  // SwarmData::expireStaleTrajs. Peers broadcast at 10Hz in every state, so a
  // couple of seconds of silence really does mean they are gone.
  swarm_traj_max_age_ = node_->declare_parameter("manager.swarm_traj_max_age", 3.0);

  bool use_geometric_path = node_->declare_parameter("manager.use_geometric_path", false);
  bool use_kinodynamic_path = node_->declare_parameter("manager.use_kinodynamic_path", false);
  bool use_optimization = node_->declare_parameter("manager.use_optimization", false);
  bool use_active_perception = node_->declare_parameter("manager.use_active_perception", false);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(node_);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    path_finder_.reset(new Astar);
    // path_finder_->setParam(node_);
    // path_finder_->setEnvironment(edt_environment_);
    // path_finder_->init();
    path_finder_->init(node_, edt_environment_);
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(node_);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(node_);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_active_perception) {
    frontier_finder_.reset(new FrontierFinder(edt_environment_, node_));
    // heading_planner_.reset(new HeadingPlanner(node_));
    // heading_planner_->setMap(sdf_map_);
    // plan_data_.view_cons_.idx_ = -1;
  }
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {
  std::lock_guard<std::mutex> lock(local_data_mutex_);
  const double t_now = (node_->now() - local_data_.start_time_).seconds();
  Eigen::Vector3d dove;
  if (collisioneDa(t_now, 6.0, distance, dove)) {
    std::cout << "collision at: " << dove.transpose() << std::endl;
    return false;
  }
  return true;
}

// Regola comune al controllo periodico (checkTrajCollision) e a quello dei
// piani nuovi (trajLiberaDa): un punto della traiettoria entro raggio_max di
// percorso che cade nella fascia di gonfiaggio e' una collisione.
//
// UNICA ECCEZIONE: se la traiettoria PARTE gia' dentro la fascia (drone finito
// a ridosso di un muro, tipicamente dopo una frenata), il primo tratto e'
// accettato finche' non si avvicina all'ostacolo piu' di quanto lo fosse alla
// partenza: e' la via d'uscita. Senza, qualunque traiettoria da li' risultava
// "in collisione" fin dal primo punto e il controllo ripianificava a raffica.
// Uscito dalla fascia, la regola torna quella di sempre.
bool FastPlannerManager::collisioneDa(
    double t_da, double raggio_max, double& distanza, Eigen::Vector3d& dove) {
  NonUniformBspline& traj = local_data_.position_traj_;  // evaluateDeBoorT non e' const
  const double T = local_data_.duration_;
  t_da = std::max(0.0, t_da);
  const Eigen::Vector3d p0 = traj.evaluateDeBoorT(std::min(t_da, T));
  bool in_uscita = sdf_map_->getInflateOccupancy(p0) == 1;
  const double d_partenza = in_uscita ? sdf_map_->getDistance(p0) : 0.0;
  double raggio = 0.0;
  for (double t = t_da + 0.02; raggio < raggio_max && t < T; t += 0.02) {
    const Eigen::Vector3d p = traj.evaluateDeBoorT(t);
    const bool gonfiato = sdf_map_->getInflateOccupancy(p) == 1;
    if (in_uscita) {
      if (!gonfiato) {
        in_uscita = false;
      } else if (sdf_map_->getDistance(p) < d_partenza - 0.05) {
        distanza = raggio;
        dove = p;
        return true;
      }
    } else if (gonfiato) {
      distanza = raggio;
      dove = p;
      return true;
    }
    raggio = (p - p0).norm();
  }
  return false;
}

bool FastPlannerManager::trajLiberaDa(double t_da) {
  std::lock_guard<std::mutex> lock(local_data_mutex_);
  double distanza;
  Eigen::Vector3d dove;
  return !collisioneDa(t_da, 6.0, distanza, dove);
}

// PERCORSO SULLA SPEZZATA, FERMANDOSI AI VERTICI. Riserva per quando il piano
// ottimizzato, partendo da fermi, taglia un angolo ed entra nella fascia di
// gonfiaggio: i tratti rettilinei del percorso accorciato sono gia' verificati
// liberi (shortenPath), e fermandosi a ogni vertice la curva non taglia gli
// angoli. Lento (0.5 m/s) ma senza vicoli: senza questa riserva un drone fermo
// dentro la stanza a "П" di cubicle_office ha fallito 30 piani di fila (35 s),
// perche' ogni traiettoria verso l'uscita sfiorava lo spigolo (V33).
void FastPlannerManager::planPolilinea(
    const vector<Eigen::Vector3d>& punti, double v, double a, double lunghezza_max) {
  struct Tratto {
    Eigen::Vector3d A, dir;
    double L, ta, vp, T, t0;
  };
  vector<Tratto> tratti;
  double percorso = 0.0, t = 0.0;
  for (size_t k = 0; k + 1 < punti.size() && percorso < lunghezza_max - 1e-3; ++k) {
    double L = (punti[k + 1] - punti[k]).norm();
    if (L < 1e-3) continue;
    L = std::min(L, lunghezza_max - percorso);
    Tratto tr;
    tr.A = punti[k];
    tr.dir = (punti[k + 1] - punti[k]).normalized();
    tr.L = L;
    tr.ta = v / a;
    tr.vp = v;
    if (a * tr.ta * tr.ta > L) {  // niente tratto a velocita' costante
      tr.ta = std::sqrt(L / a);
      tr.vp = a * tr.ta;
    }
    tr.T = 2.0 * tr.ta + (L - tr.vp * tr.ta) / tr.vp;
    tr.t0 = t;
    t += tr.T;
    percorso += L;
    tratti.push_back(tr);
  }
  const Eigen::Vector3d inizio = punti.front();
  auto pos = [&](double tq) -> Eigen::Vector3d {
    for (const auto& tr : tratti) {
      if (tq > tr.t0 + tr.T) continue;
      const double u = std::max(0.0, tq - tr.t0);
      double s;
      if (u < tr.ta) s = 0.5 * a * u * u;
      else if (u < tr.T - tr.ta) s = 0.5 * a * tr.ta * tr.ta + tr.vp * (u - tr.ta);
      else {
        const double r = tr.T - u;
        s = tr.L - 0.5 * a * r * r;
      }
      return tr.A + tr.dir * std::min(s, tr.L);
    }
    return tratti.empty() ? inizio : Eigen::Vector3d(tratti.back().A + tratti.back().dir * tratti.back().L);
  };
  const double durata = std::max(t, 0.5);
  const int seg = std::max(8, static_cast<int>(std::ceil(durata / 0.1)));
  const double dt = durata / seg;
  vector<Eigen::Vector3d> campioni, deri(4, Eigen::Vector3d::Zero());
  for (int i = 0; i <= seg; ++i) campioni.push_back(pos(i * dt));
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, campioni, deri, pp_.bspline_degree_, ctrl_pts);
  std::lock_guard<std::mutex> lock(local_data_mutex_);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);
  updateTrajInfo();
}

// FRENATA IN LINEA RETTA. Serve quando la traiettoria in corso e' in
// collisione e nessun piano nuovo ne esce pulito: prima il piano in collisione
// veniva eseguito lo stesso, e ripianificato di nuovo a partire da se' stesso
// ogni 0.25 s, sempre piu' dentro la fascia. Il filtro del ponte fermava allora
// il riferimento a ~0.8 m dalla parete con il drone a 1 m/s, che lo superava di
// 0.3-0.5 m e la toccava (campagna V27, cubicle_office, 2 ribaltamenti su 2 run).
// Frenare sul posto da 1 m/s a 2 m/s2 costa 0.25 m.
void FastPlannerManager::planStop(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double decel) {
  decel = std::max(decel, 0.5);
  const double vn = v.norm();
  const Eigen::Vector3d dir = vn > 1e-3 ? Eigen::Vector3d(v / vn) : Eigen::Vector3d::Zero();
  const double t_stop = vn / decel;
  // Durata minima, perche' la B-spline abbia segmenti di lunghezza sensata
  // anche da quasi fermo; oltre t_stop il punto resta quello d'arrivo.
  const double T = std::max(t_stop, 0.5);
  const int seg = 10;
  const double dt = T / seg;
  vector<Eigen::Vector3d> points, deri;
  for (int i = 0; i <= seg; ++i) {
    const double t = std::min(i * dt, t_stop);
    points.push_back(p + dir * (vn * t - 0.5 * decel * t * t));
  }
  deri.push_back(v);
  deri.push_back(Eigen::Vector3d::Zero());
  deri.push_back(-decel * dir);
  deri.push_back(Eigen::Vector3d::Zero());
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, points, deri, pp_.bspline_degree_, ctrl_pts);
  std::lock_guard<std::mutex> lock(local_data_mutex_);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);
  updateTrajInfo();
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(const Eigen::Vector3d& start_pt,
    const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
    const Eigen::Vector3d& end_pt, const Eigen::Vector3d& end_vel, const double& time_lb) {
  // std::cout << "[Kino replan]: start: " << start_pt.transpose() << ", " << start_vel.transpose()
  // << ", "
  //           << start_acc.transpose() << ", goal:" << end_pt.transpose() << ", " <<
  //           end_vel.transpose()
  //           << endl;

  if ((start_pt - end_pt).norm() < 1e-2) {
    cout << "Close goal" << endl;
    return false;
  }

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // Kinodynamic path searching

  auto t1 = node_->now();

  kino_path_finder_->reset();
  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
  if (status == KinodynamicAstar::NO_PATH) {
    RCLCPP_INFO(node_->get_logger(), "search 1 fail");
    // Retry
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[Kino replan]: Can't find path." << endl;
      return false;
    }
  }
  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  double t_search = (node_->now() - t1).seconds();
  t1 = node_->now();

  // Parameterize path to B-spline
  double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  // std::cout << "point set:" << std::endl;
  // for (auto pt : point_set) std::cout << pt.transpose() << std::endl;
  // std::cout << "derivative:" << std::endl;
  // for (auto dr : start_end_derivatives) std::cout << dr.transpose() << std::endl;

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      ts, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline init(ctrl_pts, pp_.bspline_degree_, ts);

  // B-spline-based optimization
  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_function |= BsplineOptimizer::MINTIME;
  vector<Eigen::Vector3d> start, end;
  init.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  // Swarm collision avoidance term
  cost_function |= BsplineOptimizer::SWARM;
  vector<NonUniformBspline> trajs;
  {
    std::lock_guard<std::mutex> swarm_lock(swarm_traj_mutex_);
    swarm_traj_data_.expireStaleTrajs(node_->now().seconds(), swarm_traj_max_age_);
    swarm_traj_data_.getValidTrajs(trajs);
  }
  bspline_optimizers_[0]->setSwarmTrajs(trajs);

  bspline_optimizers_[0]->optimize(ctrl_pts, ts, cost_function, 1, 1);
  {
    std::lock_guard<std::mutex> lock(local_data_mutex_);
    local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);
    updateTrajInfo();
  }

  vector<Eigen::Vector3d> start2, end2;
  local_data_.position_traj_.getBoundaryStates(2, 0, start2, end2);
  // std::cout << "State error: (" << (start2[0] - start[0]).norm() << ", "
  //           << (start2[1] - start[1]).norm() << ", " << (start2[2] - start[2]).norm() << ")"
  //           << std::endl;

  double t_opt = (node_->now() - t1).seconds();
  // RCLCPP_INFO(node_->get_logger(), "Kino t: %lf, opt: %lf", t_search, t_opt);

  return true;
}

void FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d>& tour,
    const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc, const double& time_lb) {
  if (tour.empty()) RCLCPP_ERROR(node_->get_logger(), "Empty path to traj planner");

  // Generate traj through waypoints-based method
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = tour[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  PolynomialTraj init_traj;
  PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

  // B-spline-based optimization
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
  seg_num = max(8, seg_num);
  double dt = duration / double(seg_num);

  // std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt <<
  // std::endl;

  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
    points.push_back(init_traj.evaluate(ts, 0));
  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_func |= BsplineOptimizer::MINTIME;

  vector<Vector3d> start, end;
  tmp_traj.getBoundaryStates(2, 0, start, end);
  start[0] = tour[0];
  start[1] = cur_vel;
  start[2] = cur_acc;
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  // Swarm collision avoidance term
  cost_func |= BsplineOptimizer::SWARM;
  vector<NonUniformBspline> trajs;
  {
    std::lock_guard<std::mutex> swarm_lock(swarm_traj_mutex_);
    swarm_traj_data_.expireStaleTrajs(node_->now().seconds(), swarm_traj_max_age_);
    swarm_traj_data_.getValidTrajs(trajs);
  }
  bspline_optimizers_[0]->setSwarmTrajs(trajs);

  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  {
    std::lock_guard<std::mutex> lock(local_data_mutex_);
    local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);
    updateTrajInfo();
  }
}

// !SECTION

// SECTION global planning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  // Generate global reference trajectory
  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

  // Insert intermediate points if two waypoints are too far
  vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();
    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;
      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }
  inter_points.push_back(points.back());

  // At least 3 waypoints are required to solve the problem
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  time(0) += pp_.max_vel_ / (2 * pp_.max_acc_);
  time(time.rows() - 1) += pp_.max_vel_ / (2 * pp_.max_acc_);

  PolynomialTraj gl_traj;
  PolynomialTraj::waypointsTraj(pos, zero, zero, zero, zero, time, gl_traj);

  auto time_now = node_->now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // truncate a local trajectory

  double dt, duration;
  Eigen::MatrixXd ctrl_pts = paramLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, pp_.bspline_degree_, dt);

  std::cout << "ctrl pt: " << ctrl_pts.rows() << std::endl;

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  {
    std::lock_guard<std::mutex> lock(local_data_mutex_);
    local_data_.position_traj_ = bspline;
    local_data_.start_time_ = time_now;
    updateTrajInfo();
  }
  RCLCPP_INFO(node_->get_logger(), "global trajectory generated.");

  return true;
}

// RISPETTO DEI LIMITI DINAMICI, in ogni traiettoria che esce dal pianificatore.
//
// L'ottimizzatore tratta velocita' e accelerazione massime come una PENALITA'
// (ld_feasi), non come un vincolo, e nessun passo successivo controllava il
// risultato. Misurato nella run BASE_LIO 2 di office_small: anticipo di
// velocita' a 1.3 m/s con manager.max_vel = 0.6, diretto contro la parete cw106;
// lo strato reattivo e' intervenuto a 0.8 m, troppo tardi per frenare a quella
// velocita', e il drone ha colpito il muro e si e' ribaltato.
// Qui, se la traiettoria supera i limiti (con margine dyn_limit_tol_), la si
// RALLENTA uniformemente scalando i nodi: il percorso geometrico - quello gia'
// verificato contro la mappa - resta identico, cambia solo il tempo con cui lo
// si percorre (velocita' / r, accelerazione / r^2).
// I limiti si leggono dai punti di controllo delle derivate, che per la
// proprieta' di inviluppo convesso maggiorano velocita' e accelerazione vere.
void FastPlannerManager::enforceDynamicLimits() {
  if (pp_.max_vel_ <= 0.0 || pp_.max_acc_ <= 0.0 || dyn_limit_tol_ <= 0.0) return;
  NonUniformBspline& traj = local_data_.position_traj_;
  const double vlim = pp_.max_vel_ * dyn_limit_tol_;
  const double alim = pp_.max_acc_ * dyn_limit_tol_;

  // Picchi VERI della traiettoria, campionati sulla curva ogni 20 ms. Prima si
  // usavano i punti di controllo delle derivate, che ne sono solo un limite
  // superiore (la curva sta nel loro inviluppo convesso): si rallentavano anche
  // traiettorie che il limite non lo superavano.
  auto picchi = [&](double& vmax, double& amax) {
    NonUniformBspline vel = traj.getDerivative();
    NonUniformBspline acc = vel.getDerivative();
    const double T = traj.getTimeSum();
    vmax = 0.0;
    amax = 0.0;
    for (double t = 0.0; t <= T + 1e-6; t += 0.02) {
      vmax = std::max(vmax, vel.evaluateDeBoorT(std::min(t, T)).norm());
      amax = std::max(amax, acc.evaluateDeBoorT(std::min(t, T)).norm());
    }
  };
  double vmax, amax;
  picchi(vmax, amax);
  const double v0 = vmax, a0 = amax;
  double ratio = std::max(vmax / vlim, std::sqrt(amax / alim));
  if (!std::isfinite(ratio) || ratio <= 1.0) return;

  // ALLUNGAMENTO CHE CONSERVA GLI ESTREMI. Riscalare uniformemente tutti i nodi
  // (come si faceva) divide per 'ratio' anche la velocita' INIZIALE: la nuova
  // traiettoria partiva piu' lenta del drone, che a ogni ripianificazione
  // (ogni ~1.5 s) riceveva una frenata secca del 30% in media. E' l'andamento a
  // strappi misurato nel rientro (velocita' fra 0.1 e 0.8 m/s con periodo ~2 s).
  // lengthenTime lascia invariati i primi e gli ultimi 2p-1 nodi, cioe' stato
  // iniziale e finale, e allunga solo la parte centrale. Se il picco cade
  // proprio negli estremi la si ripete, fino a 4 volte.
  const double ratio0 = ratio;
  int passi = 0;
  for (; passi < 4 && ratio > 1.0; ++passi) {
    const Eigen::VectorXd prima = traj.getKnot();
    traj.lengthenTime(ratio * 1.02);
    if ((traj.getKnot() - prima).norm() < 1e-9) break;  // troppo corta per allungarla
    picchi(vmax, amax);
    const double prima_ratio = ratio;
    ratio = std::max(vmax / vlim, std::sqrt(amax / alim));
    // Il picco non scende: sta negli estremi conservati (tipicamente lo stato
    // iniziale, cioe' quello reale del drone). Allungare ancora rallenterebbe
    // solo la parte centrale senza toglierlo.
    if (ratio > prima_ratio * 0.98) break;
  }
  RCLCPP_WARN(node_->get_logger(),
      "[Limiti] picco vel %.2f m/s, acc %.2f m/s2 contro %.2f/%.2f: parte centrale allungata "
      "(%.2f volte, %d passi), ora vel %.2f acc %.2f",
      v0, a0, pp_.max_vel_, pp_.max_acc_, ratio0, passi, vmax, amax);
}

void FastPlannerManager::updateTrajInfo() {
  enforceDynamicLimits();
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();

  local_data_.traj_id_ += 1;
}

Eigen::MatrixXd FastPlannerManager::paramLocalTraj(double start_t, double& dt, double& duration) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;
  global_data_.getTrajInfoInSphere(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
      start_end_derivative, dt, duration);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(
      dt, point_set, start_end_derivative, pp_.bspline_degree_, ctrl_pts);

  return ctrl_pts;
}
// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  auto t1 = node_->now();
  // calculate waypoints of heading

  auto& pos = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  const double forward_t = 2.0;
  double last_yaw = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS |
                  BsplineOptimizer::START | BsplineOptimizer::END;

  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw[0], 0, 0),
    Eigen::Vector3d(start_yaw[1], 0, 0), Eigen::Vector3d(start_yaw[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw[0], 0, 0),
    Eigen::Vector3d(end_yaw[1], 0, 0), Eigen::Vector3d(end_yaw[2], 0, 0) };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, pp_.bspline_degree_, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_ = path_yaw;
  plan_data_.dt_yaw_ = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "yaw time: " << (node_->now() - t1).seconds() << std::endl;
}

// only consider smoothness and boundary state
void FastPlannerManager::planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw,
    bool lookfwd, const double& relax_time) {
  const int seg_num = 12;
  double dt_yaw = local_data_.duration_ / double(seg_num);  // time of B-spline segment

  // double dt_yaw = 0.5;
  // int seg_num = ceil(local_data_.duration_ / dt_yaw);
  // seg_num = max(5, seg_num);  // Restrict the minimal num
  // dt_yaw = local_data_.duration_ / double(seg_num);

  Eigen::Vector3d start_yaw3d = start_yaw;
  // std::cout << "dt_yaw: " << dt_yaw << ", start yaw: " << start_yaw3d[0] << ", end: " << end_yaw
  //           << std::endl;

  while (start_yaw3d[0] < -M_PI) start_yaw3d[0] += 2 * M_PI;
  while (start_yaw3d[0] > M_PI) start_yaw3d[0] -= 2 * M_PI;
  double last_yaw = start_yaw3d[0];
  int last_idx = 0;

  // Yaw traj control points
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  // Initial state
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  // Add waypoint constraints if look forward is enabled
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;
  if (lookfwd) {
    const double forward_t = 2.0;
    const int relax_num1 = pp_.relax_time1_ / dt_yaw;
    const int relax_num2 = pp_.relax_time2_ / dt_yaw;
    for (int i = 1; i < seg_num - relax_num2; ++i) {
      double tc = i * dt_yaw;
      Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);
      double tf = min(local_data_.duration_, tc + forward_t);
      Eigen::Vector3d pf = local_data_.position_traj_.evaluateDeBoorT(tf);
      Eigen::Vector3d pd = pf - pc;
      if (pd.norm() > 1e-6) {
        // IMBARDATA DA FOTO. Candidati: la direzione di marcia (quella di
        // sempre) e, se c'e' un punteggio, altre 11 a passi di 30 gradi. Vince
        // quella da cui la telecamera inquadrerebbe piu' superfici non ancora
        // fotografate; la direzione di marcia ha un piccolo vantaggio, cosi'
        // dove non c'e' niente da fotografare si guarda avanti come prima.
        // Il lidar vede a 360 gradi: la sicurezza non dipende dall'imbardata.
        // Le direzioni "da foto" stanno entro il 60% della velocita' di
        // imbardata massima (foto piu' nitide, volo piu' tranquillo).
        const double avanti = atan2(pd(1), pd(0));
        const int n_cand = punteggio_imbardata_ ? 12 : 1;
        const double bonus_avanti = 3.0;
        double miglior_punteggio = -1e9;
        Eigen::Vector3d miglior(0, 0, 0);
        bool trovato = false;
        for (int k = 0; k < n_cand; ++k) {
          Eigen::Vector3d waypt(avanti + k * M_PI / 6.0, 0.0, 0.0);
          calcNextYaw(last_yaw, waypt(0));

          // Check if the waypoint satisfy yawdot constraint
          double dt1 = double(i - last_idx) * dt_yaw;
          double dt2 = double(seg_num + 1 - i) * dt_yaw;

          double diff1 = fabs(waypt(0) - last_yaw);
          double yd1 = min(diff1, 2 * M_PI - diff1) / dt1;

          double tmp2 = end_yaw;
          calcNextYaw(waypt(0), tmp2);
          double diff2 = fabs(waypt(0) - tmp2);
          double yd2 = min(diff2, 2 * M_PI - diff2) / dt2;

          const double lim = (k == 0) ? pp_.max_yawdot_ : 0.6 * pp_.max_yawdot_;
          if (!(yd1 < lim && yd2 < lim)) continue;
          double punteggio = punteggio_imbardata_ ? punteggio_imbardata_(pc, waypt(0)) : 0.0;
          if (k == 0) punteggio += bonus_avanti;
          if (punteggio > miglior_punteggio) {
            miglior_punteggio = punteggio;
            miglior = waypt;
            trovato = true;
          }
        }
        if (trovato) {
          last_yaw = miglior(0);
          waypts.push_back(miglior);
          waypt_idx.push_back(i);
        }
      }
    }
  }
  // Final state
  Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
  calcNextYaw(last_yaw, end_yaw3d(0));
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

  // Debug rapid change of yaw
  if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI) {
    // RCLCPP_ERROR(node_->get_logger(), "Yaw change rapidly!");
    std::cout << "start yaw: " << start_yaw3d[0] << ", " << end_yaw3d[0] << std::endl;
  }

  // // Interpolate start and end value for smoothness
  // for (int i = 1; i < seg_num; ++i)
  // {
  //   double tc = i * dt_yaw;
  //   Eigen::Vector3d waypt = (1 - double(i) / seg_num) * start_yaw3d + double(i) / seg_num *
  //   end_yaw3d; std::cout << "i: " << i << ", wp: " << waypt[0] << ", "; calcNextYaw(last_yaw,
  //   waypt(0));
  // }
  // std::cout << "" << std::endl;

  auto t1 = node_->now();

  // Call B-spline optimization solver
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END |
                  BsplineOptimizer::WAYPOINTS;
  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw3d[0], 0, 0),
    Eigen::Vector3d(start_yaw3d[1], 0, 0), Eigen::Vector3d(start_yaw3d[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0) };
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // std::cout << "2: " << (node_->now() - t1).seconds() << std::endl;

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt_yaw;

  // plan_data_.path_yaw_ = path;
  // plan_data_.dt_yaw_path_ = dt_yaw * subsp;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

bool FastPlannerManager::checkSwarmCollision(const int& id) {
  // Check collision with another drone's trajectory
  if (id < 1 || id > swarm_traj_data_.drone_num_) return true;

  std::lock_guard<std::mutex> swarm_lock(swarm_traj_mutex_);
  swarm_traj_data_.expireStaleTrajs(node_->now().seconds(), swarm_traj_max_age_);
  if (swarm_traj_data_.receive_flags_[id - 1] == false) return true;

  auto& traj = swarm_traj_data_.swarm_trajs_[id - 1];

  // Compagno FERMO (traiettoria statica) a quota diversa: e' appoggiato a terra o in attesa
  // sotto/sopra, e un velivolo largo 0.5 m ci passa sopra senza toccarlo. La regola "solo
  // distanza orizzontale" qui sotto vale per chi vola, che non tiene la quota con
  // precisione; per un drone ATTERRATO alla sua partenza voleva dire bloccare la colonna
  // d'aria sopra di lui fino al soffitto: a tre droni su office_small un drone in rientro a
  // 1.8 m ha ceduto il passo per 60 s a compagni gia' appoggiati a 3 m dalla sua partenza ed
  // e' atterrato a meta' strada allo scadere del tempo (sciame, ottobre 2026). Il drone a terra
  // resta un ostacolo della mappa (cloud_transform_bridge non toglie i compagni fermi).
  {
    const Eigen::MatrixXd pts = traj.getControlPoint();
    bool statica = pts.rows() > 0;
    for (int i = 1; i < pts.rows() && statica; ++i)
      if ((pts.row(i) - pts.row(0)).norm() > 0.05) statica = false;
    if (statica) {
      std::lock_guard<std::mutex> lock(local_data_mutex_);
      const double t_ora = std::clamp(
          (node_->now() - local_data_.start_time_).seconds(), 0.0, local_data_.duration_);
      const double z_mio = local_data_.position_traj_.evaluateDeBoorT(t_ora)(2);
      if (std::abs(z_mio - pts(0, 2)) > 0.8) return true;
    }
  }

  std::lock_guard<std::mutex> lock(local_data_mutex_);
  double self_t = 0.02 + (node_->now() - local_data_.start_time_).seconds();
  double other_t = 0.02 + node_->now().seconds() - traj.start_time_;

  while (self_t < local_data_.duration_ && self_t > 0 && other_t < traj.duration_) {
    auto self_pos = local_data_.position_traj_.evaluateDeBoorT(self_t);
    // A peer that is hovering (or has finished) broadcasts a short static
    // trajectory; clamp to its end rather than walking off it, so the peer
    // stays visible for the whole of our own horizon instead of disappearing
    // the moment its trajectory runs out.
    auto other_pos = traj.evaluateDeBoorT(std::min(other_t, traj.duration_));
    // Horizontal separation only: two drones at the same x,y are a conflict
    // whatever their altitudes, since neither holds altitude tightly enough
    // for vertical stacking to be a safe answer.
    if ((self_pos - other_pos).head<2>().norm() < swarm_clearance_) return false;

    self_t += 0.02;
    other_t += 0.02;
  }
  return true;
}

bool FastPlannerManager::swarmTrajStatica(const int& id) {
  if (id < 1 || id > swarm_traj_data_.drone_num_) return false;
  std::lock_guard<std::mutex> swarm_lock(swarm_traj_mutex_);
  if (!swarm_traj_data_.receive_flags_[id - 1]) return false;
  const Eigen::MatrixXd pts = swarm_traj_data_.swarm_trajs_[id - 1].getControlPoint();
  if (pts.rows() == 0) return false;
  const Eigen::RowVector3d primo = pts.row(0);
  for (int i = 1; i < pts.rows(); ++i)
    if ((pts.row(i) - primo).norm() > 0.05) return false;
  return true;
}

int FastPlannerManager::firstSwarmConflict() {
  for (int id = 1; id <= swarm_traj_data_.drone_num_; ++id) {
    if (id == swarm_traj_data_.drone_id_) continue;
    if (!checkSwarmCollision(id)) return id;
  }
  return -1;
}

}  // namespace c2_expl
