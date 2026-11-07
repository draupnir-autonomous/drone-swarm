#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>
#include <list>
#include <utility>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

class RayCaster;

namespace c2_expl {
class EDTEnvironment;
class PerceptionUtils;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;
};

class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, rclcpp::Node::SharedPtr node);
  ~FrontierFinder();

  void searchFrontiers();
  void computeFrontiersToVisit();

  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws,
      vector<Vector3d>& averages, vector<Viewpoint>& viewpoints);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num,
      const double& max_decay, vector<vector<Vector3d>>& points, vector<vector<double>>& yaws);
  void updateFrontierCostMatrix();
  void getSwarmCostMatrix(const vector<Vector3d>& positions, const vector<Vector3d>& velocities,
      const vector<double>& yaws, const vector<int>& ftr_ids,
      const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat);

  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  bool isFrontierCovered();
  // Archivia come dormiente la frontiera il cui baricentro coincide con
  // 'average'. Serve a chi osserva l'ESITO di una visita: una frontiera puo'
  // avere punti di vista perfettamente validi e restare comunque inosservabile
  // (celle in intercapedini piu' strette del velivolo, viste solo di scorcio),
  // e in quel caso il campionamento dei punti di vista non ha modo di
  // accorgersene. Restituisce true se l'ha trovata e spostata.
  bool banFrontier(const Eigen::Vector3d& average);

  shared_ptr<PerceptionUtils> percep_utils_;

private:
  void splitLargeFrontiers(vector<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, vector<Frontier>& splits);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(
      const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2);
  bool haveAnyOverlap(const Vector3d& min1, const Vector3d& max1, const vector<Vector3d>& mins,
      const vector<Vector3d>& maxs);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);

  int countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster);
  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);
  bool isInLidarFOV(const Eigen::Vector3d& vp_pos, const double& vp_yaw, const Vector3d& frt_cell);
  void wrapYaw(double& yaw);
  void getSwarmCostMatrix(const vector<Vector3d>& positions, const vector<Vector3d>& velocities,
      const vector<double>& yaws, Eigen::MatrixXd& mat);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);

  // Data
  vector<char> frontier_flag_;
  vector<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;

  // Protegge le liste di frontiere. Il percorso di pianificazione le LEGGE
  // (getSwarmCostMatrix, getPathForTour, getTopViewpointsInfo) mentre il
  // callback delle frontiere le RICOSTRUISCE: da qui il segfault in
  // findTourOfFrontier -> getSwarmCostMatrix.
  // Sta qui e non nei chiamanti perche' i chiamanti sono piu' d'uno e uno solo
  // teneva il lucchetto dell'FSM. Ricorsivo perche' questi metodi si richiamano
  // fra loro. Le sezioni restano corte: si e' gia' visto che serializzare
  // operazioni lunghe (2.4 s del grafo di connettivita') manda in stallo l'FSM.
  mutable std::recursive_mutex ftr_mutex_;
  vector<int> removed_ids_;
  int first_new_ftr_;  // Changed from iterator to index

  // Params
  int cluster_min_;
  double cluster_size_xy_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  double vp_obstacle_clearance_ = 0.0, vp_min_z_ = -1e9, vp_max_z_ = 1e9;
  // Campioni di punto di vista: totale, fuori box, gonfiaggio, vicino allo
  // sconosciuto, vicino a ostacoli, poco visibili.
  long scarti_[6] = { 0, 0, 0, 0, 0, 0 };
  bool isNearObstacle(const Eigen::Vector3d& pos);
  bool isViewpointValid(const Eigen::Vector3d& pos);
  void removeFrontierAligned(size_t k);
public:
  // Ricontrolla i punti di vista di tutte le frontiere contro la mappa
  // attuale; restituisce quante frontiere sono finite fra le dormienti.
  int revalidateViewpoints();
  // Punto di vista per cui la ricerca di percorso e' fallita davvero (non per
  // tempo scaduto): non va piu' proposto.
  void blacklistViewpoint(const Eigen::Vector3d& pos);
private:
  std::vector<Eigen::Vector3d> vp_blacklist_;
  bool isBlacklisted(const Eigen::Vector3d& pos) const;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;
  double lidar_fov_up_, lidar_fov_down_, lidar_max_dist_, lidar_pitch_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
};

}  // namespace c2_expl
#endif
