#include <active_perception/frontier_finder.h>

#include <algorithm>
#include <iterator>
#include <cmath>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>

#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>

namespace c2_expl {
namespace {
// perception_utils.* names are also declared by PerceptionUtils, which this
// constructor instantiates a few lines below on the same node -
// declare_parameter() throws on a duplicate name, so guard it.
template <typename T>
void declareIfNew(rclcpp::Node::SharedPtr node, const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) node->declare_parameter(name, default_value);
}
}  // namespace

FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, rclcpp::Node::SharedPtr node) {
  this->edt_env_ = edt;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  declareIfNew(node, "frontier.cluster_min", -1);
  declareIfNew(node, "frontier.cluster_size_xy", -1.0);
  declareIfNew(node, "frontier.min_candidate_dist", -1.0);
  declareIfNew(node, "frontier.min_candidate_clearance", -1.0);
  declareIfNew(node, "frontier.candidate_dphi", -1.0);
  declareIfNew(node, "frontier.candidate_rmax", -1.0);
  declareIfNew(node, "frontier.candidate_rmin", -1.0);
  declareIfNew(node, "frontier.candidate_rnum", -1);
  declareIfNew(node, "frontier.down_sample", -1);
  declareIfNew(node, "frontier.min_visib_num", -1);
  declareIfNew(node, "frontier.min_view_finish_fraction", -1.0);
  node->get_parameter("frontier.cluster_min", cluster_min_);
  node->get_parameter("frontier.cluster_size_xy", cluster_size_xy_);
  node->get_parameter("frontier.min_candidate_dist", min_candidate_dist_);
  node->get_parameter("frontier.min_candidate_clearance", min_candidate_clearance_);
  node->get_parameter("frontier.candidate_dphi", candidate_dphi_);
  node->get_parameter("frontier.candidate_rmax", candidate_rmax_);
  node->get_parameter("frontier.candidate_rmin", candidate_rmin_);
  node->get_parameter("frontier.candidate_rnum", candidate_rnum_);
  node->get_parameter("frontier.down_sample", down_sample_);
  node->get_parameter("frontier.min_visib_num", min_visib_num_);
  node->get_parameter("frontier.min_view_finish_fraction", min_view_finish_fraction_);
  // Vincoli sui PUNTI DI VISTA, opzionali (0 / valori estremi = disattivati):
  //  - viewpoint_obstacle_clearance: distanza minima del punto di vista dalle
  //    celle OCCUPATE. min_candidate_clearance non la copre: misura la distanza
  //    dallo SCONOSCIUTO (isNearUnknown). Senza, l'unico controllo sugli
  //    ostacoli era il gonfiaggio della mappa, e un drone fermo sul punto di
  //    vista poteva avere le eliche contro il muro.
  //  - viewpoint_min_z / viewpoint_max_z: fascia di quota dei punti di vista.
  //    Venivano campionati alla quota della frontiera, quindi a pochi cm dal
  //    pavimento per le frontiere basse, mentre il ponte verso PX4 non scende
  //    sotto min_flight_z: un bersaglio irraggiungibile per costruzione, che
  //    nella run V1_GPS 5 ha prodotto 337 ripianificazioni "collision detected"
  //    con traiettorie a 0.3 m da terra. Da 1 m il lidar (50 gradi verso il
  //    basso) vede comunque il pavimento attorno al velivolo.
  declareIfNew(node, "frontier.viewpoint_obstacle_clearance", 0.0);
  declareIfNew(node, "frontier.viewpoint_min_z", -1e9);
  declareIfNew(node, "frontier.viewpoint_max_z", 1e9);
  node->get_parameter("frontier.viewpoint_obstacle_clearance", vp_obstacle_clearance_);
  node->get_parameter("frontier.viewpoint_min_z", vp_min_z_);
  node->get_parameter("frontier.viewpoint_max_z", vp_max_z_);

  declareIfNew(node, "perception_utils.lidar_top_angle", -1.0);
  declareIfNew(node, "perception_utils.lidar_bottom_angle", -1.0);
  declareIfNew(node, "perception_utils.lidar_max_dist", -1.0);
  declareIfNew(node, "perception_utils.lidar_pitch", 0.0);
  node->get_parameter("perception_utils.lidar_top_angle", lidar_fov_up_);
  node->get_parameter("perception_utils.lidar_bottom_angle", lidar_fov_down_);
  node->get_parameter("perception_utils.lidar_max_dist", lidar_max_dist_);
  node->get_parameter("perception_utils.lidar_pitch", lidar_pitch_);

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  percep_utils_.reset(new PerceptionUtils(node));
}

FrontierFinder::~FrontierFinder() {
}

void FrontierFinder::searchFrontiers() {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, false);

  // Bounding box of external chunks
  vector<Eigen::Vector3d> chunk_mins, chunk_maxs;
  edt_env_->sdf_map_->mm_->getChunkBoxes(chunk_mins, chunk_maxs, false);

  vector<Eigen::Vector3d> mins, maxs;
  mins.push_back(update_min);
  mins.insert(mins.end(), chunk_mins.begin(), chunk_mins.end());
  maxs.push_back(update_max);
  maxs.insert(maxs.end(), chunk_maxs.begin(), chunk_maxs.end());

  // Removed changed frontiers in updated map
  auto resetFlag = [&](vector<Frontier>::iterator& iter, vector<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (const auto& cell : iter->cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      int adr = toadr(idx);
      if (adr >= 0 && adr < frontier_flag_.size()) {
        frontier_flag_[adr] = 0;
      }
    }
    iter = frontiers.erase(iter);
  };

  // ROS_INFO("[Frontier] Before remove: %lu", frontiers_.size());

  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    // haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max)
    if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) && isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx;
      ++iter;
    }
  }
  // ROS_INFO("[Frontier] After remove: %lu", frontiers_.size());
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) && isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);

  vector<Eigen::Vector3d> search_mins, search_maxs;
  for (int i = 0; i < static_cast<int>(mins.size()); ++i) {
    search_mins.push_back(mins[i] - Vector3d(1, 1, 0.2));
    search_maxs.push_back(maxs[i] + Vector3d(1, 1, 0.2));
    for (int k = 0; k < 3; ++k) {
      search_mins[i][k] = max(search_mins[i][k], box_min[k]);
      search_maxs[i][k] = min(search_maxs[i][k], box_max[k]);
    }
  }
  vector<Eigen::Vector3i> min_ids(mins.size()), max_ids(mins.size());
  for (int i = 0; i < static_cast<int>(mins.size()); ++i) {
    edt_env_->sdf_map_->posToIndex(search_mins[i], min_ids[i]);
    edt_env_->sdf_map_->posToIndex(search_maxs[i], max_ids[i]);
  }

  for (int i = 0; i < static_cast<int>(min_ids.size()); ++i) {
    auto min_id = min_ids[i];
    auto max_id = max_ids[i];

    for (int z = min_id(2); z <= max_id(2); ++z)
      for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y) {
          // Scanning the updated region to find seeds of frontiers
          Eigen::Vector3i cur(x, y, z);
          int cur_adr = toadr(cur);
          // isInBox anche sul SEME, non solo sull'espansione (piu' sotto): la
          // mappa e' piu' grande della exploration box, quindi una cella libera
          // fuori box con un vicino ignoto generava comunque una frontiera.
          // Indoor e' quasi solo spreco - quelle frontiere finiscono dormienti
          // perche' sampleViewpoints scarta i candidati fuori box - ma all'aperto
          // la box e' l'UNICO confine: senza questo controllo il cielo e il
          // terreno oltre il bordo diventerebbero una sorgente inesauribile di
          // frontiere, e l'esplorazione non potrebbe concludersi per costruzione.
          if (cur_adr >= 0 && cur_adr < frontier_flag_.size() && frontier_flag_[cur_adr] == 0 &&
              edt_env_->sdf_map_->isInBox(cur) && knownfree(cur) && isNeighborUnknown(cur)) {
            // Expand from the seed cell to find a complete frontier cluster
            expandFrontier(cur);
          }
        }
  }

  splitLargeFrontiers(tmp_frontiers_);
}

void FrontierFinder::expandFrontier(
    const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */) {

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  int first_adr = toadr(first);
  if (first_adr >= 0 && first_adr < frontier_flag_.size()) {
    frontier_flag_[first_adr] = 1;
  }

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      int adr = toadr(nbr);
      // Add boundary check to prevent segmentation fault
      if (adr < 0 || adr >= frontier_flag_.size()) continue;
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
          !(knownfree(nbr) && isNeighborUnknown(nbr)))
        continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 1.0 || pos[2] > 1.5) continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if (expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
}

void FrontierFinder::splitLargeFrontiers(vector<Frontier>& frontiers) {
  vector<Frontier> splits, tmps;
  for (const auto& frontier : frontiers) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(frontier, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(frontier);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, vector<Frontier>& splits) {
  // Split a frontier into small piece if it is too large
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (const auto& cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  // Compute principal component
  // Covariance matrix of cells
  Eigen::Matrix2d cov;
  cov.setZero();
  for (const auto& cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvector
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);
  // std::cout << "max idx: " << max_idx << std::endl;
  // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() <<
  // std::endl;

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (const auto& cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  // Degenerate split: every cell landed on one side (can happen with a
  // near-collinear or otherwise ill-conditioned cell distribution, where
  // dot(first_pc) doesn't actually separate the cluster). computeFrontierInfo
  // dereferences cells_.front() unconditionally and segfaults on an empty
  // cluster; treat this as "not splittable" instead of recursing forever
  // into copies of the same undivided frontier.
  if (ftr1.cells_.empty() || ftr2.cells_.empty()) return false;
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  vector<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

void FrontierFinder::updateFrontierCostMatrix() {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  if (!removed_ids_.empty()) {
    // Delete path and cost for removed clusters
    auto it_first_new = frontiers_.begin() + first_new_ftr_;
    for (auto it = frontiers_.begin(); it != it_first_new; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (int i = 0; i < static_cast<int>(removed_ids_.size()); ++i) {
        // Step iterator to the item to be removed
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
    }
    removed_ids_.clear();
  }

  auto updateCost = [](const vector<Frontier>::iterator& it1,
                        const vector<Frontier>::iterator& it2) {
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    double cost_ij = ViewNode::computeCost(
        vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  // Compute path and cost between old and new clusters
  auto it_first_new = frontiers_.begin() + first_new_ftr_;
  for (auto it1 = frontiers_.begin(); it1 != it_first_new; ++it1)
    for (auto it2 = it_first_new; it2 != frontiers_.end(); ++it2) updateCost(it1, it2);

  // Compute path and cost between new clusters
  for (auto it1 = it_first_new; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      } else
        updateCost(it1, it2);
    }
  first_new_ftr_ = frontiers_.size();
}

bool FrontierFinder::haveOverlap(
    const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

bool FrontierFinder::haveAnyOverlap(const Vector3d& min1, const Vector3d& max1,
    const vector<Vector3d>& mins, const vector<Vector3d>& maxs) {

  for (int i = 0; i < static_cast<int>(mins.size()); ++i) {
    if (haveOverlap(min1, max1, mins[i], maxs[i])) return true;
  }
  return false;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  for (const auto& cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  // Compute average position and bounding box of cluster
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (const auto& cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());

  // Compute downsampled cluster
  downsample(ftr.cells_, ftr.filtered_cells_);
}

void FrontierFinder::computeFrontiersToVisit() {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  first_new_ftr_ = frontiers_.size();
  int new_num = 0;
  int new_dormant_num = 0;
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpoints(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      frontiers_.push_back(tmp_ftr);
      auto& inserted = frontiers_.back();
      // Sort the viewpoints by coverage fraction, best view in front
      sort(inserted.viewpoints_.begin(), inserted.viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
    } else {
      // Find no viewpoint, move cluster to dormant list
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }
  // Reset indices of frontiers
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
  }
  // Diagnostica: perche' i campioni di punto di vista vengono scartati.
  // Cumulativa dall'avvio, una riga ogni 10 s.
  {
    static rclcpp::Clock orologio(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("frontier_finder"), orologio, 10000,
        "[PuntiDiVista] campioni %ld: fuori box %ld, gonfiaggio %ld, vicino allo sconosciuto %ld, "
        "vicino a ostacoli %ld, poco visibili %ld | frontiere attive %zu dormienti %zu",
        scarti_[0], scarti_[1], scarti_[2], scarti_[3], scarti_[4], scarti_[5], frontiers_.size(),
        dormant_frontiers_.size());
  }
  // ROS_INFO(
  //     "[Frontier] Frontier num: %lu, dormant: %lu", frontiers_.size(),
  //     dormant_frontiers_.size());
}

void FrontierFinder::getTopViewpointsInfo(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points,
    vector<double>& yaws, vector<Eigen::Vector3d>& averages, vector<Viewpoint>& viewpoints) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  points.clear();
  yaws.clear();
  averages.clear();
  viewpoints.clear();

  auto appendViewpoint = [&](const Frontier& frontier, const Viewpoint& view) {
    points.push_back(view.pos_);
    yaws.push_back(view.yaw_);
    averages.push_back(frontier.average_);
    viewpoints.push_back(view);
  };

  auto pickFresh = [&](Frontier& frontier) {
    bool no_view = true;
    for (const auto& view : frontier.viewpoints_) {
      if (!isViewpointValid(view.pos_)) continue;
      // Retrieve the first viewpoint that is far enough and has highest coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      appendViewpoint(frontier, view);
      no_view = false;
      break;
    }
    if (no_view) {
      for (const auto& view : frontier.viewpoints_) {
        if (!isViewpointValid(view.pos_)) continue;
        // All non-stale viewpoints are very close, just use the best-coverage one of those.
        appendViewpoint(frontier, view);
        no_view = false;
        break;
      }
    }
    return no_view;
  };

  for (auto& frontier : frontiers_) {
    // Viewpoints are sampled once when the frontier is first discovered (sampleViewpoints,
    // checked against inflated occupancy at that time) and then cached for the frontier's
    // whole lifetime - they're never re-sampled as later scans refine the map. On a cluttered
    // real scene the raw occupancy grid can mark every cached viewpoint occupied well after
    // they were sampled (inflated occupancy lags the raw grid, or later scans reveal the
    // candidate was actually inside thin geometry), which used to make this function silently
    // hand back a known-occupied point that fails A* every single time it's picked - a
    // livelock, since the frontier's cache never gets invalidated on its own. If every cached
    // candidate is currently stale-occupied, re-sample this frontier against the live map
    // before giving up on it.
    bool no_view = pickFresh(frontier);
    if (no_view) {
      frontier.viewpoints_.clear();
      sampleViewpoints(frontier);
      sort(frontier.viewpoints_.begin(), frontier.viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
      if (!frontier.viewpoints_.empty()) no_view = pickFresh(frontier);
    }
    if (no_view) {
      // Still nothing valid (frontier is now fully boxed in by real geometry) - fall back to
      // the best-coverage cached viewpoint so callers always get a result, same as before.
      appendViewpoint(frontier, frontier.viewpoints_.empty() ? Viewpoint{ frontier.average_, 0.0, 0 }
                                                               : frontier.viewpoints_.front());
    }
  }
}

void FrontierFinder::getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids,
    const int& view_num, const double& max_decay, vector<vector<Eigen::Vector3d>>& points,
    vector<vector<double>>& yaws) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  points.clear();
  yaws.clear();
  for (auto id : ids) {
    if (id < 0 || id >= static_cast<int>(frontiers_.size())) continue;

    const auto& frontier = frontiers_[id];
    if (frontier.viewpoints_.empty()) continue;

    // Get several top viewpoints that are far enough
    vector<Eigen::Vector3d> pts;
    vector<double> ys;
    int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
    for (const auto& view : frontier.viewpoints_) {
      if (static_cast<int>(pts.size()) >= view_num || view.visib_num_ <= visib_thresh) break;
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      pts.push_back(view.pos_);
      ys.push_back(view.yaw_);
    }
    if (pts.empty()) {
      // All viewpoints are very close, ignore the distance limit
      for (const auto& view : frontier.viewpoints_) {
        if (static_cast<int>(pts.size()) >= view_num || view.visib_num_ <= visib_thresh) break;
        pts.push_back(view.pos_);
        ys.push_back(view.yaw_);
      }
    }
    points.push_back(pts);
    yaws.push_back(ys);
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  clusters.clear();
  for (const auto& frontier : frontiers_) clusters.push_back(frontier.cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  clusters.clear();
  for (const auto& ft : dormant_frontiers_) clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  boxes.clear();
  for (const auto& frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

void FrontierFinder::getPathForTour(
    const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  if (frontier_ids.empty()) return;
  auto valid_frontier_id = [&](int id) {
    return id >= 0 && id < static_cast<int>(frontiers_.size());
  };

  // Compute the path from current pos to the first frontier
  const int first_id = frontier_ids.front();
  if (!valid_frontier_id(first_id) || frontiers_[first_id].viewpoints_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("frontier_finder"), "[Frontier] Invalid first frontier id %d for tour path extraction", first_id);
    return;
  }
  vector<Vector3d> segment;
  ViewNode::searchPath(pos, frontiers_[first_id].viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());

  // Get paths of tour passing all clusters
  for (int i = 0; i < static_cast<int>(frontier_ids.size()) - 1; ++i) {
    const int from_id = frontier_ids[i];
    const int to_id = frontier_ids[i + 1];
    if (!valid_frontier_id(from_id) || !valid_frontier_id(to_id)) {
      RCLCPP_WARN(rclcpp::get_logger("frontier_finder"), "[Frontier] Skip invalid edge (%d -> %d) in frontier tour", from_id, to_id);
      continue;
    }
    if (frontiers_[from_id].viewpoints_.empty() || frontiers_[to_id].viewpoints_.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("frontier_finder"), "[Frontier] Skip frontier edge (%d -> %d) due to empty viewpoint list", from_id,
          to_id);
      continue;
    }

    const auto& cached_paths = frontiers_[from_id].paths_;
    if (to_id < 0 || to_id >= static_cast<int>(cached_paths.size())) {
      // Fallback to online search when cached path matrix is inconsistent with frontier ids.
      vector<Vector3d> fallback_segment;
      ViewNode::searchPath(frontiers_[from_id].viewpoints_.front().pos_,
          frontiers_[to_id].viewpoints_.front().pos_, fallback_segment);
      path.insert(path.end(), fallback_segment.begin(), fallback_segment.end());
      continue;
    }
    auto path_iter = cached_paths.begin();
    for (int step = 0; step < to_id && path_iter != cached_paths.end(); ++step) {
      ++path_iter;
    }
    if (path_iter == cached_paths.end()) {
      vector<Vector3d> fallback_segment;
      ViewNode::searchPath(frontiers_[from_id].viewpoints_.front().pos_,
          frontiers_[to_id].viewpoints_.front().pos_, fallback_segment);
      path.insert(path.end(), fallback_segment.begin(), fallback_segment.end());
      continue;
    }
    const auto& cached_segment = *path_iter;
    path.insert(path.end(), cached_segment.begin(), cached_segment.end());
  }
}

void FrontierFinder::getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double>& yaws, Eigen::MatrixXd& mat) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);


  const int drone_num = positions.size();
  const int ftr_num = frontiers_.size();
  const int dimen = 1 + drone_num + ftr_num;
  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = 0;     // depot to drone: allow starting from drone origins.
    mat(1 + i, 0) = 1000;  // drone to depot: positive cost strongly discourages returning to drone origins.
  }
  // Virtual depot to frontiers
  for (int i = 0; i < ftr_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;  // depot to frontier: discourage starting directly from a frontier.
    mat(1 + drone_num + i, 0) = 0;     // frontier to depot: allow finishing at the depot with zero cost.
  }
  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;  // Drone-to-drone transitions get a huge cost, effectively disabled.
    }
  }

  // Costs from drones to frontiers
  for (int i = 0; i < drone_num; ++i) {
    int j = 0;
    for (const auto& ftr : frontiers_) {
      const Viewpoint& vj = ftr.viewpoints_.front();
      vector<Vector3d> path;
      mat(1 + i, 1 + drone_num + j) =
          ViewNode::computeCost(positions[i], vj.pos_, yaws[0], vj.yaw_, velocities[i], 0.0, path);
      mat(1 + drone_num + j, 1 + i) = 0;
      ++j;
    }
  }
  // Costs between frontiers
  // costs_ deve restare allineato a frontiers_: se una frontiera conserva piu'
  // costi delle frontiere vive (rimozione non propagata), j sfonda il bordo
  // della matrice e Eigen scrive oltre il buffer. Il danno non si vede qui: si
  // manifesta piu' tardi come segfault in tutt'altro punto, su memoria adiacente.
  int i = 0;
  for (const auto& ftr : frontiers_) {
    const int cost_num = static_cast<int>(ftr.costs_.size());
    if (cost_num != ftr_num) {
      RCLCPP_WARN(rclcpp::get_logger("frontier_finder"),
          "[Frontier] Costi disallineati per la frontiera %d: %d costi contro %d frontiere", i,
          cost_num, ftr_num);
    }
    int j = 0;
    for (const auto& cs : ftr.costs_) {
      if (j >= ftr_num) break;  // oltre questo indice si scrive fuori dalla matrice
      mat(1 + drone_num + i, 1 + drone_num + j) = cs;
      ++j;
    }
    ++i;
  }
  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }

}

// Cost matrix for the frontier subset inside a grid.
void FrontierFinder::getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double>& yaws, const vector<int>& ftr_ids,
    const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat) {
  // Il lucchetto copre TUTTO il corpo, non solo la chiamata annidata: gli id in
  // ftr_ids indicizzano frontiers_, e se le frontiere vengono ricostruite fra la
  // costruzione di full_mat e il suo uso quegli id puntano a righe inesistenti.
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  Eigen::MatrixXd full_mat;
  // Build the full cost matrix for lookup.
  getSwarmCostMatrix(positions, velocities, yaws, full_mat);

  // Ogni id va verificato prima di essere usato come indice: un id stantio
  // legge fuori dai bordi di full_mat.
  const int ftr_total = static_cast<int>(frontiers_.size());
  for (const int id : ftr_ids) {
    if (id < 0 || id >= ftr_total) {
      RCLCPP_WARN(rclcpp::get_logger("frontier_finder"),
          "[Frontier] Id di frontiera fuori intervallo (%d, frontiere vive %d): matrice dei costi "
          "non costruita",
          id, ftr_total);
      mat.resize(0, 0);
      return;
    }
  }

  // Get part of the full matrix according to selected frontier

  const int drone_num = positions.size();
  const int ftr_num = ftr_ids.size();
  int dimen = 1 + drone_num + ftr_num;
  if (!grid_pos.empty()) dimen += 1;

  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = 0;
    mat(1 + i, 0) = 1000;
  }
  // Virtual depot to frontiers
  for (int i = 0; i < ftr_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }
  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  // Costs from drones to frontiers
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < ftr_num; ++j) {
      mat(1 + i, 1 + drone_num + j) = full_mat(1 + i, 1 + drone_num + ftr_ids[j]);
      mat(1 + drone_num + j, 1 + i) = 0;
    }
  }
  // Costs between frontiers
  for (int i = 0; i < ftr_num; ++i) {
    for (int j = 0; j < ftr_num; ++j) {
      mat(1 + drone_num + i, 1 + drone_num + j) =
          full_mat(1 + drone_num + ftr_ids[i], 1 + drone_num + ftr_ids[j]);
    }
  }
  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }

  // Consider next grid in global tour
  if (!grid_pos.empty()) {
    // Depot
    mat(0, 1 + drone_num + ftr_num) = 1000;
    mat(1 + drone_num + ftr_num, 0) = 0;

    // Drone
    for (int i = 0; i < drone_num; ++i) {
      mat(1 + i, 1 + drone_num + ftr_num) = 1000;
      mat(1 + drone_num + ftr_num, 1 + i) = 1000;
    }

    // Frontier
    vector<Eigen::Vector3d> points, tmps;
    vector<double> yaws;
    vector<Viewpoint> vps;
    getTopViewpointsInfo(positions[0], points, yaws, tmps, vps);
    Eigen::Vector3d next_grid = grid_pos[0];

    // points salta le frontiere senza punti di vista validi, quindi puo' essere
    // piu' corto di frontiers_: gli id validati sopra non bastano a garantirlo.
    const int point_num = static_cast<int>(points.size());
    for (int i = 0; i < ftr_num; ++i) {
      if (ftr_ids[i] >= point_num) {
        RCLCPP_WARN(rclcpp::get_logger("frontier_finder"),
            "[Frontier] Nessun punto di vista per l'id %d (punti disponibili %d): costo verso la "
            "cella successiva scoraggiato",
            ftr_ids[i], point_num);
        mat(1 + drone_num + i, 1 + drone_num + ftr_num) = 1000;
        mat(1 + drone_num + ftr_num, 1 + drone_num + i) = 1000;
        continue;
      }
      double cost = ViewNode::computeCost(
          next_grid, points[ftr_ids[i]], 0, 0, Eigen::Vector3d(0, 0, 0), 0, tmps);
      mat(1 + drone_num + i, 1 + drone_num + ftr_num) = cost;
      mat(1 + drone_num + ftr_num, 1 + drone_num + i) = cost;
    }
  }
}

// Sample viewpoints around frontier's average position, check coverage to the frontier cells
void FrontierFinder::sampleViewpoints(Frontier& frontier) {

  // Evaluate sample viewpoints on circles, find ones that cover most cells
  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
      rc <= candidate_rmax_ + 1e-3; rc += dr)
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);
      sample_pos[2] = std::min(std::max(sample_pos[2], vp_min_z_), vp_max_z_);

      // Qualified viewpoint is in bounding box and in safe region.
      // Ogni scarto e' contato per motivo (vedi scarti_), per poter dire QUALE
      // vincolo rende dormienti le frontiere.
      ++scarti_[0];
      if (!edt_env_->sdf_map_->isInBox(sample_pos)) { ++scarti_[1]; continue; }
      if (edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1) { ++scarti_[2]; continue; }
      if (isNearUnknown(sample_pos)) { ++scarti_[3]; continue; }
      if (isNearObstacle(sample_pos)) { ++scarti_[4]; continue; }
      if (isBlacklisted(sample_pos)) { ++scarti_[4]; continue; }

      // Compute average yaw
      auto& cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (int i = 1; i < static_cast<int>(cells.size()); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);
      // Compute the fraction of covered and visible cells
      int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
      if (visib_num <= min_visib_num_) ++scarti_[5];
      if (visib_num > min_visib_num_) {
        Viewpoint vp = { sample_pos, avg_yaw, visib_num };
        frontier.viewpoints_.push_back(vp);
      }
    }
}

bool FrontierFinder::banFrontier(const Eigen::Vector3d& average) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it) {
    if ((it->average_ - average).norm() > 1e-3) continue;
    removeFrontierAligned(static_cast<size_t>(it - frontiers_.begin()));
    return true;
  }
  return false;
}

// Rimozione di una frontiera che tiene ALLINEATI i costi delle altre.
//
// Ogni frontiera conserva in costs_/paths_ un elemento per ciascuna frontiera
// della lista, nello stesso ordine. La rimozione diretta (com'era in
// banFrontier) lasciava quei costi al loro posto: da li' in poi il j-esimo costo
// di ogni frontiera si riferiva alla frontiera sbagliata. Lo segnalava l'avviso
// "Costi disallineati" nei log, e il giro fra le frontiere veniva calcolato su
// distanze scambiate.
void FrontierFinder::removeFrontierAligned(size_t k) {
  if (k >= frontiers_.size()) return;
  for (size_t i = 0; i < frontiers_.size(); ++i) {
    if (i == k) continue;
    auto& f = frontiers_[i];
    if (k < f.costs_.size()) f.costs_.erase(std::next(f.costs_.begin(), k));
    if (k < f.paths_.size()) f.paths_.erase(std::next(f.paths_.begin(), k));
  }
  dormant_frontiers_.push_back(frontiers_[k]);
  frontiers_.erase(frontiers_.begin() + k);
  // Gli indici sono usati come chiavi altrove (points_/yaws_ sono allineati a
  // questo ordine), quindi vanno rinumerati subito dopo la rimozione.
  int idx = 0;
  for (auto& ft : frontiers_) ft.id_ = idx++;
  first_new_ftr_ = std::min<int>(first_new_ftr_, static_cast<int>(frontiers_.size()));
}

// Un punto di vista e' valido se il velivolo ci puo' stare: dentro il box,
// fuori dalla fascia di gonfiaggio e lontano dagli ostacoli quanto richiesto.
bool FrontierFinder::isViewpointValid(const Eigen::Vector3d& pos) {
  auto& map = edt_env_->sdf_map_;
  return map->isInBox(pos) && map->getOccupancy(pos) != SDFMap::OCCUPIED &&
         map->getInflateOccupancy(pos) == 0 && !isNearObstacle(pos) && !isBlacklisted(pos);
}

// LISTA NERA dei punti di vista irraggiungibili.
//
// Un punto di vista puo' essere libero, lontano dagli ostacoli e con la
// frontiera in vista, e tuttavia stare in una sacca senza accesso: spazio
// osservato attraverso una fessura in cui il velivolo non passa. Con l'anello
// di campionamento allargato a 2 m capita spesso (run V6_LIO 1: 146 s di blocco,
// punto di vista oltre la parete cw106). Il pianificatore lo riprovava finche'
// non scattava l'archiviazione dopo 30 fallimenti, e la frontiera poi rinasceva
// con lo stesso punto. Qui il punto viene escluso al PRIMO fallimento vero:
// alla prossima revisione la frontiera usa il punto successivo o viene
// ricampionata altrove.
void FrontierFinder::blacklistViewpoint(const Eigen::Vector3d& pos) {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);
  if (isBlacklisted(pos)) return;
  vp_blacklist_.push_back(pos);
  if (vp_blacklist_.size() > 2000) vp_blacklist_.erase(vp_blacklist_.begin());
}

bool FrontierFinder::isBlacklisted(const Eigen::Vector3d& pos) const {
  for (const auto& b : vp_blacklist_)
    if ((b - pos).squaredNorm() < 0.3 * 0.3) return true;
  return false;
}

// RICONTROLLO DEI PUNTI DI VISTA a ogni aggiornamento delle frontiere.
//
// I punti di vista vengono campionati quando la frontiera nasce e poi tenuti
// in memoria per tutta la sua vita. Se nel frattempo la mappa rivela un muro
// li' accanto, il punto resta in lista anche se ora cade nella fascia di
// gonfiaggio: il pianificatore non trova mai un percorso per arrivarci e
// ripianifica a vuoto all'infinito ("No path to next viewpoint"). E' la causa
// dei blocchi misurati: 66 s nella run V2_GPS 4 con il punto di vista a 6 cm
// dalla parete cw106, dalla parte opposta rispetto al drone, e 616 s nella run
// BASE_LIO 4.
// Qui si scartano i punti non piu' validi; se non ne resta nessuno la frontiera
// viene ricampionata sulla mappa attuale, e se nemmeno cosi' ha un punto di
// vista possibile passa fra le dormienti invece di restare un bersaglio
// irraggiungibile.
int FrontierFinder::revalidateViewpoints() {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);
  int dormienti = 0;
  for (size_t k = 0; k < frontiers_.size();) {
    auto& vps = frontiers_[k].viewpoints_;
    vps.erase(std::remove_if(vps.begin(), vps.end(),
                  [this](const Viewpoint& v) { return !isViewpointValid(v.pos_); }),
        vps.end());
    if (vps.empty()) {
      sampleViewpoints(frontiers_[k]);
      sort(vps.begin(), vps.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
    }
    if (vps.empty()) {
      removeFrontierAligned(k);
      ++dormienti;
      continue;
    }
    ++k;
  }
  return dormienti;
}

bool FrontierFinder::isFrontierCovered() {
  std::lock_guard<std::recursive_mutex> lk(ftr_mutex_);

  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

  // Bounding box of external chunks
  vector<Eigen::Vector3d> chunk_mins, chunk_maxs;
  edt_env_->sdf_map_->mm_->getChunkBoxes(chunk_mins, chunk_maxs, false);

  vector<Eigen::Vector3d> mins, maxs;
  mins.push_back(update_min);
  mins.insert(mins.end(), chunk_mins.begin(), chunk_mins.end());
  maxs.push_back(update_max);
  maxs.insert(maxs.end(), chunk_maxs.begin(), chunk_maxs.end());

  auto checkChanges = [&](const vector<Frontier>& frontiers) {
    for (const auto& ftr : frontiers) {
      // haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)
      if (!haveAnyOverlap(ftr.box_min_, ftr.box_max_, mins, maxs)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (const auto& cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}

// Una cella OCCUPATA entro vp_obstacle_clearance_ in orizzontale (e +-0.2 m in
// verticale, la fascia dell'ingombro del velivolo). Si legge l'occupazione e
// non l'ESDF: l'ESDF e' aggiornato solo attorno al drone e altrove vale
// default_dist, che renderebbe il controllo cieco o bloccante a seconda del
// valore.
bool FrontierFinder::isNearObstacle(const Eigen::Vector3d& pos) {
  if (vp_obstacle_clearance_ <= 0.0) return false;
  const int vox_num = static_cast<int>(std::ceil(vp_obstacle_clearance_ / resolution_));
  const double r2 = vp_obstacle_clearance_ * vp_obstacle_clearance_;
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y) {
      const double dx = x * resolution_, dy = y * resolution_;
      if (dx * dx + dy * dy > r2) continue;
      for (int z = -2; z <= 2; ++z) {
        const Eigen::Vector3d vox(pos[0] + dx, pos[1] + dy, pos[2] + z * resolution_);
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::OCCUPIED) return true;
      }
    }
  return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN) return true;
      }
  return false;
}

int FrontierFinder::countVisibleCells(
    const Eigen::Vector3d& pos, const double& yaw, const vector<Eigen::Vector3d>& cluster) {
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (const auto& cell : cluster) {
    // Check if frontier cell is inside FOV
    // if (!percep_utils_->insideFOV(cell)) continue;
    if (!isInLidarFOV(pos, yaw, cell)) continue;

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 ||
          edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
        visib = false;
        break;
      }
    }
    if (visib) visib_num += 1;
  }
  return visib_num;
}

void FrontierFinder::downsample(
    const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& cell : cluster_in) cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (const auto& pt : cloudf->points) cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI) yaw += 2 * M_PI;
  while (yaw > M_PI) yaw -= 2 * M_PI;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  // At least one neighbor is unknown
  auto nbrs = tenNeighbors(voxel);
  for (const auto& nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN &&
        edt_env_->sdf_map_->isInMap(nbr))
      return true;
  }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

bool FrontierFinder::isInLidarFOV(
    const Eigen::Vector3d& vp_pos, const double& vp_yaw, const Vector3d& frt_cell) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  // transform.rotate(Eigen::AngleAxisd(-lidar_roll_, Eigen::Vector3d::UnitX()));
  transform.rotate(Eigen::AngleAxisd(-lidar_pitch_, Eigen::Vector3d::UnitY()));
  transform.rotate(Eigen::AngleAxisd(-vp_yaw, Eigen::Vector3d::UnitZ()));

  if ((vp_pos - frt_cell).norm() > lidar_max_dist_) return false;
  Eigen::Vector3d pt2see = transform * (frt_cell - vp_pos);
  // pitch
  float pitch = atan2(pt2see.z(), sqrt(pt2see.x() * pt2see.x() + pt2see.y() * pt2see.y()));
  if (pitch > lidar_fov_up_ || pitch < -lidar_fov_down_) return false;

  return true;
}

}  // namespace c2_expl
