#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
#include <plan_env/multi_map_manager.h>
#include <plan_env/raycast.h>

#include <cmath>
#include <limits>

namespace c2_expl {
SDFMap::SDFMap() {
}

SDFMap::~SDFMap() {
}

void SDFMap::initMap(rclcpp::Node::SharedPtr node) {
  mp_.reset(new MapParam);
  md_.reset(new MapData);
  mr_.reset(new MapROS);
  mm_.reset(new MultiMapManager);

  // Params of map properties
  double x_size, y_size, z_size;
  node->declare_parameter("sdf_map.resolution", -1.0);
  node->declare_parameter("sdf_map.map_size_x", -1.0);
  node->declare_parameter("sdf_map.map_size_y", -1.0);
  node->declare_parameter("sdf_map.map_size_z", -1.0);
  // Angolo (x, y) minimo della mappa in coordinate mondo. Di serie (NaN) la mappa e'
  // centrata nell'origine, come e' sempre stata; prepara_mondo.py lo imposta per i
  // mondi la cui zona da esplorare non e' centrata in (0, 0) (ottobre 2026), che
  // altrimenti chiederebbero una mappa grande il doppio della distanza dall'origine.
  node->declare_parameter("sdf_map.map_origin_x", std::numeric_limits<double>::quiet_NaN());
  node->declare_parameter("sdf_map.map_origin_y", std::numeric_limits<double>::quiet_NaN());
  node->declare_parameter("sdf_map.obstacles_inflation", -1.0);
  node->declare_parameter("sdf_map.local_bound_inflate", 1.0);
  node->declare_parameter("sdf_map.local_map_margin", 1);
  node->declare_parameter("sdf_map.esdf_radius", -1.0);
  node->declare_parameter("sdf_map.ground_height", 1.0);
  node->declare_parameter("sdf_map.default_dist", 5.0);
  node->declare_parameter("sdf_map.optimistic", true);
  node->declare_parameter("sdf_map.signed_dist", false);
  node->declare_parameter("sdf_map.no_drone_1", false);
  node->get_parameter("sdf_map.resolution", mp_->resolution_);
  node->get_parameter("sdf_map.map_size_x", x_size);
  node->get_parameter("sdf_map.map_size_y", y_size);
  node->get_parameter("sdf_map.map_size_z", z_size);
  node->get_parameter("sdf_map.obstacles_inflation", mp_->obstacles_inflation_);
  node->get_parameter("sdf_map.local_bound_inflate", mp_->local_bound_inflate_);
  node->get_parameter("sdf_map.local_map_margin", mp_->local_map_margin_);
  node->get_parameter("sdf_map.esdf_radius", mp_->esdf_radius_);
  node->get_parameter("sdf_map.ground_height", mp_->ground_height_);
  node->get_parameter("sdf_map.default_dist", mp_->default_dist_);
  node->get_parameter("sdf_map.optimistic", mp_->optimistic_);
  node->get_parameter("sdf_map.signed_dist", mp_->signed_dist_);
  node->get_parameter("sdf_map.no_drone_1", mp_->no_drone_1_);

  mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
  mp_->resolution_inv_ = 1 / mp_->resolution_;
  double origin_x = node->get_parameter("sdf_map.map_origin_x").as_double();
  double origin_y = node->get_parameter("sdf_map.map_origin_y").as_double();
  if (!std::isfinite(origin_x)) origin_x = -x_size / 2.0;
  if (!std::isfinite(origin_y)) origin_y = -y_size / 2.0;
  mp_->map_origin_ = Eigen::Vector3d(origin_x, origin_y, mp_->ground_height_);
  mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i) mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
  mp_->map_min_boundary_ = mp_->map_origin_;
  mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;

  // Params of raycasting-based fusion
  node->declare_parameter("sdf_map.p_hit", 0.70);
  node->declare_parameter("sdf_map.p_miss", 0.35);
  node->declare_parameter("sdf_map.p_min", 0.12);
  node->declare_parameter("sdf_map.p_max", 0.97);
  node->declare_parameter("sdf_map.p_occ", 0.80);
  node->declare_parameter("sdf_map.max_ray_length", -0.1);
  node->declare_parameter("sdf_map.virtual_ceil_height", -0.1);
  node->get_parameter("sdf_map.p_hit", mp_->p_hit_);
  node->get_parameter("sdf_map.p_miss", mp_->p_miss_);
  node->get_parameter("sdf_map.p_min", mp_->p_min_);
  node->get_parameter("sdf_map.p_max", mp_->p_max_);
  node->get_parameter("sdf_map.p_occ", mp_->p_occ_);
  node->get_parameter("sdf_map.max_ray_length", mp_->max_ray_length_);
  node->get_parameter("sdf_map.virtual_ceil_height", mp_->virtual_ceil_height_);

  auto logit = [](const double& x) { return log(x / (1 - x)); };
  mp_->prob_hit_log_ = logit(mp_->p_hit_);
  mp_->prob_miss_log_ = logit(mp_->p_miss_);
  mp_->clamp_min_log_ = logit(mp_->p_min_);
  mp_->clamp_max_log_ = logit(mp_->p_max_);
  mp_->min_occupancy_log_ = logit(mp_->p_occ_);
  mp_->unknown_flag_ = 0.01;
  RCLCPP_INFO(node->get_logger(),
      "hit: %f, miss: %f, min: %f, max: %f, thresh: %f", mp_->prob_hit_log_, mp_->prob_miss_log_,
      mp_->clamp_min_log_, mp_->clamp_max_log_, mp_->min_occupancy_log_);

  // Initialize data buffer of map
  int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
  md_->occupancy_buffer_local_only_ =
      vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
  md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_->infl_tmp1_ = vector<char>(buffer_size, 0);
  md_->infl_tmp2_ = vector<char>(buffer_size, 0);
  md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_->count_hit_ = vector<short>(buffer_size, 0);
  md_->count_miss_ = vector<short>(buffer_size, 0);
  md_->flag_rayend_ = vector<char>(buffer_size, -1);
  md_->flag_visited_ = vector<char>(buffer_size, -1);
  md_->tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_->tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_->raycast_num_ = 0;
  md_->reset_updated_box_ = true;

  // Try retriving bounding box of map, set box to map size if not specified
  vector<string> axis = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i) {
    node->declare_parameter("sdf_map.box_min_" + axis[i], mp_->map_min_boundary_[i]);
    node->declare_parameter("sdf_map.box_max_" + axis[i], mp_->map_max_boundary_[i]);
    node->get_parameter("sdf_map.box_min_" + axis[i], mp_->box_mind_[i]);
    node->get_parameter("sdf_map.box_max_" + axis[i], mp_->box_maxd_[i]);
  }
  posToIndex(mp_->box_mind_, mp_->box_min_);
  posToIndex(mp_->box_maxd_, mp_->box_max_);

  // Initialize ROS wrapper
  mr_->setMap(this);
  mr_->node_ = node;
  mr_->init();

  mm_->setMap(this);
  mm_->init(node);

  caster_.reset(new RayCaster);
  caster_->setParams(mp_->resolution_, mp_->map_origin_);

  for (int i = 0; i < 3; ++i) {
    md_->all_min_[i] = 1000000;
    md_->all_max_[i] = -1000000;
  }

  // Compute initial vertices, box and normals of map in current drone's frame
  mp_->rot_sw_ << cos(0.0), -sin(0.0), 0, sin(0.0), cos(0.0), 0, 0, 0, 1;
  mp_->trans_sw_ << 0, 0, 0;

  Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
  left_bottom = mp_->box_mind_;
  right_top = mp_->box_maxd_;

  left_top[0] = left_bottom[0];
  left_top[1] = right_top[1];
  left_top[2] = left_bottom[2];
  right_bottom[0] = right_top[0];
  right_bottom[1] = left_bottom[1];
  right_bottom[2] = left_bottom[2];
  right_top[2] = left_bottom[2];

  mp_->vertices_ = { left_bottom, right_bottom, right_top, left_top };
  if (mp_->use_swarm_tf_) {
    for (auto& vert : mp_->vertices_) vert = mp_->rot_sw_ * vert + mp_->trans_sw_;
  }

  mp_->vmin_ = mp_->vmax_ = mp_->vertices_[0];
  for (size_t j = 1; j < mp_->vertices_.size(); ++j) {
    for (int k = 0; k < 2; ++k) {
      mp_->vmin_[k] = min(mp_->vmin_[k], mp_->vertices_[j][k]);
      mp_->vmax_[k] = max(mp_->vmax_[k], mp_->vertices_[j][k]);
    }
  }

  for (int j = 0; j < 4; ++j) {
    Eigen::Vector3d dir = (mp_->vertices_[(j + 1) % 4] - mp_->vertices_[j]).normalized();
    mp_->normals_.push_back(dir);
  }
}

void SDFMap::resetBuffer() {
  resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
  md_->local_bound_min_ = Eigen::Vector3i::Zero();
  md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_->map_voxel_num_(dim)];
  double z[mp_->map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::aggiornaESDFSeServe() {
  mr_->updateESDFCallback();
}

void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = md_->local_bound_min_;
  Eigen::Vector3i max_esdf = md_->local_bound_max_;
  // FINESTRA DELL'ESDF attorno al drone (sdf_map.esdf_radius). Il riquadro
  // aggiornato da una nuvola e' quello dei punti colpiti, cioe' con raggi fino a
  // 20 m quasi tutta la mappa: su cubicle_office l'ESDF costava 0.12 s sul Jetson
  // a ogni nuvola, nello stesso gruppo di callback della pianificazione. L'ESDF
  // lo leggono solo l'ottimizzatore B-spline (traiettorie che partono dal drone e
  // lunghe al massimo radius_far = 7 m, planTrajToView) e il controllo di
  // partenza in planner_manager: fuori dalla finestra restano i valori
  // dell'ultimo calcolo, che nessuno usa.
  if (mp_->esdf_radius_ > 0.0 && md_->have_camera_pos_) {
    Eigen::Vector3i c_min, c_max;
    const Eigen::Vector3d r(mp_->esdf_radius_, mp_->esdf_radius_, 0.0);
    posToIndex(md_->last_camera_pos_ - r, c_min);
    posToIndex(md_->last_camera_pos_ + r, c_max);
    for (int k = 0; k < 2; ++k) {
      min_esdf[k] = std::max(min_esdf[k], c_min[k]);
      max_esdf[k] = std::min(max_esdf[k], c_max[k]);
    }
    if (min_esdf[0] > max_esdf[0] || min_esdf[1] > max_esdf[1]) return;
  }

  if (mp_->optimistic_) {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                         0 :
                         std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  } else {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              int adr = toAddress(x, y, z);
              return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                         md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3) ?
                         0 :
                         std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
          [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
          max_esdf[1], 1);
    }
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
          [&](int x, double val) {
            md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
          },
          min_esdf[0], max_esdf[0], 0);
    }

  if (mp_->signed_dist_) {
    // Compute negative distance
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_->occupancy_buffer_inflate_[x * mp_->map_voxel_num_(1) *
                                                        mp_->map_voxel_num_(2) +
                                                    y * mp_->map_voxel_num_(2) + z] == 0 ?
                         0 :
                         std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
            [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
            max_esdf[1], 1);
      }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
            [&](int x, double val) {
              md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
            },
            min_esdf[0], max_esdf[0], 0);
      }
    // Merge negative distance with positive
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
      for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          if (md_->distance_buffer_neg_[idx] > 0.0)
            md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
        }
  }
}

void SDFMap::setCacheOccupancy(const int& adr, const int& occ) {
  // Add to update list if first visited
  if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0) md_->cache_voxel_.push(adr);

  if (occ == 0)
    md_->count_miss_[adr] = 1;
  else /* if (occ == 1) */
    md_->count_hit_[adr] += 1;
}

void SDFMap::inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
    const Eigen::Vector3d& camera_pos) {
  if (point_num == 0) return;
  // Everything below assumes the sensor sits inside the map: closetPointInMap()
  // intersects the ray with the map box from the inside, and the ray caster
  // walks from the hit point back towards camera_pos. With the sensor outside,
  // both produce indices off the grid and toAddress() - which does no bounds
  // checking at all - turns them into out-of-range offsets. That is a real
  // crash, not a hypothetical: a drone that tumbled past the +-10m map edge
  // took its exploration_node down with it inside setCacheOccupancy
  // ("Invalid permissions for mapped object"). Nothing useful can be fused from
  // out there anyway.
  if (!isInMap(camera_pos)) {
    rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("sdf_map"), clock, 1000,
        "Sensor at (%.2f %.2f %.2f) is outside the map - skipping this cloud", camera_pos[0],
        camera_pos[1], camera_pos[2]);
    return;
  }
  md_->raycast_num_ += 1;
  md_->last_camera_pos_ = camera_pos;
  md_->have_camera_pos_ = true;

  Eigen::Vector3d update_min = camera_pos;
  Eigen::Vector3d update_max = camera_pos;
  if (md_->reset_updated_box_) {
    md_->update_min_ = camera_pos;
    md_->update_max_ = camera_pos;
    md_->reset_updated_box_ = false;
  }

  Eigen::Vector3d pt_w, tmp;
  Eigen::Vector3i idx;
  int vox_adr;
  double length;
  for (int i = 0; i < point_num; ++i) {
    auto& pt = points.points[i];
    pt_w << pt.x, pt.y, pt.z;
    int tmp_flag;
    // Set flag for projected point
    if (!isInMap(pt_w)) {
      // Find closest point in map and set free
      pt_w = closetPointInMap(pt_w, camera_pos);
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_)
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
      if (pt_w[2] < 0.2) continue;
      tmp_flag = 0;
    } else {
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_) {
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
        if (pt_w[2] < 0.2) continue;
        tmp_flag = 0;
      } else
        tmp_flag = 1;
    }
    posToIndex(pt_w, idx);
    // Belt and braces on top of the camera_pos guard above: closetPointInMap()
    // lands exactly on the map boundary, where rounding can still push the
    // index one cell past the last one.
    if (!isInMap(idx)) continue;
    vox_adr = toAddress(idx);
    setCacheOccupancy(vox_adr, tmp_flag);

    for (int k = 0; k < 3; ++k) {
      update_min[k] = min(update_min[k], pt_w[k]);
      update_max[k] = max(update_max[k], pt_w[k]);
    }
    // Raycasting between camera center and point
    if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
      continue;
    else
      md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    caster_->input(pt_w, camera_pos);
    caster_->nextId(idx);
    while (caster_->nextId(idx)) {
      if (!isInMap(idx)) continue;
      setCacheOccupancy(toAddress(idx), 0);
    }
  }

  Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
  posToIndex(update_max + bound_inf, md_->local_bound_max_);
  posToIndex(update_min - bound_inf, md_->local_bound_min_);
  boundIndex(md_->local_bound_min_);
  boundIndex(md_->local_bound_max_);
  mr_->local_updated_ = true;

  // Bounding box for subsequent updating and overall updateing
  for (int k = 0; k < 3; ++k) {
    md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
    md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
    md_->all_min_[k] = min(update_min[k], md_->all_min_[k]);
    md_->all_max_[k] = max(update_max[k], md_->all_max_[k]);
  }

  // Update map data
  vector<uint32_t> new_voxel_ids_;
  while (!md_->cache_voxel_.empty()) {
    int adr = md_->cache_voxel_.front();
    md_->cache_voxel_.pop();
    double log_odds_update =
        md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;
    md_->count_hit_[adr] = md_->count_miss_[adr] = 0;
    if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3) {
      md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;
      new_voxel_ids_.push_back(adr);
    }
    md_->occupancy_buffer_[adr] =
        std::min(std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
            mp_->clamp_max_log_);

    if (md_->occupancy_buffer_local_only_[adr] < mp_->clamp_min_log_ - 1e-3) {
      md_->occupancy_buffer_local_only_[adr] = mp_->min_occupancy_log_;
    }
    md_->occupancy_buffer_local_only_[adr] =
        std::min(std::max(md_->occupancy_buffer_local_only_[adr] + log_odds_update,
                     mp_->clamp_min_log_),
            mp_->clamp_max_log_);
  }

  mm_->updateMapChunk(new_voxel_ids_);
}

Eigen::Vector3d SDFMap::closetPointInMap(
    const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap() {
  int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);

  // Guards occupancy_buffer_inflate_ against concurrent reads from
  // getInflateOccupancy() (see occ_inflate_mutex_'s declaration in sdf_map.h).
  std::lock_guard<std::mutex> lock(occ_inflate_mutex_);

  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // GONFIAGGIO SEPARABILE. Prima: per ogni voxel occupato del riquadro si
  // scriveva il cubo (2*inf_step+1)^3 attorno (343 celle con 0.3 m a 0.1), su
  // tutto il riquadro visto dal lidar e a ogni nuvola: sul Jetson era la parte
  // piu' pesante della fusione, nello stesso gruppo di callback della
  // pianificazione. Il cubo e' il prodotto di tre segmenti, quindi lo stesso
  // insieme di celle si ottiene dilatando lungo z, poi y, poi x (7+7+7 passi per
  // cella toccata invece di 343). Risultato IDENTICO: una cella diventa 1 se e'
  // nella mappa e dista al piu' inf_step (in ogni asse) da un voxel occupato del
  // riquadro; le celle fuori dal riquadro vengono solo messe a 1, come prima.
  const Eigen::Vector3i& vn = mp_->map_voxel_num_;
  auto& t1 = md_->infl_tmp1_;
  auto& t2 = md_->infl_tmp2_;
  auto& l1 = md_->infl_list1_;
  auto& l2 = md_->infl_list2_;
  l1.clear();
  l2.clear();
  // 1) semi occupati del riquadro dilatati lungo z -> t1
  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        int id1 = toAddress(x, y, z);
        if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_) {
          const int z0 = std::max(0, z - inf_step), z1 = std::min(vn(2) - 1, z + inf_step);
          for (int zz = z0; zz <= z1; ++zz) {
            const int a = toAddress(x, y, zz);
            if (!t1[a]) {
              t1[a] = 1;
              l1.push_back(a);
            }
          }
        }
      }
  // 2) dilatazione lungo y -> t2
  Eigen::Vector3i id;
  for (const int a : l1) {
    addressToIndex(a, id);
    t1[a] = 0;
    const int y0 = std::max(0, id(1) - inf_step), y1 = std::min(vn(1) - 1, id(1) + inf_step);
    for (int yy = y0; yy <= y1; ++yy) {
      const int b = toAddress(id(0), yy, id(2));
      if (!t2[b]) {
        t2[b] = 1;
        l2.push_back(b);
      }
    }
  }
  // 3) dilatazione lungo x -> mappa gonfiata
  for (const int b : l2) {
    addressToIndex(b, id);
    t2[b] = 0;
    const int x0 = std::max(0, id(0) - inf_step), x1 = std::min(vn(0) - 1, id(0) + inf_step);
    for (int xx = x0; xx <= x1; ++xx) md_->occupancy_buffer_inflate_[toAddress(xx, id(1), id(2))] = 1;
  }

  // Soffitto virtuale: strato occupato alla quota virtual_ceil_height_, presente
  // solo nella mappa e non nel mondo. Serve dove non c'e' una superficie vera
  // sopra il velivolo - all'aperto i raggi verso l'alto non tornano, e senza un
  // tetto il pianificatore non ha nulla che gli impedisca di salire.
  //
  // Attenzione a cosa fa e cosa NON fa: limita la QUOTA DI VOLO, perche' agisce
  // sul buffer inflazionato che il pianificatore consulta per le collisioni. Non
  // rende "noto" il volume sopra, e quindi non e' cio' che fa terminare
  // l'esplorazione: a delimitare le frontiere e' box_max_z, insieme al controllo
  // isInBox sul seme in FrontierFinder::searchFrontiers.
  // Disabilitato quando il valore sta sotto il pavimento della mappa, che e' il
  // significato del -10.0 usato nelle configurazioni.
  if (mp_->virtual_ceil_height_ > mp_->map_origin_(2)) {
    const int ceil_id = static_cast<int>(
        floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_));
    if (ceil_id >= 0 && ceil_id < mp_->map_voxel_num_(2)) {
      for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
        for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y) {
          const Eigen::Vector3i pt(x, y, ceil_id);
          if (!isInMap(pt)) continue;
          md_->occupancy_buffer_inflate_[toAddress(pt)] = 1;
        }
    }
  }
}

double SDFMap::getResolution() {
  return mp_->resolution_;
}

int SDFMap::getVoxelNum() {
  return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
}

double SDFMap::getExploredVolume() {
  int explored_count = 0;
  int buffer_size = getVoxelNum();
  for (int i = 0; i < buffer_size; ++i) {
    // Count voxels that are known (not UNKNOWN)
    if (md_->occupancy_buffer_[i] >= mp_->clamp_min_log_ - 1e-3) {
      explored_count++;
    }
  }
  double voxel_volume = mp_->resolution_ * mp_->resolution_ * mp_->resolution_;
  return explored_count * voxel_volume;
}

bool SDFMap::getBaseCoor(const int& id, Eigen::Vector4d& transform) {
  if (mp_->no_drone_1_ && id == 1) {
    transform = Eigen::Vector4d(0, 0, 0, 0);
    return true;
  }

  auto iter = md_->swarm_transform_.find(id);
  if (iter == md_->swarm_transform_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("sdf_map"), "No transform of drone %d yet", id);
    return false;
  }
  transform = iter->second;
  return true;
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_->map_origin_, size = mp_->map_size_;
}

void SDFMap::getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) {
  bmin = mp_->box_mind_;
  bmax = mp_->box_maxd_;
}

void SDFMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset) {
  bmin = md_->update_min_;
  bmax = md_->update_max_;
  if (reset) md_->reset_updated_box_ = true;
}

double SDFMap::getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_->resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * mp_->resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_->resolution_inv_;

  return dist;
}
}  // namespace c2_expl
// SDFMap
