#include <chrono>
#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>
#include <plan_env/multi_map_manager.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <geometry_msgs/msg/point.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <visualization_msgs/msg/marker.hpp>

#include <fstream>

namespace c2_expl {
namespace {
bool pointInPolygon(const std::vector<std::array<double, 2>>& poly, double x, double y) {
  if (poly.size() < 3) return false;
  bool inside = false;
  size_t j = poly.size() - 1;
  for (size_t i = 0; i < poly.size(); j = i++) {
    const double xi = poly[i][0];
    const double yi = poly[i][1];
    const double xj = poly[j][0];
    const double yj = poly[j][1];
    const bool intersect = ((yi > y) != (yj > y)) &&
                           (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}
}  // namespace

MapROS::MapROS() {
}

MapROS::~MapROS() {
}

void MapROS::setMap(SDFMap* map) {
  this->map_ = map;
}

void MapROS::init() {
  node_->declare_parameter("map_ros.esdf_slice_height", -0.1);
  node_->declare_parameter("map_ros.visualization_truncate_height", -0.1);
  node_->declare_parameter("map_ros.visualization_truncate_low", -0.1);
  node_->declare_parameter("map_ros.show_occ_time", false);
  node_->declare_parameter("map_ros.show_esdf_time", false);
  node_->declare_parameter("map_ros.show_all_map", false);
  node_->declare_parameter("map_ros.use_local_only", true);
  node_->declare_parameter("map_ros.frame_id", string("world"));
  node_->declare_parameter("map_ros.virtual_ground_enable", false);
  node_->declare_parameter("map_ros.virtual_ground_z", 0.0);
  node_->declare_parameter("map_ros.virtual_ground_stride", 1);
  node_->declare_parameter("map_ros.boundary_enable", false);
  node_->declare_parameter("map_ros.boundary_csv", string(""));
  node_->declare_parameter("map_ros.boundary_offset_x", 0.0);
  node_->declare_parameter("map_ros.boundary_offset_y", 0.0);
  node_->declare_parameter("map_ros.boundary_marker_enable", true);
  node_->declare_parameter("map_ros.boundary_marker_z", 0.0);
  node_->declare_parameter("map_ros.boundary_marker_width", 0.05);
  node_->declare_parameter("map_ros.boundary_marker_r", 0.8);
  node_->declare_parameter("map_ros.boundary_marker_g", 0.3);
  node_->declare_parameter("map_ros.boundary_marker_b", 0.1);
  node_->declare_parameter("map_ros.boundary_marker_a", 0.9);
  node_->declare_parameter("map_ros.save_pcd_path", string("/tmp/c2x_explored_map.pcd"));

  node_->get_parameter("map_ros.esdf_slice_height", esdf_slice_height_);
  node_->get_parameter("map_ros.visualization_truncate_height", visualization_truncate_height_);
  node_->get_parameter("map_ros.visualization_truncate_low", visualization_truncate_low_);
  node_->get_parameter("map_ros.show_occ_time", show_occ_time_);
  node_->get_parameter("map_ros.show_esdf_time", show_esdf_time_);
  node_->get_parameter("map_ros.show_all_map", show_all_map_);
  node_->get_parameter("map_ros.use_local_only", use_local_only_);
  node_->get_parameter("map_ros.frame_id", frame_id_);
  node_->get_parameter("map_ros.virtual_ground_enable", virtual_ground_enable_);
  node_->get_parameter("map_ros.virtual_ground_z", virtual_ground_z_);
  node_->get_parameter("map_ros.virtual_ground_stride", virtual_ground_stride_);
  if (virtual_ground_stride_ < 1) virtual_ground_stride_ = 1;
  node_->get_parameter("map_ros.boundary_enable", boundary_enable_);
  node_->get_parameter("map_ros.boundary_csv", boundary_csv_);
  node_->get_parameter("map_ros.boundary_offset_x", boundary_offset_x_);
  node_->get_parameter("map_ros.boundary_offset_y", boundary_offset_y_);
  node_->get_parameter("map_ros.boundary_marker_enable", boundary_marker_enable_);
  node_->get_parameter("map_ros.boundary_marker_z", boundary_marker_z_);
  node_->get_parameter("map_ros.boundary_marker_width", boundary_marker_width_);
  node_->get_parameter("map_ros.boundary_marker_r", boundary_marker_r_);
  node_->get_parameter("map_ros.boundary_marker_g", boundary_marker_g_);
  node_->get_parameter("map_ros.boundary_marker_b", boundary_marker_b_);
  node_->get_parameter("map_ros.boundary_marker_a", boundary_marker_a_);
  node_->get_parameter("map_ros.save_pcd_path", save_pcd_path_);

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;

  // Periodo dell'ESDF (map_ros.esdf_period, di serie 0.05 s = a ogni nuvola). Sul
  // Jetson l'ESDF a ogni nuvola costava 0.06 s x 6 Hz nel thread della
  // pianificazione (cubicle_office, 17 ottobre 2026); lo legge solo l'ottimizzatore,
  // e ogni piano si controlla comunque sulla mappa gonfiata, sempre aggiornata.
  const double esdf_period =
      node_->has_parameter("map_ros.esdf_period") ?
          node_->get_parameter("map_ros.esdf_period").as_double() :
          node_->declare_parameter("map_ros.esdf_period", 0.05);
  esdf_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(esdf_period), [this]() { updateESDFCallback(); });
  vis_timer_ =
      node_->create_wall_timer(std::chrono::duration<double>(0.5), [this]() { visCallback(); });

  map_all_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/occupancy_all", 10);
  map_local_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/occupancy_local", 10);
  map_known_local_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/known_local", 10);
  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/unknown", 10);
  map_free_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/free", 10);
  virtual_ground_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/virtual_ground", 10);
  boundary_marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("sdf_map/boundary", 1);
  esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_map/esdf", 10);
  update_range_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("sdf_map/update_range", 10);

  save_map_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  save_map_srv_ = node_->create_service<std_srvs::srv::Trigger>("sdf_map/save_map",
      std::bind(&MapROS::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, save_map_cb_group_);

  boundary_ready_ = false;
  if (boundary_enable_) {
    if (!boundary_csv_.empty() && loadBoundaryCsv(boundary_csv_)) {
      buildBoundaryMask();
    } else {
      RCLCPP_WARN(node_->get_logger(), "[MapROS] boundary enabled but CSV is missing or invalid: %s",
          boundary_csv_.c_str());
      boundary_enable_ = false;
    }
  }

  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(node_, "map_ros/cloud"));
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(node_, "odom_world"));

  sync_cloud_odom_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
      MapROS::SyncPolicyCloudPose(1000), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(
      std::bind(&MapROS::cloudOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

  map_start_time_ = node_->now();
}

bool MapROS::loadBoundaryCsv(const std::string& path) {
  boundary_poly_.clear();
  boundary_min_x_ = std::numeric_limits<double>::infinity();
  boundary_max_x_ = -std::numeric_limits<double>::infinity();
  boundary_min_y_ = std::numeric_limits<double>::infinity();
  boundary_max_y_ = -std::numeric_limits<double>::infinity();

  std::ifstream fin(path);
  if (!fin.is_open()) {
    RCLCPP_WARN(node_->get_logger(), "[MapROS] Failed to open boundary CSV: %s", path.c_str());
    return false;
  }

  std::string line;
  bool first_line = true;
  while (std::getline(fin, line)) {
    if (line.empty()) continue;
    if (first_line) {
      first_line = false;
      if (line.find("x") != std::string::npos && line.find(",") != std::string::npos) {
        continue;
      }
    }
    size_t comma = line.find(',');
    if (comma == std::string::npos) continue;
    try {
      double x = std::stod(line.substr(0, comma));
      double y = std::stod(line.substr(comma + 1));
      x += boundary_offset_x_;
      y += boundary_offset_y_;
      boundary_poly_.push_back({ x, y });
      boundary_min_x_ = std::min(boundary_min_x_, x);
      boundary_max_x_ = std::max(boundary_max_x_, x);
      boundary_min_y_ = std::min(boundary_min_y_, y);
      boundary_max_y_ = std::max(boundary_max_y_, y);
    } catch (const std::exception&) {
      continue;
    }
  }

  if (boundary_poly_.size() < 3) {
    RCLCPP_WARN(node_->get_logger(), "[MapROS] Boundary CSV has too few points: %s", path.c_str());
    boundary_poly_.clear();
    return false;
  }

  return true;
}

void MapROS::buildBoundaryMask() {
  boundary_mask_.clear();
  boundary_ready_ = false;
  if (!boundary_enable_ || boundary_poly_.size() < 3) return;

  const int nx = map_->mp_->map_voxel_num_(0);
  const int ny = map_->mp_->map_voxel_num_(1);
  if (nx <= 0 || ny <= 0) return;

  boundary_mask_size_y_ = ny;
  boundary_mask_.assign(nx * ny, 0);

  Eigen::Vector3d pos;
  for (int x = 0; x < nx; ++x) {
    for (int y = 0; y < ny; ++y) {
      map_->indexToPos(Eigen::Vector3i(x, y, 0), pos);
      const double px = pos(0);
      const double py = pos(1);
      if (px < boundary_min_x_ || px > boundary_max_x_ || py < boundary_min_y_ ||
          py > boundary_max_y_) {
        continue;
      }
      if (pointInPolygon(boundary_poly_, px, py)) {
        boundary_mask_[x * ny + y] = 1;
      }
    }
  }
  boundary_ready_ = true;
}

bool MapROS::isInBoundaryIndex(int x, int y) const {
  if (!boundary_ready_) return true;
  if (x < 0 || y < 0) return false;
  const int nx = map_->mp_->map_voxel_num_(0);
  const int ny = boundary_mask_size_y_;
  if (x >= nx || y >= ny) return false;
  return boundary_mask_[x * ny + y] != 0;
}

void MapROS::publishBoundaryMarker() {
  if (!boundary_marker_enable_ || !boundary_ready_ || !boundary_marker_pub_) return;
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = node_->now();
  mk.ns = "boundary";
  mk.id = 0;
  mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = boundary_marker_width_;
  mk.color.r = boundary_marker_r_;
  mk.color.g = boundary_marker_g_;
  mk.color.b = boundary_marker_b_;
  mk.color.a = boundary_marker_a_;

  mk.points.clear();
  geometry_msgs::msg::Point p;
  for (const auto& pt : boundary_poly_) {
    p.x = pt[0];
    p.y = pt[1];
    p.z = boundary_marker_z_;
    mk.points.push_back(p);
  }
  if (!boundary_poly_.empty()) {
    p.x = boundary_poly_.front()[0];
    p.y = boundary_poly_.front()[1];
    p.z = boundary_marker_z_;
    mk.points.push_back(p);
  }

  boundary_marker_pub_->publish(mk);
}

void MapROS::visCallback() {
  static int cnt = 0;
  cnt++;

  // Le nuvole di visualizzazione (mappa intera, locale, pavimento virtuale) si
  // costruiscono scorrendo tutta la parte esplorata della mappa, due volte, ogni
  // 0.5 s e nello stesso gruppo di callback della pianificazione: 0.035-0.058 s a
  // chiamata sul Jetson (cubicle_office), per topic che legge solo RViz. Senza
  // iscritti non si costruiscono.
  if (map_all_pub_->get_subscription_count() > 0) publishMapAll();
  if (map_local_pub_->get_subscription_count() > 0 ||
      (map_local_inflate_pub_ && map_local_inflate_pub_->get_subscription_count() > 0) ||
      virtual_ground_pub_->get_subscription_count() > 0)
    publishMapLocal();
  publishBoundaryMarker();
}

void MapROS::updateESDFCallback() {
  if (!esdf_need_update_) return;
  auto t1 = node_->now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = node_->now();
  esdf_time_ += (t2 - t1).seconds();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).seconds());
  esdf_num_++;
  if (show_esdf_time_)
    RCLCPP_WARN(node_->get_logger(), "ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).seconds(),
        esdf_time_ / esdf_num_, max_esdf_time_);
}

void MapROS::cloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
  odom_pos_(0) = odom->pose.pose.position.x;
  odom_pos_(1) = odom->pose.pose.position.y;
  odom_pos_(2) = odom->pose.pose.position.z;
  const auto t1 = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();

  map_->inputPointCloud(cloud, num, odom_pos_);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
  // Tempo di inserimento della nuvola (raycasting + gonfiaggio locale): gira nello
  // stesso gruppo di callback della pianificazione e dell'ESDF, quindi la ritarda.
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
  fuse_time_ += dt;
  max_fuse_time_ = max(max_fuse_time_, dt);
  if (++fuse_num_ % 50 == 0 && show_occ_time_)
    RCLCPP_WARN(node_->get_logger(), "Fusion t: cur: %lf, avg: %lf, max: %lf, punti %d, n %d", dt,
        fuse_time_ / fuse_num_, max_fuse_time_, num, fuse_num_);
}

void MapROS::publishMapAll() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  Eigen::Vector3i min_idx, max_idx;
  map_->posToIndex(map_->md_->all_min_, min_idx);
  map_->posToIndex(map_->md_->all_max_, max_idx);

  map_->boundIndex(min_idx);
  map_->boundIndex(max_idx);

  for (int x = min_idx[0]; x <= max_idx[0]; ++x)
    for (int y = min_idx[1]; y <= max_idx[1]; ++y) {
      if (!isInBoundaryIndex(x, y)) continue;
      for (int z = min_idx[2]; z <= max_idx[2]; ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
            map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
    }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_all_pub_->publish(cloud_msg);
}

void MapROS::saveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  // Same occupied-voxel iteration as publishMapAll(), but without the
  // visualization height truncation - this is meant to be a complete export,
  // not a view-friendly slice.
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_idx, max_idx;
  map_->posToIndex(map_->md_->all_min_, min_idx);
  map_->posToIndex(map_->md_->all_max_, max_idx);
  map_->boundIndex(min_idx);
  map_->boundIndex(max_idx);

  for (int x = min_idx[0]; x <= max_idx[0]; ++x)
    for (int y = min_idx[1]; y <= max_idx[1]; ++y) {
      if (!isInBoundaryIndex(x, y)) continue;
      for (int z = min_idx[2]; z <= max_idx[2]; ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
    }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  if (cloud.points.empty()) {
    res->success = false;
    res->message = "Map is empty, nothing to save.";
    return;
  }

  if (pcl::io::savePCDFileBinary(save_pcd_path_, cloud) != 0) {
    res->success = false;
    res->message = "Failed to write " + save_pcd_path_;
    RCLCPP_ERROR(node_->get_logger(), "[MapROS] Failed to save map to %s", save_pcd_path_.c_str());
    return;
  }

  res->success = true;
  res->message = save_pcd_path_ + " (" + std::to_string(cloud.points.size()) + " points)";
  RCLCPP_INFO(node_->get_logger(), "[MapROS] Saved %zu occupied points to %s", cloud.points.size(),
      save_pcd_path_.c_str());
}

void MapROS::publishMapLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  const auto& occ_buffer =
      use_local_only_ ? map_->md_->occupancy_buffer_local_only_ : map_->md_->occupancy_buffer_;
  Eigen::Vector3i min_cut, max_cut;
  if (use_local_only_) {
    map_->posToIndex(map_->md_->all_min_, min_cut);
    map_->posToIndex(map_->md_->all_max_, max_cut);
  } else {
    min_cut = map_->md_->local_bound_min_;
    max_cut = map_->md_->local_bound_max_;
  }
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  const int size_x = max_cut(0) - min_cut(0) + 1;
  const int size_y = max_cut(1) - min_cut(1) + 1;
  const bool need_ground = virtual_ground_enable_ && size_x > 0 && size_y > 0;
  std::vector<char> ground_mask;
  if (need_ground) {
    ground_mask.assign(size_x * size_y, 0);
  }

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      if (!isInBoundaryIndex(x, y)) continue;
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        double occ = occ_buffer[map_->toAddress(x, y, z)];
        if (need_ground && occ >= map_->mp_->clamp_min_log_ - 1e-3) {
          int gx = x - min_cut(0);
          int gy = y - min_cut(1);
          ground_mask[gx * size_y + gy] = 1;
        }
        // Only occupied cells
        if (occ > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  if (map_local_pub_) {
    map_local_pub_->publish(cloud_msg);
  }
  pcl::toROSMsg(cloud2, cloud_msg);
  if (map_local_inflate_pub_) {
    map_local_inflate_pub_->publish(cloud_msg);
  }

  if (need_ground) {
    publishVirtualGround(min_cut, max_cut, ground_mask, size_y);
  }
}

void MapROS::publishVirtualGround(
    const Eigen::Vector3i& min_cut, const Eigen::Vector3i& max_cut,
    const std::vector<char>& ground_mask, int size_y) {
  if (!virtual_ground_enable_ || !virtual_ground_pub_ || ground_mask.empty()) return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3d pos;
  const int stride = std::max(1, virtual_ground_stride_);

  for (int x = min_cut(0); x <= max_cut(0); x += stride) {
    int gx = x - min_cut(0);
    for (int y = min_cut(1); y <= max_cut(1); y += stride) {
      int gy = y - min_cut(1);
      if (!ground_mask[gx * size_y + gy]) continue;
      map_->indexToPos(Eigen::Vector3i(x, y, 0), pos);
      pcl::PointXYZ pt;
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = virtual_ground_z_;
      cloud.push_back(pt);
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  virtual_ground_pub_->publish(cloud_msg);
}

void MapROS::publishKnownLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      if (!isInBoundaryIndex(x, y)) continue;
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        double occ = map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)];
        if (occ >= map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  if (map_known_local_pub_) {
    map_known_local_pub_->publish(cloud_msg);
  }
}

void MapROS::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_idx, max_idx;
  map_->posToIndex(map_->md_->all_min_, min_idx);
  map_->posToIndex(map_->md_->all_max_, max_idx);

  map_->boundIndex(min_idx);
  map_->boundIndex(max_idx);

  for (int x = min_idx(0); x <= max_idx(0); ++x)
    for (int y = min_idx(1); y <= max_idx(1); ++y) {
      if (!isInBoundaryIndex(x, y)) continue;
      for (int z = min_idx(2); z <= max_idx(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] <
            map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
    }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_->publish(cloud_msg);
}

void MapROS::publishFree() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_idx, max_idx;
  map_->posToIndex(map_->md_->all_min_, min_idx);
  map_->posToIndex(map_->md_->all_max_, max_idx);

  map_->boundIndex(min_idx);
  map_->boundIndex(max_idx);

  for (int x = min_idx(0); x <= max_idx(0); ++x)
    for (int y = min_idx(1); y <= max_idx(1); ++y) {
      if (!isInBoundaryIndex(x, y)) continue;
      for (int z = min_idx(2); z <= max_idx(2); ++z) {
        double occ = map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)];
        if (occ >= map_->mp_->clamp_min_log_ - 1e-3 && occ <= map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
    }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  map_free_pub_->publish(cloud_msg);
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::msg::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = node_->now();
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_->publish(mk);
}

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut =
      map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                        map_->mp_->local_map_margin_, map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut =
      map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                        map_->mp_->local_map_margin_, map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      if (!isInBoundaryIndex(x, y)) continue;
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_->publish(cloud_msg);
}
}  // namespace c2_expl
