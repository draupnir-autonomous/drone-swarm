#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <memory>
#include <random>
#include <string>
#include <vector>

using std::default_random_engine;
using std::normal_distribution;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace c2_expl {
class SDFMap;

class MapROS {
public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap* map);
  void init();

private:
  void cloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
      const nav_msgs::msg::Odometry::ConstSharedPtr& odom);
  void updateESDFCallback();
  void visCallback();

  void publishMapAll();
  void publishMapLocal();
  void publishKnownLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishUnknown();
  void publishFree();
  void publishVirtualGround(const Eigen::Vector3i& min_cut, const Eigen::Vector3i& max_cut,
      const std::vector<char>& ground_mask, int size_y);
  bool loadBoundaryCsv(const std::string& path);
  void buildBoundaryMask();
  bool isInBoundaryIndex(int x, int y) const;
  void publishBoundaryMarker();

  // Export the full occupied-voxel map (no visualization height truncation)
  // to a .pcd file on request, e.g. `ros2 service call /sdf_map/save_map
  // std_srvs/srv/Trigger`.
  void saveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  SDFMap* map_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
      nav_msgs::msg::Odometry>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  rclcpp::Node::SharedPtr node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  SynchronizerCloudPose sync_cloud_odom_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_local_pub_,
      map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_, map_free_pub_,
      map_known_local_pub_, virtual_ground_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr update_range_pub_,
      boundary_marker_pub_;
  rclcpp::TimerBase::SharedPtr esdf_timer_, vis_timer_;
  // Own group so this read-only diagnostic service stays reachable even while the FSM's
  // default-group timers are busy replanning back-to-back on a slow/cluttered map.
  rclcpp::CallbackGroup::SharedPtr save_map_cb_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  string save_pcd_path_;

  // params, msg publication
  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_;
  bool use_local_only_;
  bool virtual_ground_enable_;
  double virtual_ground_z_;
  int virtual_ground_stride_;
  bool boundary_enable_;
  string boundary_csv_;
  double boundary_offset_x_, boundary_offset_y_;
  bool boundary_marker_enable_;
  double boundary_marker_z_, boundary_marker_width_;
  double boundary_marker_r_, boundary_marker_g_, boundary_marker_b_, boundary_marker_a_;
  string frame_id_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d odom_pos_;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;

  std::vector<std::array<double, 2>> boundary_poly_;
  std::vector<char> boundary_mask_;
  int boundary_mask_size_y_;
  bool boundary_ready_;
  double boundary_min_x_, boundary_max_x_, boundary_min_y_, boundary_max_y_;

  rclcpp::Time map_start_time_;

  friend SDFMap;
};
}  // namespace c2_expl

#endif
