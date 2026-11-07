/**
 * Standalone smoke-test node for the ported plan_env package: owns a single
 * SDFMap fed by live sensor data (point cloud + odometry), for Phase 2
 * verification against Gazebo before plan_env is wired into the full
 * exploration_manager FSM (Phase 5).
 */
#include <plan_env/sdf_map.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("plan_env_test_node");

  auto sdf_map = std::make_shared<c2_expl::SDFMap>();
  sdf_map->initMap(node);

  RCLCPP_INFO(node->get_logger(), "plan_env_test_node: SDFMap initialized, spinning.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
