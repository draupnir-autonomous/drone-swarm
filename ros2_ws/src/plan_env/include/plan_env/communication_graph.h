#ifndef COMMUNICATION_GRAPH_H
#define COMMUNICATION_GRAPH_H

// Neighbour discovery under the paper's limited-communication model: each UAV
// broadcasts a periodic heartbeat carrying its own position, and classifies
// every peer it hears from as CONNECTED / NO_CONNECTION by comparing the
// distance against communication.connection_threshold (r_comm in the paper).
// getNeighborIds() is what gates task allocation - only drones currently
// within r_comm take part in a MeetingOpt round.

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <map>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <c2x_msgs/msg/heartbeat.hpp>
#include <plan_env/comm_data.h>
#include <plan_env/edt_environment.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::vector;

namespace c2_expl {
class EDTEnvironment;
class SDFMap;
struct CommunicationParams;

class CommunicationGraph {
public:
  CommunicationGraph(const EDTEnvironment::Ptr& edt_env, rclcpp::Node::SharedPtr node);
  ~CommunicationGraph();

  void getNeighborIds(vector<int>& neighbor_ids);
  // True when the peer is currently inside r_comm. MultiMapManager uses this to
  // decide whether a map chunk may be shipped to that drone at all.
  bool isConnected(int drone_id);

private:
  void updateDynamicGraph(const HeartbeatMsg& hb_msg);
  void updateAllLinks();
  LinkType classifyLinkType(double distance);
  // Segmento libero da ostacoli noti fra due posizioni: e' il criterio
  // ottico che affianca la distanza. Vedi la nota nel .cpp sull'asimmetria.
  bool hasLineOfSight(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
  void cleanupStaleNeighbors();

  void addNewNeighbor(int drone_id, const Eigen::Vector3d& position, LinkType link_type);
  void updateExistingNeighbor(NeighborInfo& neighbor, const Eigen::Vector3d& position,
      LinkType new_link_type, int drone_id);
  LinkType evaluateLinkType(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void visualizeCommunicationGraph();

  // swarm
  void heartbeatTimerCallback();
  void heartbeatCallback(const c2x_msgs::msg::Heartbeat::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  int drone_id_;
  std::map<int, NeighborInfo> neighbors_;  // key: drone_id

  shared_ptr<SDFMap> sdf_map_;
  shared_ptr<EDTEnvironment> edt_env_;
  shared_ptr<CommunicationData> cd_;
  shared_ptr<CommunicationParams> cp_;

  // neighbors_ is written from the heartbeat timer/subscription and read by
  // getNeighborIds()/isConnected() from the FSM's opt timer and from
  // MultiMapManager - all of which run in different callback groups under the
  // MultiThreadedExecutor, so the map could otherwise be rehashed mid-iteration.
  std::mutex neighbors_mutex_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<c2x_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<c2x_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  double calculateDistance(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2) {
    return (pos1 - pos2).norm();
  }
};

}  // namespace c2_expl
#endif
