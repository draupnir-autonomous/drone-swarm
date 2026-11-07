#include <plan_env/communication_graph.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>

#include <cmath>

namespace {
template <typename T>
T declareIfNew(rclcpp::Node::SharedPtr node, const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) node->declare_parameter(name, default_value);
  return node->get_parameter(name).get_value<T>();
}

std_msgs::msg::ColorRGBA makeConnectedColor() {
  std_msgs::msg::ColorRGBA c;
  c.r = 0.0;
  c.g = 1.0;
  c.b = 0.0;
  c.a = 1.0;
  return c;
}
}  // namespace

namespace c2_expl {

CommunicationGraph::CommunicationGraph(
    const EDTEnvironment::Ptr& edt_env, rclcpp::Node::SharedPtr node)
    : node_(node) {
  this->edt_env_ = edt_env;
  sdf_map_ = edt_env_->sdf_map_;

  cp_.reset(new CommunicationParams);
  cd_.reset(new CommunicationData);

  // Load parameters
  cp_->connection_threshold_ = declareIfNew(node_, "communication.connection_threshold", 500.0);
  cp_->los_enabled_ = declareIfNew(node_, "communication.line_of_sight", true);
  cp_->los_unknown_blocks_ = declareIfNew(node_, "communication.los_unknown_blocks", false);
  cp_->heartbeat_interval_ = declareIfNew(node_, "communication.heartbeat_interval", -1.0);
  cp_->neighbor_timeout_ = declareIfNew(node_, "communication.neighbor_timeout", -1.0);
  drone_id_ = declareIfNew(node_, "exploration.drone_id", 1);

  // Initialize communication data
  cd_->ego_id_ = drone_id_;
  cd_->ego_position_.setZero();
  cd_->have_odom_ = false;
  cd_->linktype_str_ = { "NO_CONNECTION", "CONNECTED" };

  // Relative topic names: every exploration node runs inside its own /drone_N
  // namespace, so "odom_world" resolves per-drone, while the heartbeat
  // send/recv pair is relayed between namespaces by c2x_px4_bridge's
  // swarm_relay node.
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("odom_world", 1,
      std::bind(&CommunicationGraph::odometryCallback, this, std::placeholders::_1));

  heartbeat_pub_ =
      node_->create_publisher<c2x_msgs::msg::Heartbeat>("communication/heartbeat_send", 10);
  heartbeat_sub_ = node_->create_subscription<c2x_msgs::msg::Heartbeat>(
      "communication/heartbeat_recv", 100,
      std::bind(&CommunicationGraph::heartbeatCallback, this, std::placeholders::_1));
  graph_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("communication/graph_vis", 1);

  const double interval = cp_->heartbeat_interval_ > 1e-6 ? cp_->heartbeat_interval_ : 0.05;
  heartbeat_timer_ = node_->create_wall_timer(std::chrono::duration<double>(interval),
      std::bind(&CommunicationGraph::heartbeatTimerCallback, this));
}

CommunicationGraph::~CommunicationGraph() {
}

void CommunicationGraph::updateDynamicGraph(const HeartbeatMsg& hb_msg) {
  LinkType current_link_type = evaluateLinkType(cd_->ego_position_, hb_msg.position_);
  std::lock_guard<std::mutex> lock(neighbors_mutex_);
  auto neighbor_it = neighbors_.find(hb_msg.drone_id_);

  if (neighbor_it == neighbors_.end()) {
    // Add new neighbor if connected
    if (current_link_type != LinkType::NO_CONNECTION) {
      addNewNeighbor(hb_msg.drone_id_, hb_msg.position_, current_link_type);
    }
  } else {
    // Update existing neighbor
    updateExistingNeighbor(
        neighbor_it->second, hb_msg.position_, current_link_type, hb_msg.drone_id_);
  }
}

void CommunicationGraph::addNewNeighbor(
    int drone_id, const Eigen::Vector3d& position, LinkType link_type) {
  RCLCPP_INFO(
      node_->get_logger(), "[CommGraph] Drone %d detected new neighbor %d", drone_id_, drone_id);

  NeighborInfo new_neighbor;
  new_neighbor.neighbor_id_ = drone_id;
  new_neighbor.neighbor_position_ = position;
  new_neighbor.last_update_time_ = node_->now();
  new_neighbor.connection_type_ = link_type;

  neighbors_[drone_id] = new_neighbor;
}

void CommunicationGraph::updateExistingNeighbor(
    NeighborInfo& neighbor, const Eigen::Vector3d& position, LinkType new_link_type, int drone_id) {
  LinkType previous_link_type = neighbor.connection_type_;

  // Update only if not consecutively NO_CONNECTION
  if (new_link_type != LinkType::NO_CONNECTION || previous_link_type != LinkType::NO_CONNECTION) {
    neighbor.last_update_time_ = node_->now();
  }

  neighbor.neighbor_position_ = position;
  neighbor.connection_type_ = new_link_type;

  // Log link type changes
  if (previous_link_type != new_link_type) {
    RCLCPP_INFO(node_->get_logger(), "[CommGraph] Drone %d neighbor %d link type changed: %s --> %s",
        drone_id_, drone_id, cd_->linktype_str_[static_cast<int>(previous_link_type)].c_str(),
        cd_->linktype_str_[static_cast<int>(new_link_type)].c_str());
  }
}

// Il segmento fra due droni e' libero da ostacoli NOTI?
//
// E' il cuore del modello ottico: si campiona la retta fra le due posizioni al
// passo della mappa e si guarda se attraversa voxel occupati.
//
// ASIMMETRIA, da conoscere: ogni drone valuta questo sulla PROPRIA mappa, che e'
// parziale e diversa da quella degli altri. A puo' concludere di non vedere B
// mentre B crede di vedere A. Nella realta' il collegamento ottico o c'e' o non
// c'e', simmetrico; qui e' la STIMA a essere asimmetrica. Non lo si corregge
// forzando la simmetria (servirebbe un accordo, e l'accordo richiede il
// collegamento che si sta valutando): lo si accetta e lo si tiene presente
// leggendo i log, esattamente come accade a un sistema reale che deve decidere
// con informazione locale.
bool CommunicationGraph::hasLineOfSight(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2) {
  if (!sdf_map_) return true;
  const Eigen::Vector3d delta = pos2 - pos1;
  const double dist = delta.norm();
  if (dist < 1e-3) return true;

  // Passo pari alla risoluzione della mappa: piu' fitto sarebbe sprecato
  // (ricadrebbe nello stesso voxel), piu' rado lascerebbe passare il raggio
  // attraverso un muro sottile.
  const double step = std::max(0.05, sdf_map_->getResolution());
  const int samples = static_cast<int>(dist / step) + 1;
  const Eigen::Vector3d dir = delta / dist;

  for (int i = 1; i < samples; ++i) {
    const Eigen::Vector3d p = pos1 + dir * (i * step);
    const int occ = sdf_map_->getOccupancy(p);
    if (occ == SDFMap::OCCUPIED) return false;
    if (cp_->los_unknown_blocks_ && occ == SDFMap::UNKNOWN) return false;
  }
  return true;
}

LinkType CommunicationGraph::evaluateLinkType(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2) {
  double dist = calculateDistance(pos1, pos2);
  if (classifyLinkType(dist) == LinkType::NO_CONNECTION) return LinkType::NO_CONNECTION;
  if (cp_->los_enabled_ && !hasLineOfSight(pos1, pos2)) return LinkType::NO_CONNECTION;
  return LinkType::CONNECTED;
}

LinkType CommunicationGraph::classifyLinkType(double distance) {
  if (distance <= cp_->connection_threshold_) {
    return LinkType::CONNECTED;
  }
  return LinkType::NO_CONNECTION;
}

// Caller must hold neighbors_mutex_.
void CommunicationGraph::cleanupStaleNeighbors() {
  auto curr_time = node_->now();

  // delete neighbors that have timed out
  for (auto it = neighbors_.begin(); it != neighbors_.end();) {
    if ((curr_time - it->second.last_update_time_).seconds() > cp_->neighbor_timeout_) {
      RCLCPP_WARN(node_->get_logger(),
          "[CommGraph] neighbor %d timed out and has been removed from the list.", it->first);
      it = neighbors_.erase(it);
    } else {
      ++it;
    }
  }
}

// Caller must hold neighbors_mutex_.
void CommunicationGraph::updateAllLinks() {
  // Reclassify all existing links based on current positions
  for (auto& neighbor_pair : neighbors_) {
    NeighborInfo& neighbor = neighbor_pair.second;
    neighbor.connection_type_ = evaluateLinkType(cd_->ego_position_, neighbor.neighbor_position_);
    neighbor.last_update_time_ = node_->now();
  }
}

void CommunicationGraph::getNeighborIds(vector<int>& neighbor_ids) {
  neighbor_ids.clear();
  {
    std::lock_guard<std::mutex> lock(neighbors_mutex_);
    for (const auto& neighbor_pair : neighbors_) {
      if (neighbor_pair.second.connection_type_ != LinkType::NO_CONNECTION) {
        neighbor_ids.push_back(neighbor_pair.first);
      }
    }
  }
  neighbor_ids.push_back(cd_->ego_id_);  // Always include self
}

bool CommunicationGraph::isConnected(int drone_id) {
  if (drone_id == cd_->ego_id_) return true;
  std::lock_guard<std::mutex> lock(neighbors_mutex_);
  auto it = neighbors_.find(drone_id);
  if (it == neighbors_.end()) return false;
  return it->second.connection_type_ != LinkType::NO_CONNECTION;
}

void CommunicationGraph::visualizeCommunicationGraph() {
  if (!cd_->have_odom_) return;

  static const std_msgs::msg::ColorRGBA connected_color = makeConnectedColor();

  visualization_msgs::msg::Marker link_marker;
  link_marker.header.frame_id = "world";
  link_marker.header.stamp = node_->now();
  link_marker.ns = "communication_links";
  link_marker.id = drone_id_;
  link_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  link_marker.action = visualization_msgs::msg::Marker::ADD;
  link_marker.scale.x = 0.08;
  link_marker.pose.orientation.w = 1.0;  // Identity quaternion

  {
    std::lock_guard<std::mutex> lock(neighbors_mutex_);
    for (const auto& neighbor_pair : neighbors_) {
      const NeighborInfo& neighbor = neighbor_pair.second;
      if (neighbor.connection_type_ == LinkType::NO_CONNECTION) continue;

      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = cd_->ego_position_(0);
      start_point.y = cd_->ego_position_(1);
      start_point.z = cd_->ego_position_(2);
      end_point.x = neighbor.neighbor_position_(0);
      end_point.y = neighbor.neighbor_position_(1);
      end_point.z = neighbor.neighbor_position_(2);

      link_marker.points.push_back(start_point);
      link_marker.points.push_back(end_point);

      link_marker.colors.push_back(connected_color);
      link_marker.colors.push_back(connected_color);
    }
  }

  graph_pub_->publish(link_marker);
}

void CommunicationGraph::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  cd_->ego_position_(0) = msg->pose.pose.position.x;
  cd_->ego_position_(1) = msg->pose.pose.position.y;
  cd_->ego_position_(2) = msg->pose.pose.position.z;

  if (!cd_->have_odom_) {
    cd_->have_odom_ = true;
    cd_->last_odom_time_ = node_->now();
  }
}

// periodic heartbeat to broadcast own state
void CommunicationGraph::heartbeatTimerCallback() {
  {
    std::lock_guard<std::mutex> lock(neighbors_mutex_);
    // periodically clean up stale neighbors
    this->cleanupStaleNeighbors();
    // update all links
    this->updateAllLinks();
  }

  // Don't advertise a position we don't have yet: before the first odometry
  // sample ego_position_ is the origin, and a peer that believed it would
  // classify the link off a bogus distance.
  if (!cd_->have_odom_) return;

  c2x_msgs::msg::Heartbeat hb_msg;
  hb_msg.drone_id = cd_->ego_id_;
  hb_msg.pos[0] = cd_->ego_position_(0);
  hb_msg.pos[1] = cd_->ego_position_(1);
  hb_msg.pos[2] = cd_->ego_position_(2);
  hb_msg.timestamp = node_->now().seconds();

  // publish heartbeat
  heartbeat_pub_->publish(hb_msg);

  this->visualizeCommunicationGraph();
}

// heartbeat messages from other drones
void CommunicationGraph::heartbeatCallback(const c2x_msgs::msg::Heartbeat::SharedPtr msg) {
  // ignore self heartbeat
  if (msg->drone_id == cd_->ego_id_) {
    return;
  }
  // Without our own position the link distance is meaningless.
  if (!cd_->have_odom_) return;

  HeartbeatMsg hb_msg;
  hb_msg.drone_id_ = msg->drone_id;
  hb_msg.position_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  hb_msg.heartbeat_timestamp_ =
      rclcpp::Time(static_cast<int64_t>(msg->timestamp * 1e9), RCL_ROS_TIME);

  // process the received heartbeat
  this->updateDynamicGraph(hb_msg);
}

}  // namespace c2_expl
