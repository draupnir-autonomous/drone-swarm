/**
 * Inter-drone message relay: the ROS2 stand-in for ROS1's swarm_bridge
 * (bridge_node_udp), which broadcast each drone's "<topic>_send" over UDP and
 * republished whatever arrived on "<topic>_recv".
 *
 * Here every exploration stack lives in its own /drone_N namespace, so this
 * single node subscribes to /drone_i/<topic>_send for every drone and forwards
 * each message to /drone_j/<topic>_recv for all j != i. Handing the delivery to
 * one relay - rather than letting the drones subscribe to each other directly -
 * keeps the "who can hear whom" decision in one place, and mirrors the
 * send/recv topic pair the exploration nodes were already written against.
 *
 * Range gating is deliberately NOT done here. CommunicationGraph is what
 * enforces r_comm: it classifies a peer as CONNECTED only while the distance is
 * within communication.connection_threshold, and everything downstream
 * (MeetingOpt participants, map-chunk transmission) is gated on that. Dropping
 * messages here as well would additionally cut the heartbeats themselves, and a
 * drone that stops hearing heartbeats can never discover that a peer came back
 * into range.
 */
#include <rclcpp/rclcpp.hpp>

#include <c2x_msgs/msg/bspline.hpp>
#include <c2x_msgs/msg/chunk_data.hpp>
#include <c2x_msgs/msg/chunk_stamps.hpp>
#include <c2x_msgs/msg/drone_state.hpp>
#include <c2x_msgs/msg/heartbeat.hpp>
#include <c2x_msgs/msg/meeting_opt.hpp>
#include <c2x_msgs/msg/meeting_opt_response.hpp>

#include <memory>
#include <string>
#include <vector>

namespace {
// Destinatario di un messaggio: 0 = tutti gli altri droni. I pezzi di mappa sono indirizzati
// (to_drone_id): mandarli a tutti raddoppiava il traffico con tre droni, e il drone che non
// era il destinatario li riceveva solo per scartarli (sciame, ottobre 2026).
template <typename MsgT>
int destinatario(const MsgT&) {
  return 0;
}
int destinatario(const c2x_msgs::msg::ChunkData& msg) {
  return msg.to_drone_id;
}
}  // namespace

class SwarmRelay : public rclcpp::Node {
public:
  SwarmRelay() : Node("c2x_swarm_relay") {
    drone_num_ = this->declare_parameter<int>("drone_num", 1);
    ns_prefix_ = this->declare_parameter<std::string>("ns_prefix", "/drone_");
    if (drone_num_ < 1) drone_num_ = 1;

    // Sized to carry a burst of map chunks without dropping any: MultiMapManager
    // answers a single ChunkStamps advert with every chunk the peer is missing,
    // which right after a link comes back can be hundreds of messages at once.
    relay<c2x_msgs::msg::DroneState>("swarm_expl/drone_state", 20);
    relay<c2x_msgs::msg::MeetingOpt>("swarm_expl/meeting_opt", 200);
    relay<c2x_msgs::msg::MeetingOptResponse>("swarm_expl/meeting_opt_res", 200);
    relay<c2x_msgs::msg::Bspline>("planning/swarm_traj", 200);
    relay<c2x_msgs::msg::Heartbeat>("communication/heartbeat", 100);
    relay<c2x_msgs::msg::ChunkStamps>("multi_map_manager/chunk_stamps", 20);
    relay<c2x_msgs::msg::ChunkData>("multi_map_manager/chunk_data", 5000);

    RCLCPP_INFO(this->get_logger(), "Swarm relay up for %d drones (namespace prefix '%s')",
        drone_num_, ns_prefix_.c_str());
  }

private:
  template <typename MsgT>
  void relay(const std::string& base_topic, const size_t depth) {
    auto pubs = std::make_shared<std::vector<typename rclcpp::Publisher<MsgT>::SharedPtr>>();
    pubs->reserve(drone_num_);
    for (int j = 1; j <= drone_num_; ++j) {
      pubs->push_back(this->create_publisher<MsgT>(
          ns_prefix_ + std::to_string(j) + "/" + base_topic + "_recv", depth));
    }

    for (int i = 1; i <= drone_num_; ++i) {
      const int from_idx = i - 1;
      auto sub = this->create_subscription<MsgT>(
          ns_prefix_ + std::to_string(i) + "/" + base_topic + "_send", depth,
          [pubs, from_idx](const typename MsgT::SharedPtr msg) {
            const int dest = destinatario(*msg);
            for (size_t j = 0; j < pubs->size(); ++j) {
              if (dest > 0 && static_cast<int>(j) != dest - 1) continue;
              // Never echo a message back to its sender: the exploration nodes
              // do filter self-addressed messages, but bouncing every heartbeat
              // and chunk straight back is pure waste.
              if (static_cast<int>(j) == from_idx) continue;
              (*pubs)[j]->publish(*msg);
            }
          });
      subs_.push_back(sub);
    }
  }

  int drone_num_;
  std::string ns_prefix_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwarmRelay>());
  rclcpp::shutdown();
  return 0;
}
