#ifndef _MULTI_MAP_MANAGER_H_
#define _MULTI_MAP_MANAGER_H_

// Map-chunk synchronisation between UAVs ("drones exchange local map updates
// within communication range" in the paper). Every batch of newly-updated
// voxels this drone produces is appended to a numbered chunk; drones
// periodically advertise which chunk index ranges they hold (ChunkStamps) and
// unicast the chunks a peer is missing (ChunkData). Received chunks are merged
// straight into the occupancy buffer, so each drone converges on the union of
// everyone's map without ever shipping the whole grid.

#include <rclcpp/rclcpp.hpp>

#include <c2x_msgs/msg/chunk_data.hpp>
#include <c2x_msgs/msg/chunk_stamps.hpp>

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>
#include <unordered_map>

using std::shared_ptr;
using std::unordered_map;
using std::vector;

namespace c2_expl {
class SDFMap;
class MapROS;
class CommunicationGraph;

// A map chunk, the elementary exchange unit between robots
struct MapChunk {
  uint32_t idx_;  // Start from 1
  vector<uint32_t> voxel_adrs_;
  vector<uint8_t> voxel_occ_;

  bool need_query_;
  bool empty_;
  // Pezzo di un compagno: quando e' arrivato qui (s simulati). Vedi sendChunks.
  double t_ins_ = 0.0;
};

struct ChunksData {
  vector<MapChunk> chunks_;
  vector<int> idx_list_;
};

struct ChunksBox {
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  bool valid_;
};

class MultiMapManager {
public:
  MultiMapManager();
  ~MultiMapManager();
  void setMap(SDFMap* map);
  void setCommunicationGraph(shared_ptr<CommunicationGraph> comm_graph);
  void init(rclcpp::Node::SharedPtr node);

  void updateMapChunk(const vector<uint32_t>& adrs);
  void getChunkBoxes(
      vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset = true);

private:
  void flushAdrBuffer(bool allow_partial);
  bool canTransmitChunkTo(const int& other_drone_id) const;
  void sendChunks(const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list);
  void getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs);
  void insertChunkToMap(const MapChunk& chunk, const int& chunk_drone_id);
  void adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx);

  void stampTimerCallback();
  void chunkTimerCallback();
  void stampMsgCallback(const c2x_msgs::msg::ChunkStamps::SharedPtr msg);
  void chunkCallback(const c2x_msgs::msg::ChunkData::SharedPtr msg);

  // Operations on the chunk idx list
  void findMissedChunkIds(
      const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids);
  bool findIntersect(
      const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr);
  void mergeChunkIds(const vector<int>& list1, const vector<int>& list2, vector<int>& output);

  // data----------------

  int drone_id_, map_num_;
  int chunk_size_;
  double chunk_partial_flush_interval_;
  // Max ChunkData messages published in one sendChunks call - see the comment
  // there; an unbounded burst crashed FastDDS's writer.
  int max_chunks_per_send_ = 40;
  double stamp_reply_interval_ = 2.0;
  double last_adr_buffer_update_time_;

  SDFMap* map_ = nullptr;
  shared_ptr<CommunicationGraph> comm_graph_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<c2x_msgs::msg::ChunkStamps>::SharedPtr stamp_pub_;
  rclcpp::Publisher<c2x_msgs::msg::ChunkData>::SharedPtr chunk_pub_;
  rclcpp::Subscription<c2x_msgs::msg::ChunkStamps>::SharedPtr stamp_sub_;
  rclcpp::Subscription<c2x_msgs::msg::ChunkData>::SharedPtr chunk_sub_;
  rclcpp::TimerBase::SharedPtr stamp_timer_, chunk_timer_;
  // Gruppi di callback propri (vedi init): nel gruppo di serie la pianificazione li affamava.
  rclcpp::CallbackGroup::SharedPtr recv_cb_group_, timer_cb_group_;

  vector<ChunksData> multi_map_chunks_;                   // Main map data
  vector<uint32_t> adr_buffer_;                           // Buffer for chunks of this map
  vector<vector<c2x_msgs::msg::ChunkData>> chunk_buffer_;  // Buffer for chunks of external map
  vector<unordered_map<int, char>> buffer_map_;  // Avoid repeated insertion of the same chunk msg
  vector<double> last_chunk_stamp_time_;
  // Un pezzo gia' mandato a un compagno non si rimanda prima di resend_interval_ (s simulati):
  // prima ogni risposta rimandava gli STESSI pezzi mancanti finche' il compagno non li
  // confermava, e la sua coda si riempiva di doppioni (vedi sendChunks).
  double resend_interval_ = 2.0;
  // [destinatario-1][proprietario-1][idx-1] = istante dell'ultimo invio (sotto chunks_mutex_).
  vector<vector<vector<double>>> last_sent_;

  // Bounding box of map chunks of swarm
  vector<ChunksBox> chunk_boxes_;

  // multi_map_chunks_/adr_buffer_ are touched from the map's cloud thread
  // (updateMapChunk), the stamp timer, the chunk timer and both subscriptions -
  // all different callback groups under the MultiThreadedExecutor.
  std::mutex chunks_mutex_;
  std::mutex buffer_mutex_;
  std::mutex boxes_mutex_;

  friend SDFMap;
  friend MapROS;
};
}  // namespace c2_expl

#endif
