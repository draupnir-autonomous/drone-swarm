#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <plan_env/communication_graph.h>

#include <algorithm>
#include <limits>

namespace {
template <typename T>
T declareIfNew(rclcpp::Node::SharedPtr node, const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) node->declare_parameter(name, default_value);
  return node->get_parameter(name).get_value<T>();
}
}  // namespace

namespace c2_expl {

MultiMapManager::MultiMapManager() {
}

MultiMapManager::~MultiMapManager() {
}

void MultiMapManager::setMap(SDFMap* map) {
  this->map_ = map;
}

void MultiMapManager::setCommunicationGraph(shared_ptr<CommunicationGraph> comm_graph) {
  this->comm_graph_ = comm_graph;
}

void MultiMapManager::init(rclcpp::Node::SharedPtr node) {
  node_ = node;

  drone_id_ = declareIfNew(node_, "exploration.drone_id", 1);
  map_num_ = declareIfNew(node_, "exploration.drone_num", 1);
  chunk_size_ = declareIfNew(node_, "multi_map_manager.chunk_size", 200);
  chunk_partial_flush_interval_ =
      declareIfNew(node_, "multi_map_manager.chunk_partial_flush_interval", 0.5);
  max_chunks_per_send_ = declareIfNew(node_, "multi_map_manager.max_chunks_per_send", 40);
  // Ogni quanto si risponde alla lista dei pezzi di un compagno (era fisso a 2 s). Vedi la
  // nota nelle configurazioni multi-drone: con 200 celle per pezzo, 40 pezzi per risposta e
  // una risposta ogni 2 s passavano ~4000 celle/s per coppia, contro le ~7700 celle/s NUOVE
  // che ogni drone osserva su cubicle_office; le mappe restavano indietro (a fine missione
  // ognuna conteneva l'80-82% dell'unione) e ogni drone esplorava di nuovo cio' che gli altri
  // avevano gia' visto (sciame, ottobre 2026).
  stamp_reply_interval_ = declareIfNew(node_, "multi_map_manager.stamp_reply_interval", 2.0);
  resend_interval_ = declareIfNew(node_, "multi_map_manager.resend_interval", 2.0);

  if (chunk_size_ <= 0) {
    RCLCPP_WARN(
        node_->get_logger(), "[MultiMapManager] Invalid chunk_size=%d, fallback to 200", chunk_size_);
    chunk_size_ = 200;
  }
  if (chunk_partial_flush_interval_ < 0.0) {
    chunk_partial_flush_interval_ = 0.0;
  }
  if (map_num_ < 1) map_num_ = 1;
  last_adr_buffer_update_time_ = node_->now().seconds();

  multi_map_chunks_.resize(map_num_);
  for (auto& data : multi_map_chunks_) {
    data.idx_list_ = {};
  }
  chunk_boxes_.resize(map_num_);
  for (auto& box : chunk_boxes_) {
    box.valid_ = false;
  }
  chunk_buffer_.resize(map_num_);
  buffer_map_.resize(map_num_);
  last_chunk_stamp_time_.assign(map_num_, 0.0);
  last_sent_.assign(map_num_, vector<vector<double>>(map_num_));

  // A single drone has nobody to synchronise with: skip the timers and the
  // topics entirely so the chunk bookkeeping above stays inert.
  if (map_num_ < 2) return;

  // Relative names - relayed across the per-drone namespaces by
  // c2x_px4_bridge's swarm_relay node, mirroring ROS1's swarm_bridge.
  stamp_pub_ =
      node_->create_publisher<c2x_msgs::msg::ChunkStamps>("multi_map_manager/chunk_stamps_send", 10);
  // Depth 5000 of ChunkData (200 voxel addresses + occupancies each) is a lot
  // of writer history to keep alive; the burst below is now bounded instead.
  // 1000 (era 200): con tre droni una risposta a un compagno sono fino a 120 pezzi (i miei e
  // quelli inoltrati) e se ne mandano a due compagni ogni stamp_reply_interval_; con 200 la
  // coda del writer scartava pezzi non ancora consegnati al relay (~7.5 MB a 1500 celle).
  chunk_pub_ =
      node_->create_publisher<c2x_msgs::msg::ChunkData>("multi_map_manager/chunk_data_send", 1000);

  // Sottoscrizioni e timer dello scambio NON stanno nel gruppo di callback di serie del nodo:
  // li' ci sono anche la fusione del lidar, l'ESDF, la macchina a stati e il calcolo delle
  // frontiere, che sul Jetson lo tengono occupato quasi sempre. I pezzi in arrivo restavano
  // in coda: a missione avviata un compagno aveva lo 0-10% dei pezzi degli altri su
  // empty_room e il 25-50% su office_small (riga [Mappe]), e i droni riesploravano cio' che
  // un altro aveva gia' visto (sciame, ottobre 2026). Qui tutto e' protetto dai mutex sotto;
  // insertChunkToMap scrive celle di un buffer di dimensione fissa, come la fusione del lidar.
  recv_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions recv_opts;
  recv_opts.callback_group = recv_cb_group_;
  stamp_sub_ = node_->create_subscription<c2x_msgs::msg::ChunkStamps>(
      "multi_map_manager/chunk_stamps_recv", 10,
      std::bind(&MultiMapManager::stampMsgCallback, this, std::placeholders::_1), recv_opts);
  chunk_sub_ = node_->create_subscription<c2x_msgs::msg::ChunkData>(
      "multi_map_manager/chunk_data_recv", rclcpp::QoS(rclcpp::KeepLast(5000)),
      std::bind(&MultiMapManager::chunkCallback, this, std::placeholders::_1), recv_opts);

  stamp_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.1),
      std::bind(&MultiMapManager::stampTimerCallback, this), timer_cb_group_);
  chunk_timer_ = node_->create_wall_timer(std::chrono::duration<double>(0.1),
      std::bind(&MultiMapManager::chunkTimerCallback, this), timer_cb_group_);
}

// Caller must hold chunks_mutex_.
void MultiMapManager::flushAdrBuffer(bool allow_partial) {
  if (drone_id_ < 1 || drone_id_ > map_num_) return;
  auto& own_chunks = multi_map_chunks_[drone_id_ - 1];

  const auto append_chunk = [&](const size_t begin, const size_t end) {
    MapChunk chunk;
    chunk.voxel_adrs_.insert(
        chunk.voxel_adrs_.end(), adr_buffer_.begin() + begin, adr_buffer_.begin() + end);
    chunk.idx_ = own_chunks.chunks_.size() + 1;
    chunk.need_query_ = true;
    chunk.empty_ = false;
    own_chunks.chunks_.push_back(std::move(chunk));
  };

  size_t processed = 0;
  while (processed + static_cast<size_t>(chunk_size_) <= adr_buffer_.size()) {
    append_chunk(processed, processed + static_cast<size_t>(chunk_size_));
    processed += static_cast<size_t>(chunk_size_);
  }

  if (allow_partial && processed < adr_buffer_.size()) {
    append_chunk(processed, adr_buffer_.size());
    processed = adr_buffer_.size();
  }

  if (processed > 0) {
    if (own_chunks.idx_list_.empty()) {
      own_chunks.idx_list_ = { 1, static_cast<int>(own_chunks.chunks_.back().idx_) };
    } else {
      own_chunks.idx_list_.back() = static_cast<int>(own_chunks.chunks_.back().idx_);
    }
    adr_buffer_.erase(adr_buffer_.begin(), adr_buffer_.begin() + processed);
  }
}

void MultiMapManager::updateMapChunk(const vector<uint32_t>& adrs) {
  if (map_num_ < 2) return;
  if (drone_id_ < 1 || drone_id_ > map_num_) {
    rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[MultiMapManager] Invalid drone_id=%d (drone_num=%d), skip chunk update", drone_id_,
        map_num_);
    return;
  }
  if (adrs.empty()) return;

  std::lock_guard<std::mutex> lock(chunks_mutex_);
  adr_buffer_.insert(adr_buffer_.end(), adrs.begin(), adrs.end());
  last_adr_buffer_update_time_ = node_->now().seconds();
  flushAdrBuffer(false);
}

bool MultiMapManager::canTransmitChunkTo(const int& other_drone_id) const {
  if (other_drone_id < 1 || other_drone_id > map_num_) return false;
  if (other_drone_id == drone_id_) return true;
  if (!comm_graph_) return true;
  return comm_graph_->isConnected(other_drone_id);
}

void MultiMapManager::stampTimerCallback() {
  const double now = node_->now().seconds();

  c2x_msgs::msg::ChunkStamps msg;
  msg.from_drone_id = drone_id_;
  msg.time = now;

  {
    std::lock_guard<std::mutex> lock(chunks_mutex_);
    if (!adr_buffer_.empty() &&
        now - last_adr_buffer_update_time_ >= chunk_partial_flush_interval_) {
      // Flush tail addresses to avoid permanent lag when update rate is low.
      flushAdrBuffer(true);
    }

    msg.idx_lists.reserve(multi_map_chunks_.size());
    for (const auto& chunks : multi_map_chunks_) {
      c2x_msgs::msg::IdxList idx_list;
      idx_list.ids = chunks.idx_list_;
      msg.idx_lists.push_back(idx_list);
    }
  }
  stamp_pub_->publish(msg);
}

void MultiMapManager::stampMsgCallback(const c2x_msgs::msg::ChunkStamps::SharedPtr msg) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  if (msg->from_drone_id < 1 || msg->from_drone_id > map_num_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[MultiMapManager] Ignore stamp from invalid drone_id=%d (drone_num=%d)",
        msg->from_drone_id, map_num_);
    return;
  }
  if (msg->from_drone_id == drone_id_) return;
  if (!canTransmitChunkTo(msg->from_drone_id)) return;

  // Throttle by local receive time to avoid cross-robot clock skew issues.
  const double now = node_->now().seconds();
  if (now - last_chunk_stamp_time_[msg->from_drone_id - 1] < stamp_reply_interval_) return;
  last_chunk_stamp_time_[msg->from_drone_id - 1] = now;

  if (msg->idx_lists.size() < static_cast<size_t>(map_num_)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[MultiMapManager] stamp idx_lists too short: from=%d got=%zu expect>=%d",
        msg->from_drone_id, msg->idx_lists.size(), map_num_);
  }

  // Diagnostica dello scambio: quanti dei MIEI pezzi ha questo compagno (dalla sua lista).
  if (drone_id_ >= 1 && drone_id_ <= static_cast<int>(msg->idx_lists.size())) {
    int suoi = 0, miei = 0;
    const auto& ids = msg->idx_lists[drone_id_ - 1].ids;
    for (size_t k = 0; k + 1 < ids.size(); k += 2) suoi += ids[k + 1] - ids[k] + 1;
    {
      std::lock_guard<std::mutex> lock(chunks_mutex_);
      miei = static_cast<int>(multi_map_chunks_[drone_id_ - 1].chunks_.size());
    }
    rclcpp::Clock orologio(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), orologio, 10000,
        "[Mappe] drone %d ha %d dei miei %d pezzi (%d celle per pezzo)", msg->from_drone_id, suoi,
        miei, chunk_size_);
  }

  // Check others' stamp info and send chunks unknown by them
  for (int i = 0; i < static_cast<int>(multi_map_chunks_.size()); ++i) {
    if (i == msg->from_drone_id - 1) continue;
    vector<int> missed;
    const vector<int> empty_ids;
    vector<int> self_ids;
    {
      std::lock_guard<std::mutex> lock(chunks_mutex_);
      self_ids = multi_map_chunks_[i].idx_list_;
    }
    const vector<int>& peer_ids =
        (i < static_cast<int>(msg->idx_lists.size())) ? msg->idx_lists[i].ids : empty_ids;
    findMissedChunkIds(self_ids, peer_ids, missed);
    sendChunks(i + 1, msg->from_drone_id, missed);
  }
}

void MultiMapManager::findMissedChunkIds(
    const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids) {
  // Compute the complement set of other idx
  if (other_idx_list.empty()) {
    miss_ids = self_idx_list;
    return;
  }

  vector<int> not_in_other;
  if (other_idx_list[0] > 1) {
    not_in_other.push_back(1);
    not_in_other.push_back(other_idx_list[0] - 1);
  }
  for (size_t i = 1; i < other_idx_list.size(); i += 2) {
    not_in_other.push_back(other_idx_list[i] + 1);
    if (i == other_idx_list.size() - 1) {
      int infinite = std::numeric_limits<int>::max();
      not_in_other.push_back(infinite);
    } else {
      not_in_other.push_back(other_idx_list[i + 1] - 1);
    }
  }

  // Compute the intersection of self and not_in_other (brute-force, O(n^2))
  for (size_t i = 0; i + 1 < self_idx_list.size(); i += 2) {
    for (size_t j = 0; j + 1 < not_in_other.size(); j += 2) {
      int minr, maxr;
      if (findIntersect(self_idx_list[i], self_idx_list[i + 1], not_in_other[j],
              not_in_other[j + 1], minr, maxr)) {
        miss_ids.push_back(minr);
        miss_ids.push_back(maxr);
      }
    }
  }
}

bool MultiMapManager::findIntersect(
    const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr) {
  minr = std::max(min1, min2);
  maxr = std::min(max1, max2);
  if (minr <= maxr) return true;
  return false;
}

void MultiMapManager::sendChunks(
    const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  if (chunk_drone_id < 1 || chunk_drone_id > map_num_ || to_drone_id < 1 || to_drone_id > map_num_) {
    return;
  }
  if (!canTransmitChunkTo(to_drone_id)) return;
  if (idx_list.empty()) return;
  if (idx_list.size() % 2 != 0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[MultiMapManager] Invalid idx_list length=%zu for sendChunks", idx_list.size());
    return;
  }

  std::lock_guard<std::mutex> lock(chunks_mutex_);
  auto& data = multi_map_chunks_[chunk_drone_id - 1];
  if (data.chunks_.empty()) return;

  // Bound the burst. When two drones first come into range one of them is
  // typically missing hundreds of chunks, and this used to publish every one of
  // them back to back inside a single subscription callback. That burst was
  // enough to take FastDDS down: an exploration_node segfaulted inside
  // __rmw_publish -> DataWriterImpl::write -> RTPSMessageGroup::send. The peer
  // re-advertises what it still lacks every 2s (last_chunk_stamp_time_), so
  // capping here only spreads the same transfer over a few adverts.
  int sent = 0;
  // Pezzi in volo: chi e' gia' stato mandato a questo compagno da meno di resend_interval_
  // si salta e si passa ai successivi. Prima ogni risposta rimandava gli stessi primi
  // max_chunks_per_send_ pezzi mancanti finche' il compagno non li aveva inseriti e
  // annunciati: la sua coda si riempiva di copie e il trasferimento restava fermo (a 60
  // pezzi per decine di secondi, sciame ottobre 2026). Ora i pezzi scorrono in fila; uno
  // perso si rimanda dopo resend_interval_.
  const double now = node_->now().seconds();
  auto& inviati = last_sent_[to_drone_id - 1][chunk_drone_id - 1];
  if (inviati.size() < data.chunks_.size()) inviati.resize(data.chunks_.size(), -1e9);

  for (size_t i = 0; i + 1 < idx_list.size() && sent < max_chunks_per_send_; i += 2) {
    int begin = std::max(1, idx_list[i]);
    int end = idx_list[i + 1];
    if (begin > end) continue;
    if (begin > static_cast<int>(data.chunks_.size())) continue;
    end = std::min(end, static_cast<int>(data.chunks_.size()));

    for (int j = begin; j <= end && sent < max_chunks_per_send_; ++j) {
      auto& chunk = data.chunks_[j - 1];
      if (chunk.empty_) continue;
      if (now >= inviati[j - 1] && now - inviati[j - 1] < resend_interval_) continue;
      // Pezzo di un terzo drone: lo si inoltra solo se e' qui da almeno resend_interval_.
      // Se il proprietario e' collegato al destinatario glielo ha gia' mandato lui, e la
      // copia inoltrata raddoppiava il traffico (con tre droni tutti collegati, ogni pezzo
      // arrivava due volte); se non lo e', l'inoltro parte con un ritardo di pochi secondi.
      if (chunk_drone_id != drone_id_ && now >= chunk.t_ins_ &&
          now - chunk.t_ins_ < resend_interval_)
        continue;

      c2x_msgs::msg::ChunkData msg;
      msg.from_drone_id = drone_id_;
      msg.to_drone_id = to_drone_id;
      msg.chunk_drone_id = chunk_drone_id;
      msg.idx = chunk.idx_;
      msg.voxel_adrs = chunk.voxel_adrs_;
      if (chunk_drone_id == drone_id_ && chunk.need_query_) {
        // Should query the occ info in map if they are still empty
        getOccOfChunk(chunk.voxel_adrs_, chunk.voxel_occ_);
        chunk.need_query_ = false;
      }
      if (chunk.voxel_occ_.size() != chunk.voxel_adrs_.size()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
            "[MultiMapManager] Skip malformed chunk owner=%d idx=%u adrs=%zu occ=%zu",
            chunk_drone_id, chunk.idx_, chunk.voxel_adrs_.size(), chunk.voxel_occ_.size());
        continue;
      }
      msg.voxel_occ = chunk.voxel_occ_;

      chunk_pub_->publish(msg);
      inviati[j - 1] = now;
      ++sent;
    }
  }
}

void MultiMapManager::getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs) {
  occs.clear();
  occs.reserve(adrs.size());
  // Controllo di intervallo sull'indirizzo: qui si indicizzava crudo, e un
  // valore fuori range non fallisce - legge memoria di qualcun altro. E' la
  // stessa classe di difetto gia' corretta in sdf_map.cpp (inputPointCloud), e
  // qui produceva il crash getOccOfChunk <- sendChunks <- stampMsgCallback.
  const size_t buf_size = map_->md_->occupancy_buffer_.size();
  for (auto adr : adrs) {
    if (adr >= buf_size) {
      occs.push_back(0);  // fuori mappa: si dichiara libero, non si legge
      continue;
    }
    uint8_t occ = map_->md_->occupancy_buffer_[adr] > map_->mp_->min_occupancy_log_ ? 1 : 0;
    occs.push_back(occ);
  }
}

void MultiMapManager::chunkCallback(const c2x_msgs::msg::ChunkData::SharedPtr msg) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  if (msg->from_drone_id < 1 || msg->from_drone_id > map_num_ || msg->to_drone_id < 1 ||
      msg->to_drone_id > map_num_ || msg->chunk_drone_id < 1 || msg->chunk_drone_id > map_num_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[MultiMapManager] Ignore invalid chunk ids: from=%d to=%d chunk_owner=%d (drone_num=%d)",
        msg->from_drone_id, msg->to_drone_id, msg->chunk_drone_id, map_num_);
    return;
  }

  // Receive chunks from other drones, store them in chunk buffer
  if (msg->from_drone_id == drone_id_) return;
  // Unicast filter: only process chunks addressed to this drone
  if (msg->to_drone_id != drone_id_) return;
  if (!canTransmitChunkTo(msg->from_drone_id)) return;
  if (msg->idx == 0) return;
  // Pezzo gia' inserito (copia arrivata in ritardo): si scarta subito, senza copiarlo.
  {
    std::lock_guard<std::mutex> lock(chunks_mutex_);
    const auto& tenuti = multi_map_chunks_[msg->chunk_drone_id - 1].chunks_;
    if (msg->idx <= tenuti.size() && !tenuti[msg->idx - 1].empty_) return;
  }
  if (msg->voxel_adrs.size() != msg->voxel_occ.size()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[MultiMapManager] Drop malformed chunk from=%d owner=%d idx=%u adrs=%zu occ=%zu",
        msg->from_drone_id, msg->chunk_drone_id, msg->idx, msg->voxel_adrs.size(),
        msg->voxel_occ.size());
    return;
  }

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  // Ignore chunks that are in the insertion buffer
  if (buffer_map_[msg->chunk_drone_id - 1].find(msg->idx) !=
      buffer_map_[msg->chunk_drone_id - 1].end())
    return;

  chunk_buffer_[msg->chunk_drone_id - 1].push_back(*msg);
  buffer_map_[msg->chunk_drone_id - 1][msg->idx] = 1;
}

void MultiMapManager::chunkTimerCallback() {
  // Process chunks in buffers
  for (int i = 0; i < static_cast<int>(chunk_buffer_.size()); ++i) {
    vector<c2x_msgs::msg::ChunkData> buffer;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (chunk_buffer_[i].empty()) continue;
      buffer.swap(chunk_buffer_[i]);
      buffer_map_[i].clear();
    }

    // Compute the idx list of buffered chunks
    std::sort(buffer.begin(), buffer.end(),
        [](const c2x_msgs::msg::ChunkData& chunk1, const c2x_msgs::msg::ChunkData& chunk2) {
          return chunk1.idx < chunk2.idx;
        });
    if (buffer.front().idx == 0) continue;

    vector<int> idx_list = { int(buffer.front().idx) };
    int last_idx = idx_list[0];
    for (size_t j = 1; j < buffer.size(); ++j) {
      if (buffer[j].idx == 0) continue;
      if (static_cast<int>(buffer[j].idx) - last_idx > 1) {
        idx_list.push_back(last_idx);
        idx_list.push_back(buffer[j].idx);
      }
      last_idx = buffer[j].idx;
    }
    if (last_idx <= 0) continue;
    idx_list.push_back(last_idx);

    std::lock_guard<std::mutex> lock(chunks_mutex_);
    auto& chunks_data = multi_map_chunks_[i];

    // Add placeholder for chunks
    const int len_inc = last_idx - static_cast<int>(chunks_data.chunks_.size());
    for (int j = 0; j < len_inc; ++j) {
      chunks_data.chunks_.push_back(MapChunk());
      auto& back_chunk = chunks_data.chunks_.back();
      back_chunk.idx_ = chunks_data.chunks_.size();
      back_chunk.empty_ = true;
      back_chunk.need_query_ = false;
    }

    // Process data in buffer
    for (const auto& msg : buffer) {
      if (msg.idx == 0 || msg.idx > chunks_data.chunks_.size()) continue;
      auto& chunk = chunks_data.chunks_[msg.idx - 1];
      if (chunk.empty_) {  // Only insert a chunk once
        if (msg.voxel_adrs.size() != msg.voxel_occ.size()) continue;
        chunk.voxel_adrs_ = msg.voxel_adrs;
        chunk.voxel_occ_ = msg.voxel_occ;
        insertChunkToMap(chunk, msg.chunk_drone_id);
        chunk.empty_ = false;
        chunk.t_ins_ = node_->now().seconds();
      }
    }

    // Update ChunksData's idx_list_
    vector<int> union_list;
    mergeChunkIds(idx_list, chunks_data.idx_list_, union_list);
    chunks_data.idx_list_ = union_list;
  }
}

void MultiMapManager::mergeChunkIds(
    const vector<int>& list1, const vector<int>& list2, vector<int>& output) {
  if (list1.empty()) {
    output = list2;
    return;
  }

  output = list1;
  int tmp1, tmp2;
  for (size_t i = 0; i + 1 < list2.size(); i += 2) {
    // For each interval in list2, merge it into output list
    bool intersect = false;
    for (size_t j = 0; j + 1 < output.size(); j += 2) {
      if (findIntersect(output[j], output[j + 1], list2[i], list2[i + 1], tmp1, tmp2)) {
        output[j] = std::min(output[j], list2[i]);
        output[j + 1] = std::max(output[j + 1], list2[i + 1]);
        intersect = true;
      }
    }
    if (!intersect) {  // Insert the interval in appropriate position
      vector<int> tmp = { list2[i], list2[i + 1] };
      if (list2[i + 1] < output.front()) {
        output.insert(output.begin(), tmp.begin(), tmp.end());
      } else if (list2[i] > output.back()) {
        output.insert(output.end(), tmp.begin(), tmp.end());
      } else {
        for (auto iter = output.begin() + 1; iter != output.end(); iter += 2) {
          if (*iter < list2[i] && *(iter + 1) > list2[i + 1]) {
            output.insert(iter + 1, tmp.begin(), tmp.end());
            break;
          }
        }
      }
    }
    // Remove redundant idx
    if (output.size() >= 4) {
      for (auto iter = output.begin() + 1; iter < output.end() - 1;) {
        if (*iter >= *(iter + 1) - 1) {
          iter = output.erase(iter);
          iter = output.erase(iter);
        } else {
          iter += 2;
        }
      }
    }
  }
}

void MultiMapManager::adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx) {
  // x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z
  uint32_t tmp_adr = adr;
  const int a = map_->mp_->map_voxel_num_[1] * map_->mp_->map_voxel_num_[2];
  const int b = map_->mp_->map_voxel_num_[2];

  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
}

void MultiMapManager::insertChunkToMap(const MapChunk& chunk, const int& drone_id) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  if (drone_id < 1 || drone_id > map_num_) return;
  if (chunk.voxel_adrs_.size() != chunk.voxel_occ_.size()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), clock, 1000,
        "[MultiMapManager] Skip insert malformed chunk owner=%d idx=%u", drone_id, chunk.idx_);
    return;
  }

  // All drones share one world frame here (PX4/Gazebo spawns them into the same
  // ENU origin), so no per-drone base-coordinate transform is applied - the
  // ROS1 original kept that step commented out for the same reason.
  const int inf_step =
      static_cast<int>(std::ceil(map_->mp_->obstacles_inflation_ / map_->mp_->resolution_));

  for (size_t i = 0; i < chunk.voxel_adrs_.size(); ++i) {
    auto& adr = chunk.voxel_adrs_[i];

    Eigen::Vector3i idx;
    adrToIndex(adr, idx);

    Eigen::Vector3d pos;
    map_->indexToPos(idx, pos);

    if (!map_->isInMap(pos)) continue;

    map_->posToIndex(pos, idx);
    auto adr_tf = map_->toAddress(idx);

    map_->md_->occupancy_buffer_[adr_tf] =
        chunk.voxel_occ_[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;

    // Update the chunk box
    {
      std::lock_guard<std::mutex> lock(boxes_mutex_);
      if (chunk_boxes_[drone_id - 1].valid_) {
        for (int k = 0; k < 3; ++k) {
          chunk_boxes_[drone_id - 1].min_[k] =
              std::min(chunk_boxes_[drone_id - 1].min_[k], pos[k]);
          chunk_boxes_[drone_id - 1].max_[k] =
              std::max(chunk_boxes_[drone_id - 1].max_[k], pos[k]);
        }
      } else {
        chunk_boxes_[drone_id - 1].min_ = chunk_boxes_[drone_id - 1].max_ = pos;
        chunk_boxes_[drone_id - 1].valid_ = true;
      }
    }

    // Update the all box
    for (int k = 0; k < 3; ++k) {
      map_->md_->all_min_[k] = std::min(map_->md_->all_min_[k], pos[k]);
      map_->md_->all_max_[k] = std::max(map_->md_->all_max_[k], pos[k]);
    }
    // Inflate for the occupied
    if (chunk.voxel_occ_[i] == 1) {
      // occupancy_buffer_inflate_ is concurrently written by
      // SDFMap::clearAndInflateLocalMap() on the cloud thread and read by
      // checkTrajCollision() on its own callback group - see the mutex comment
      // in sdf_map.h.
      std::lock_guard<std::mutex> lock(map_->occ_inflate_mutex_);
      for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
        for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
          for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z) {
            Eigen::Vector3i inf_pt(idx[0] + inf_x, idx[1] + inf_y, idx[2] + inf_z);
            if (!map_->isInMap(inf_pt)) continue;
            int inf_adr = map_->toAddress(inf_pt);
            map_->md_->occupancy_buffer_inflate_[inf_adr] = 1;
          }
    }
  }
}

void MultiMapManager::getChunkBoxes(
    vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset) {
  std::lock_guard<std::mutex> lock(boxes_mutex_);
  for (auto& box : chunk_boxes_) {
    if (box.valid_) {
      mins.push_back(box.min_);
      maxs.push_back(box.max_);
      if (reset) box.valid_ = false;
    }
  }
}

}  // namespace c2_expl
