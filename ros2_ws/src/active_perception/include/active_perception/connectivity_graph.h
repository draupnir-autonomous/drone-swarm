#ifndef _CONNECTIVITY_GRAPH_H_
#define _CONNECTIVITY_GRAPH_H_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

namespace c2_expl {

using Position = Eigen::Vector3d;

struct ConnectivityEdge {
  typedef std::shared_ptr<ConnectivityEdge> Ptr;
  typedef std::shared_ptr<const ConnectivityEdge> ConstPtr;

  enum class TYPE { UNKNOWN, FREE, PORTAL };

  ConnectivityEdge() {}
  ~ConnectivityEdge() {}

  TYPE type_;
  int id1_, id2_;
  double cost_;

  std::vector<Position> path_;
};

struct ConnectivityNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ConnectivityNode> Ptr;
  typedef std::shared_ptr<const ConnectivityNode> ConstPtr;

  enum class TYPE { UNKNOWN, FREE };

  ConnectivityNode() {}
  ConnectivityNode(const int& id, const Position& pos, const TYPE& type) : id_(id), pos_(pos),
                                                                           type_(type) {}
  ~ConnectivityNode() {}

  void addNeighbor(const int& neighbor_id, const double& cost, const ConnectivityEdge::TYPE& type) {
    ConnectivityEdge edge;
    edge.id1_ = id_;
    edge.id2_ = neighbor_id;
    edge.cost_ = cost;
    edge.type_ = type;

    // Remove old edge with same neighbor id
    for (auto it = neighbors_.begin(); it != neighbors_.end(); ++it) {
      if (it->id2_ == neighbor_id) {
        neighbors_.erase(it);
        break;
      }
    }

    neighbors_.push_back(edge);
  }

  void addNeighborWithPath(const int& neighbor_id, const double& cost,
      const ConnectivityEdge::TYPE& type, const std::vector<Position>& path) {
    ConnectivityEdge edge;
    edge.id1_ = id_;
    edge.id2_ = neighbor_id;
    edge.cost_ = cost;
    edge.type_ = type;
    edge.path_ = path;

    // Remove old edge with same neighbor id
    for (auto it = neighbors_.begin(); it != neighbors_.end(); ++it) {
      if (it->id2_ == neighbor_id) {
        neighbors_.erase(it);
        break;
      }
    }

    neighbors_.push_back(edge);
  }

  void removeNeighbor(const int& neighbor_id) {
    for (auto it = neighbors_.begin(); it != neighbors_.end(); ++it) {
      if (it->id2_ == neighbor_id) {
        neighbors_.erase(it);
        break;
      }
    }
  }

  void clearNeighbors() { neighbors_.clear(); }

  TYPE type_;
  int id_;
  Position pos_;  // actual position of the node

  std::vector<ConnectivityEdge> neighbors_;  // id, cost (distance)
};

class ConnectivityGraph {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ConnectivityGraph> Ptr;
  typedef std::shared_ptr<const ConnectivityGraph> ConstPtr;

  ConnectivityGraph() {}
  ~ConnectivityGraph() {}

  ConnectivityGraph(rclcpp::Node::SharedPtr node) {}

  // Lucchetto esportabile: getNode() consegna un puntatore al nodo, e chi lo
  // riceve ne modifica i vicini FUORI dalla protezione di nodes_mutex_ mentre
  // un altro thread puo' rimuoverlo o ricostruirlo. Proteggere la mappa non
  // basta: va tenuto il lucchetto per tutta la sequenza leggi-modifica.
  std::unique_lock<std::recursive_mutex> lockNodes() const {
    return std::unique_lock<std::recursive_mutex>(nodes_mutex_);
  }

  void addNode(ConnectivityNode::Ptr node);
  void removeNode(const int& id);
  void clearNodes();
  ConnectivityNode::Ptr getNode(const int& id);
  void getNodeNum(int& num);
  int getNodeNum();
  void getNodePositions(std::vector<Position>& node_positions);
  void getNodePositionsWithIDs(std::vector<Position>& node_positions, std::vector<int>& node_ids);

  double searchConnectivityGraphBFS(const int& id1, const int& id2, std::vector<int>& path);
  double searchConnectivityGraphAStar(const int& id1, const int& id2, std::vector<int>& path);
  void getFullConnectivityGraph(std::vector<std::pair<Position, Position>>& edges,
      std::vector<ConnectivityEdge::TYPE>& edges_types, std::vector<double>& edges_costs);
  void getFullConnectivityGraphPath(
      std::vector<std::vector<Position>>& paths, std::vector<ConnectivityEdge::TYPE>& edges_types);

  void findDisconnectedNodes(std::set<int>& disconnected_nodes);

  bool saveConnectivityGraph(const std::string& file_path);
  bool loadConnectivityGraph(const std::string& file_path);

private:
  std::unordered_map<int, ConnectivityNode::Ptr> nodes_;

  // Protegge nodes_. Il grafo e' condiviso: UniformGrid::getConnectivityGraph()
  // ne restituisce il puntatore, e i chiamanti lo usano fuori dal lucchetto
  // della griglia - quindi l'oggetto sfugge a QUELLA protezione e va difeso da
  // se'. E' il residuo della corsa che produceva l'abort con firma
  // updateGridData -> updateConnectivityGraph -> heap corrotto.
  // Deve stare QUI e non a monte: un lucchetto sull'interfaccia di HGrid
  // e' gia' stato provato e rimosso, perche' le sue sezioni critiche durano
  // quanto la costruzione del grafo (2.4 s) e mandavano in stallo l'FSM.
  // Qui invece si tiene solo per il tempo di toccare la mappa dei nodi.
  mutable std::recursive_mutex nodes_mutex_;

  double max_search_time_ = 1e-4;
};
}  // namespace c2_expl

#endif
