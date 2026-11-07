#ifndef _GRAPH_NODE_H_
#define _GRAPH_NODE_H_
#include <vector>
#include <unordered_map>
#include <queue>
#include <list>
#include <memory>
#include <mutex>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Eigen>

using Eigen::Vector3d;
using Eigen::Vector3i;
using std::cout;
using std::list;
using std::queue;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

namespace c2_expl {
// Basic noded type containing only general artributes required by graph search
class BaseNode {
public:
  typedef shared_ptr<BaseNode> Ptr;
  BaseNode() {
    g_value_ = 1000000;
    closed_ = false;
  }
  ~BaseNode() {
  }

  virtual void print() {
    std::cout << "Base node" << std::endl;
  }

  int id_;
  bool closed_;
  double g_value_;
};

// Node type for viewpoint refinement
class Astar;
class SDFMap;
class ViewNode : public BaseNode {
public:
  typedef shared_ptr<ViewNode> Ptr;
  ViewNode(const Vector3d& p, const double& y);
  ViewNode() {
  }
  ~ViewNode() {
  }

  virtual void print() {
    std::cout << "View node" << yaw_ << std::endl;
  }

  void printNeighbors() {
    for (auto v : neighbors_) v->print();
  }

  double costTo(const ViewNode::Ptr& node);
  static double computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1,
      const double& y2, const Vector3d& v1, const double& yd1, vector<Vector3d>& path);
  static double computeCostPos(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path);
  // Coarse to fine path searching
  static double searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path);
  static bool isUnreachableCost(const double& cost) { return cost >= kUnreachableCost; }

  // Data
  vector<ViewNode::Ptr> neighbors_;
  ViewNode::Ptr parent_;
  Vector3d pos_, vel_;
  double yaw_, yaw_dot_;

  // Parameters shared among nodes
  static double vm_, am_, yd_, ydd_, w_dir_;
  static const double kUnreachableCost;
  static shared_ptr<Astar> astar_;
  // UNA SOLA istanza di A* condivisa da tutte le ricerche di percorso, con
  // dentro liste aperte/chiuse e pool di nodi: usata da piu' thread
  // contemporaneamente il suo stato si corrompe, e l'errore emerge molto dopo
  // dentro una malloc qualsiasi. E' la firma
  // graph_node.cpp:searchPath -> astar2.cpp:clear che restava dopo aver
  // protetto griglia, grafo di connettivita' e frontiere.
  // Un lucchetto e non un'istanza per thread: le ricerche qui sono brevi
  // (risoluzione 0.4, con timeout), e istanze separate andrebbero anche
  // inizializzate con mappa e parametri.
  static std::mutex astar_mutex_;
  static shared_ptr<RayCaster> caster_;
  static shared_ptr<SDFMap> map_;
};
}  // namespace c2_expl
#endif
