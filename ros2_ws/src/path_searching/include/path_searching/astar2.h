#ifndef _ASTAR2_H
#define _ASTAR2_H

#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <chrono>
#include <cstdint>
#include <mutex>
#include "plan_env/edt_environment.h"
#include <path_searching/matrix_hash.h>
namespace c2_expl {
class Node {
public:
  Eigen::Vector3i index;
  Eigen::Vector3d position;
  double g_score, f_score;
  Node* parent;

  /* -------------------- */
  Node() {
    parent = NULL;
  }
  ~Node() {};
};
typedef Node* NodePtr;

class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class Astar {
public:
  Astar();
  ~Astar();
  enum { REACH_END = 1, NO_PATH = 2 };

  void init(rclcpp::Node::SharedPtr node, const EDTEnvironment::Ptr& env);
  void reset();
  int search(
      const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt, bool optimistic = true);
  int searchUnknownBBox(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt,
      const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max);
  int searchUnknownOnlyBBox(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt,
      const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max);
  void setResolution(const double& res);
  static double pathLength(const vector<Eigen::Vector3d>& path);

  std::vector<Eigen::Vector3d> getPath();

  double lambda_heu_;
  double max_search_time_;
  // Vero se l'ultima search() e' terminata per TEMPO SCADUTO e non perche' il
  // percorso non esiste: le due risposte erano entrambe NO_PATH e chi chiamava
  // non poteva distinguerle.
  bool last_timed_out_ = false;
  // search() ripetuta con un tempo piu' lungo se la prima scade per tempo.
  int searchPatient(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt,
      double patient_time, bool optimistic = true);
  // Fattore di lentezza della CPU (prestazioni.fattore_tempo_cpu, 1 sul PC): moltiplica
  // i tetti di tempo, cosi' una ricerca esplora gli stessi nodi su ogni macchina.
  double fattore_tempo_ = 1.0;
  // Quota minima dei percorsi (astar.quota_minima, coordinate mondo; spento di
  // serie): vedi search().
  double quota_minima_ = -1e9;

private:
  // Nodo i del pool, creato al primo uso (vedi init()).
  NodePtr nodoDelPool(int i);
  void backtrack(const NodePtr& end_node, const Eigen::Vector3d& end);
  void posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx);
  double getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);

  // main data structure
  vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_;
  std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;

  // INSIEMI APERTO/CHIUSO SU ARRAY DENSI (Jetson, ottobre 2026). Le due
  // unordered_map qui sopra costavano tre ricerche hash per vicino (26 per nodo):
  // con l'A* all'80% del tempo della pianificazione sul Jetson era la parte piu'
  // pesante dopo i lock. Qui lo stato di ogni cella della griglia dell'A* sta in
  // un array con un contatore di generazione (reset() = ++generazione_, niente da
  // svuotare). Stessa semantica delle mappe, quindi stessi percorsi; le celle fuori
  // dalla griglia (partenza fuori mappa) usano ancora le mappe.
  enum : uint8_t { ASSENTE = 0, APERTO = 1, CHIUSO = 2 };
  std::vector<uint32_t> gen_cella_;
  std::vector<uint8_t> stato_cella_;
  std::vector<NodePtr> nodo_cella_;
  uint32_t generazione_ = 1;
  Eigen::Vector3i dim_griglia_ = Eigen::Vector3i::Zero();
  void dimensionaGriglia();
  inline int cella(const Eigen::Vector3i& id) const {
    if (id(0) < 0 || id(1) < 0 || id(2) < 0 || id(0) >= dim_griglia_(0) ||
        id(1) >= dim_griglia_(1) || id(2) >= dim_griglia_(2))
      return -1;
    return (id(0) * dim_griglia_(1) + id(1)) * dim_griglia_(2) + id(2);
  }
  inline NodePtr trovaAperto(const Eigen::Vector3i& id) {
    const int c = cella(id);
    if (c < 0) {
      auto it = open_set_map_.find(id);
      return it == open_set_map_.end() ? nullptr : it->second;
    }
    return (gen_cella_[c] == generazione_ && stato_cella_[c] == APERTO) ? nodo_cella_[c] : nullptr;
  }
  inline void mettiAperto(const Eigen::Vector3i& id, NodePtr n) {
    const int c = cella(id);
    if (c < 0) {
      open_set_map_[id] = n;
      return;
    }
    gen_cella_[c] = generazione_;
    stato_cella_[c] = APERTO;
    nodo_cella_[c] = n;
  }
  // Toglie dall'aperto e mette nel chiuso (open_set_map_.erase + close_set_map_.insert).
  inline void chiudi(const Eigen::Vector3i& id) {
    const int c = cella(id);
    if (c < 0) {
      open_set_map_.erase(id);
      close_set_map_.insert(std::make_pair(id, 1));
      return;
    }
    gen_cella_[c] = generazione_;
    stato_cella_[c] = CHIUSO;
  }
  inline bool eChiuso(const Eigen::Vector3i& id) {
    const int c = cella(id);
    if (c < 0) return close_set_map_.find(id) != close_set_map_.end();
    return gen_cella_[c] == generazione_ && stato_cella_[c] == CHIUSO;
  }
  std::vector<Eigen::Vector3d> path_nodes_;

  EDTEnvironment::Ptr edt_env_;
  rclcpp::Clock::SharedPtr clock_;

  // parameter
  int allocate_num_;
  double tie_breaker_;
  double resolution_, inv_resolution_;
  Eigen::Vector3d map_size_3d_, origin_;
};

}  // namespace c2_expl

#endif
