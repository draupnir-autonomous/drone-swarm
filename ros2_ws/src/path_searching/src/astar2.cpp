#include <path_searching/astar2.h>
#include <algorithm>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace {
// C2ExplorationManager creates a second Astar instance (ViewNode::astar_)
// on top of FastPlannerManager's own path_finder_, both initialized on the
// same node; ROS2's declare_parameter throws on a repeated name.
template <typename T>
void declareIfNew(rclcpp::Node::SharedPtr node, const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) node->declare_parameter(name, default_value);
}
}  // namespace

namespace c2_expl {
Astar::Astar() {
}

Astar::~Astar() {
  for (size_t i = 0; i < path_node_pool_.size(); i++) delete path_node_pool_[i];
}

NodePtr Astar::nodoDelPool(int i) {
  if (i >= static_cast<int>(path_node_pool_.size())) {
    const int nuovo = std::min(allocate_num_,
        std::max({ i + 1, 2 * static_cast<int>(path_node_pool_.size()), 4096 }));
    path_node_pool_.reserve(nuovo);
    while (static_cast<int>(path_node_pool_.size()) < nuovo) path_node_pool_.push_back(new Node);
  }
  return path_node_pool_[i];
}

void Astar::init(rclcpp::Node::SharedPtr node, const EDTEnvironment::Ptr& env) {
  declareIfNew(node, "astar.resolution_astar", -1.0);
  declareIfNew(node, "astar.lambda_heu", -1.0);
  declareIfNew(node, "astar.max_search_time", -1.0);
  declareIfNew(node, "astar.allocate_num", -1);
  node->get_parameter("astar.resolution_astar", resolution_);
  node->get_parameter("astar.lambda_heu", lambda_heu_);
  node->get_parameter("astar.max_search_time", max_search_time_);
  node->get_parameter("astar.allocate_num", allocate_num_);
  // TEMPO DI RICERCA SCALATO CON LA CPU (Jetson Orin Nano, ottobre 2026). Il tetto
  // astar.max_search_time e' in secondi di orologio, tarato sul PC: sul Jetson,
  // ~1.8-2 volte piu' lento, nello stesso tempo una ricerca esplora la meta' dei
  // nodi, e percorsi che sul PC si trovano li' scadevano: celle raggiungibili
  // trattate da irraggiungibili, giri piu' lunghi (cubicle_office: 402 m volati
  // contro 257). Il launch mette il fattore a 2 su aarch64, 1 altrove.
  declareIfNew(node, "prestazioni.fattore_tempo_cpu", 1.0);
  node->get_parameter("prestazioni.fattore_tempo_cpu", fattore_tempo_);
  if (fattore_tempo_ > 0.0) max_search_time_ *= fattore_tempo_;
  declareIfNew(node, "astar.quota_minima", -1e9);
  node->get_parameter("astar.quota_minima", quota_minima_);

  tie_breaker_ = 1.0 + 1.0 / 1000;
  clock_ = node->get_clock();

  this->edt_env_ = env;

  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  edt_env_->sdf_map_->getRegion(origin_, map_size_3d_);
  dimensionaGriglia();
  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  // Pool creato man mano (fino ad allocate_num_ nodi, come prima) invece che tutto
  // subito: un milione di nodi sono ~100 MB per istanza, e il grafo di connettivita'
  // ne usa una per thread.
  path_node_pool_.clear();
  use_node_num_ = 0;
  iter_num_ = 0;
}

void Astar::setResolution(const double& res) {
  if (res == resolution_) return;
  resolution_ = res;
  this->inv_resolution_ = 1.0 / resolution_;
  dimensionaGriglia();
}

void Astar::dimensionaGriglia() {
  // Griglia dell'A* sulla mappa (stessi indici di posToIndex). Oltre ~20 M celle
  // (risoluzioni fini) si resta sulle mappe hash per non allocare troppo.
  Eigen::Vector3i d;
  for (int k = 0; k < 3; ++k) d(k) = static_cast<int>(std::ceil(map_size_3d_(k) * inv_resolution_)) + 1;
  const long n = static_cast<long>(d(0)) * d(1) * d(2);
  if (resolution_ <= 0.0 || n <= 0 || n > 20000000L) {
    dim_griglia_.setZero();
    return;
  }
  dim_griglia_ = d;
  if (static_cast<long>(gen_cella_.size()) < n) {
    gen_cella_.assign(n, 0u);
    stato_cella_.assign(n, ASSENTE);
    nodo_cella_.assign(n, nullptr);
  } else {
    std::fill(gen_cella_.begin(), gen_cella_.end(), 0u);  // indici cambiati: niente da riusare
  }
}

int Astar::searchPatient(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt,
    double patient_time, bool optimistic) {
  if (fattore_tempo_ > 0.0) patient_time *= fattore_tempo_;
  reset();
  int res = search(start_pt, end_pt, optimistic);
  if (res == REACH_END || !last_timed_out_ || patient_time <= max_search_time_) return res;
  // Il tetto normale (astar.max_search_time, 10 ms) e' tarato sulle centinaia di
  // ricerche del calcolo dei costi fra frontiere. Per una singola ricerca da
  // cui dipende se il drone si muove o resta fermo non basta: in office_small un
  // tragitto di 8.5 m fra le pareti scadeva sempre, e il drone restava in
  // hovering a ripianificare verso un punto raggiungibile (run V4_LIO 2, 30 s).
  const double normale = max_search_time_;
  max_search_time_ = patient_time;
  reset();
  res = search(start_pt, end_pt, optimistic);
  max_search_time_ = normale;
  return res;
}

int Astar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt, bool optimistic) {
  last_timed_out_ = false;

  NodePtr cur_node = nodoDelPool(0);
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  mettiAperto(cur_node->index, cur_node);
  use_node_num_ += 1;

  // PAVIMENTO DEI PERCORSI (astar.quota_minima = min_flight_z del ponte, 1.0 m).
  // Il ponte non lascia mai il riferimento sotto quella quota, ma il percorso
  // la ignorava: nel parco passava SOTTO la piattaforma del parco giochi (fondo
  // a ~1.34 m) e sotto le chiome, dove l'ottimizzatore (optimization.dist0 0.7
  // dal terreno E dalla piattaforma) metteva la traiettoria a meta', 0.62-0.69 m.
  // Il filtro di sicurezza del ponte allontanava il riferimento dalla
  // piattaforma spingendolo in basso: drone a 0.38-0.43 m sul terreno (run
  // 2026-10-18 0054, 1103, 1153, 1320). Con il pavimento, sotto la piattaforma
  // non resta posto (la fascia gonfiata scende a ~1.0 m) e il percorso gira
  // intorno o sopra. Mai sotto il pavimento, ne' sotto la partenza o l'arrivo
  // se stanno piu' in basso (decollo a 0.85 m, punti di riferimento della griglia):
  // quelle ricerche restano possibili come prima.
  const double z_min = std::min({ quota_minima_, start_pt.z(), end_pt.z() }) - 1e-3;

  const auto t1 = std::chrono::steady_clock::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                     abs(cur_node->index(1) - end_index(1)) <= 1 &&
                     abs(cur_node->index(2) - end_index(2)) <= 1;
    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    // Early termination if time up
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count() > max_search_time_) {
      last_timed_out_ = true;
      return NO_PATH;
    }

    open_set_.pop();
    chiudi(cur_node->index);
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;
    // Un solo lock della mappa gonfiata per nodo espanso (prima ~130, uno per lettura).
    std::lock_guard<std::mutex> lk_infl(edt_env_->sdf_map_->inflateMutex());

    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < 1e-3) continue;
          nbr_pos = cur_pos + step;
          // Check safety
          if (!edt_env_->sdf_map_->isInBox(nbr_pos)) {
            continue;
          }
          if (nbr_pos.z() < z_min) continue;

          if (edt_env_->sdf_map_->getInflateOccupancyNoLock(nbr_pos) == 1 ||
              (!optimistic && edt_env_->sdf_map_->getOccupancy(nbr_pos) == SDFMap::UNKNOWN))
            continue;

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l <= len + 1e-2; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (edt_env_->sdf_map_->getInflateOccupancyNoLock(ckpt) == 1 ||
                (!optimistic && edt_env_->sdf_map_->getOccupancy(ckpt) == SDFMap::UNKNOWN)) {
              safe = false;
              break;
            }
          }
          if (!safe) continue;

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (eChiuso(nbr_idx)) continue;

          NodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          NodePtr esistente = trovaAperto(nbr_idx);
          if (!esistente) {
            neighbor = nodoDelPool(use_node_num_);
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              cout << "run out of node pool." << endl;
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < esistente->g_score) {
            neighbor = esistente;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          mettiAperto(nbr_idx, neighbor);
        }
  }
  return NO_PATH;
}

int Astar::searchUnknownBBox(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt,
    const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max) {
  NodePtr cur_node = nodoDelPool(0);
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  mettiAperto(cur_node->index, cur_node);
  use_node_num_ += 1;

  const auto t1 = std::chrono::steady_clock::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) && cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    // Early termination if time up
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count() > max_search_time_) {
      return NO_PATH;
    }

    open_set_.pop();
    chiudi(cur_node->index);
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;
    // Un solo lock della mappa gonfiata per nodo espanso (prima ~130, uno per lettura).
    std::lock_guard<std::mutex> lk_infl(edt_env_->sdf_map_->inflateMutex());

    for (double dx = -resolution_ + 1e-3; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_ + 1e-3; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_ + 1e-3; dz <= resolution_ + 1e-3; dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < 1e-3) continue;
          nbr_pos = cur_pos + step;
          // Check safety - only X and Y since grid is 2D
          auto isInBox = [&](const Eigen::Vector3d& pos) {
            return pos(0) >= bbox_min(0) && pos(0) <= bbox_max(0) && pos(1) >= bbox_min(1) &&
                   pos(1) <= bbox_max(1);
          };
          if (!isInBox(nbr_pos) || !edt_env_->sdf_map_->isInBox(nbr_pos)) {
            continue;
          }

          if (edt_env_->sdf_map_->getInflateOccupancyNoLock(nbr_pos) == 1) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (edt_env_->sdf_map_->getInflateOccupancyNoLock(ckpt) == 1) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (eChiuso(nbr_idx)) continue;

          NodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          NodePtr esistente = trovaAperto(nbr_idx);
          if (!esistente) {
            neighbor = nodoDelPool(use_node_num_);
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              RCLCPP_WARN(rclcpp::get_logger("astar"), "[AStar] Run out of node pool. Duration: %f",
                  std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < esistente->g_score) {
            neighbor = esistente;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          mettiAperto(nbr_idx, neighbor);
        }
  }
  return NO_PATH;
}

int Astar::searchUnknownOnlyBBox(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt,
    const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max) {
  NodePtr cur_node = nodoDelPool(0);
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  mettiAperto(cur_node->index, cur_node);
  use_node_num_ += 1;

  const auto t1 = std::chrono::steady_clock::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool reach_end = cur_node->index(0) == end_index(0) && cur_node->index(1) == end_index(1) &&
                     cur_node->index(2) == end_index(2);

    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    if (abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 &&
        abs(cur_node->index(2) - end_index(2)) <= 1) {
      bool safe = true;
      Vector3d dir = end_pt - cur_node->position;
      double len = dir.norm();
      dir.normalize();
      for (double l = 0.1; l < len; l += 0.1) {
        Vector3d ckpt = cur_node->position + l * dir;
        if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1 ||
            edt_env_->sdf_map_->getOccupancy(ckpt) != SDFMap::UNKNOWN) {
          safe = false;
          break;
        }
      }
      if (safe) {
        backtrack(cur_node, end_pt);
        return REACH_END;
      }
    }

    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count() > max_search_time_) {
      return NO_PATH;
    }

    open_set_.pop();
    chiudi(cur_node->index);
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;
    // Un solo lock della mappa gonfiata per nodo espanso (prima ~130, uno per lettura).
    std::lock_guard<std::mutex> lk_infl(edt_env_->sdf_map_->inflateMutex());

    for (double dx = -resolution_ + 1e-3; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_ + 1e-3; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_ + 1e-3; dz <= resolution_ + 1e-3; dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < 1e-3) continue;
          nbr_pos = cur_pos + step;

          auto isInBox = [&](const Eigen::Vector3d& pos) {
            return pos(0) >= bbox_min(0) && pos(0) <= bbox_max(0) && pos(1) >= bbox_min(1) &&
                   pos(1) <= bbox_max(1);
          };
          if (!isInBox(nbr_pos) || !edt_env_->sdf_map_->isInBox(nbr_pos)) {
            continue;
          }

          if (edt_env_->sdf_map_->getInflateOccupancyNoLock(nbr_pos) == 1 ||
              edt_env_->sdf_map_->getOccupancy(nbr_pos) != SDFMap::UNKNOWN) {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (edt_env_->sdf_map_->getInflateOccupancyNoLock(ckpt) == 1 ||
                edt_env_->sdf_map_->getOccupancy(ckpt) != SDFMap::UNKNOWN) {
              safe = false;
              break;
            }
          }
          if (!safe) {
            continue;
          }

          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (eChiuso(nbr_idx)) continue;

          NodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          NodePtr esistente = trovaAperto(nbr_idx);
          if (!esistente) {
            neighbor = nodoDelPool(use_node_num_);
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              RCLCPP_WARN(rclcpp::get_logger("astar"), "[AStar] Run out of node pool. Duration: %f",
                  std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count());
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < esistente->g_score) {
            neighbor = esistente;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          mettiAperto(nbr_idx, neighbor);
        }
  }

  return NO_PATH;
}

void Astar::reset() {
  open_set_map_.clear();
  close_set_map_.clear();
  if (++generazione_ == 0) {  // giro del contatore: si azzera tutto, una volta ogni 4 miliardi
    std::fill(gen_cella_.begin(), gen_cella_.end(), 0u);
    generazione_ = 1;
  }
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);
  for (int i = 0; i < use_node_num_; i++) {
    path_node_pool_[i]->parent = NULL;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
}

double Astar::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;
  for (size_t i = 0; i < path.size() - 1; ++i) length += (path[i + 1] - path[i]).norm();
  return length;
}

void Astar::backtrack(const NodePtr& end_node, const Eigen::Vector3d& end) {
  path_nodes_.push_back(end);
  path_nodes_.push_back(end_node->position);
  NodePtr cur_node = end_node;
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node->position);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath() {
  return path_nodes_;
}

double Astar::getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  double h;
  double diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

void Astar::posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

}  // namespace c2_expl
