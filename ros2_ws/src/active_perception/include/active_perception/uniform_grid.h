#ifndef _UNIFORM_GRID_H_
#define _UNIFORM_GRID_H_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <active_perception/connectivity_graph.h>
#include <active_perception/cache_percorsi.h>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::unordered_set;
using std::vector;

class RayCaster;

namespace c2_expl {

class EDTEnvironment;
class Astar;
class HGrid;

enum GridState { EXPLORED, EXPLORING, EXPLORABLE, UNREACHABLE };

// struct GridInfo {};

class GridInfo {
public:
  GridInfo() {
  }
  ~GridInfo() {
  }

  int unknown_num_;
  int free_num_;
  // Le frontiere contenute si contano da contained_frontier_ids_. Qui c'era un
  // frontier_num_ che nessuno scriveva mai e che il costruttore non azzerava:
  // sembrava un dato e restituiva memoria indeterminata.
  Eigen::Vector3d center_;
  vector<Eigen::Vector3d> centers_free_;
  vector<Eigen::Vector3d> centers_unknown_;
  vector<Eigen::Vector3d> centers_free_active_;
  vector<Eigen::Vector3d> centers_unknown_active_;
  vector<int> centers_free_active_idx_;
  vector<int> centers_unknown_active_idx_;
  unordered_map<int, int> frontier_cell_nums_;
  unordered_map<int, int> contained_frontier_ids_;
  bool is_updated_;
  bool need_divide_, active_;

  bool is_prev_relevant_;
  bool is_cur_relevant_;
  bool is_reachable_;
  bool was_in_tour_;  // Track if this grid has ever been added to tour

  GridState state_;

  // Vertices and their box in xy plane, in current drone's frame
  Eigen::Vector3d vmin_, vmax_;
  vector<Eigen::Vector3d> vertices_;

  // Normals of separating lines in xy plane, associated with vertices_
  vector<Eigen::Vector3d> normals_;
};

class UniformGrid {

public:
  UniformGrid(const shared_ptr<EDTEnvironment>& edt, rclcpp::Node::SharedPtr node, const int& level);
  ~UniformGrid();

  void initGridData();
  void updateBaseCoor();
  void getCCLcenters(const int& grid_id, std::vector<Vector3d>& centers_free,
      std::vector<Vector3d>& centers_unknown, std::vector<int>& free_nums,
      std::vector<int>& unknown_nums);
  void updateGridData(const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids,
      vector<int>& parti_ids_all, const Eigen::Vector3d& pos);

  void inputFrontiers(const vector<Eigen::Vector3d>& avgs);
  void getUpdatedBoxMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2);
  Eigen::Vector3d rectifyCenter(const Eigen::Vector3d& center, double search_radius = 2.0);
  ConnectivityGraph::Ptr getConnectivityGraph() {
    return connectivity_graph_;
  }
  int getConnectivityNodeId(const int& cell_id, const int& center_id) const {
    return cell_id * kNodeStride + center_id;
  }

private:
  static bool sameFrontierIdSet(
      const std::unordered_map<int, int>& a, const std::unordered_map<int, int>& b);
  static bool boxesOverlap2D(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1,
      const Eigen::Vector3d& min2, const Eigen::Vector3d& max2);
  static const char* gridStateName(const GridState& state);
  bool shouldUpdateGrid(const GridInfo& grid, const Eigen::Vector3d& update_min,
      const Eigen::Vector3d& update_max, const vector<Eigen::Vector3d>& chunk_mins,
      const vector<Eigen::Vector3d>& chunk_maxs, bool& overlap_with_fov) const;
  void finalizeUpdatedGridState(const int& cell_id);
  void rebuildRelevantGridList();
  void updateAssignedGridIds(const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids);
  void updateGridInfo(const Eigen::Vector3i& id, const Eigen::Vector3d& pos);
  std::vector<std::unordered_set<int>> splitUnknownComponentByConvexity(
      const std::unordered_set<int>& voxel_addrs, const int step) const;
  Eigen::Vector3d projectToNearestUnknownVoxel(
      const std::unordered_set<int>& voxel_addrs, const Eigen::Vector3d& ref_center) const;
  void updateConnectivityGraph(
      const std::vector<int>& update_cell_ids, const std::vector<int>& old_center_counts);
  void getNearbyGridIds(const int& adr, std::vector<int>& neighbor_ids);
  int makeNodeId(const int& cell_id, const int& center_id) const {
    return cell_id * kNodeStride + center_id;
  }
  bool pathInsideBBox(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& bbox_min,
      const Eigen::Vector3d& bbox_max) const;
  // Lavoratore del grafo di connettivita': A* e raycaster propri, cosi' gli archi
  // si calcolano in parallelo (vedi updateConnectivityGraph).
  struct LavoratoreCG {
    unique_ptr<RayCaster> caster;
    unique_ptr<Astar> astar;
  };
  double computeFreeCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
      const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
      std::vector<Eigen::Vector3d>& path, LavoratoreCG& w);
  double computeUnknownCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
      const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
      std::vector<Eigen::Vector3d>& path, LavoratoreCG& w);
  double computeUnknownOnlyCostBBox(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
      const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
      std::vector<Eigen::Vector3d>& path, LavoratoreCG& w);

  int toAddress(const Eigen::Vector3i& id);
  void adrToIndex(const int& adr, Eigen::Vector3i& idx);
  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos);
  bool insideGrid(const Eigen::Vector3i& id);
  bool isRelevant(const GridInfo& grid);
  // Riepilogo dello stato delle celle: serve a capire PERCHE' l'allocazione
  // non assegna nulla, invece di dedurlo dal codice.
  std::string gridSummary();
  // Centro geometrico di una cella, per la ripartizione fra droni.
  bool getGridCenter(int grid_id, Eigen::Vector3d& center) const;
  // Celle attualmente rilevanti: l'insieme autorevole di cio' che va ancora
  // esplorato, indipendentemente da chi lo possiede.
  // Copia degli indirizzi di una componente CCL, presa SOTTO LUCCHETTO.
  // HGrid accedeva direttamente a ccl_voxels_addr_ (e' friend), scavalcando la
  // protezione: il lucchetto difendeva i metodi della griglia mentre HGrid
  // entrava dai dati. Da li' l'ultimo segfault, in addressToIndex su un buffer
  // svuotato da un altro thread mentre veniva letto. Copiare e lavorare sulla
  // copia tiene la sezione critica corta e rende impossibile la lettura di un
  // vettore in corso di modifica.
  // Lucchetto esportabile: HGrid e' classe amica e legge grid_data_,
  // ccl_* e le trasformate DIRETTAMENTE, in 25 punti. Proteggere i soli metodi
  // di UniformGrid non serviva a nulla contro quegli accessi. Chi traversa quei
  // dati prende questo lucchetto per la durata della traversata - che e' breve
  // e non contiene ricerche di percorso, quindi non ripropone lo stallo visto
  // serializzando l'interfaccia di HGrid.
  std::unique_lock<std::recursive_mutex> lockData() const {
    return std::unique_lock<std::recursive_mutex>(data_mutex_);
  }

  bool copyCCLVoxelAddrs(int grid_id, int component, vector<int>& out) const {
    std::lock_guard<std::recursive_mutex> lk(data_mutex_);
    out.clear();
    if (grid_id < 0 || grid_id >= static_cast<int>(ccl_voxels_addr_.size())) return false;
    if (component < 0 || component >= static_cast<int>(ccl_voxels_addr_[grid_id].size()))
      return false;
    const auto& src = ccl_voxels_addr_[grid_id][component];
    out.assign(src.begin(), src.end());
    return true;
  }

  void getRelevantIds(vector<int>& ids) const {
    std::lock_guard<std::recursive_mutex> lk(data_mutex_);
    ids = relevant_id_;
  }

  // TUTTE le celle, rilevanti o no.
  //
  // Serve come dominio della ripartizione fra droni. relevant_id_ dipende dalla
  // mappa LOCALE (raggiungibile e ancora esplorabile), quindi differisce da
  // drone a drone: partizionando su quello, tre droni partizionano tre insiemi
  // diversi e l'esclusivita' non puo' valere, per quanto simmetrico sia il
  // criterio - misurate 595 sovrapposizioni in 4 run, e un tentativo di
  // correggerle rendendo simmetrica l'isteresi le ha portate a 747.
  // La griglia intera e' invece statica e identica su ogni drone: come dominio
  // rende il calcolo riproducibile. Cio' che e' localmente rilevante torna a
  // filtrare il LAVORO, non la proprieta'.
  void getAllIds(vector<int>& ids) const {
    std::lock_guard<std::recursive_mutex> lk(data_mutex_);
    ids.resize(grid_data_.size());
    for (size_t i = 0; i < grid_data_.size(); ++i) ids[i] = static_cast<int>(i);
  }

  // Frontiere contenute in ogni cella, indicizzate per id.
  //
  // Serve a PESARE la ripartizione: bilanciare il numero di celle le conta
  // tutte, comprese quelle gia' esplorate, e a fine run le celle vive sono
  // poche - il bilanciatore vede i carichi pari mentre il lavoro vero e' tutto
  // su un drone, che resta l'ultimo a volare mentre gli altri aspettano.
  // A differenza di relevant_id_ questo conteggio viene da inputFrontiers(),
  // cioe' dall'insieme GLOBALE delle frontiere sulla mappa condivisa: resta
  // quindi concorde fra i droni e non rompe la riproducibilita' che rende
  // esclusiva la ripartizione.
  //
  // Si legge contained_frontier_ids_, non il campo frontier_num_ che gli sta
  // accanto: quello non viene scritto da nessuna parte nel progetto (verificato
  // su tutti i sorgenti) e, non essendo inizializzato dal costruttore, leggerlo
  // e' memoria indeterminata. Ci ho pesato sopra la ripartizione per una
  // campagna intera prima di accorgermene: i carichi risultavano tutti a zero e
  // il ribilanciamento diventava un'operazione nulla.
  void getFrontierCounts(vector<int>& counts) const {
    std::lock_guard<std::recursive_mutex> lk(data_mutex_);
    counts.resize(grid_data_.size());
    for (size_t i = 0; i < grid_data_.size(); ++i)
      counts[i] = static_cast<int>(grid_data_[i].contained_frontier_ids_.size());
  }

  shared_ptr<EDTEnvironment> edt_;
  unique_ptr<RayCaster> caster_;
  unique_ptr<Astar> path_finder_;
  // Memoria degli archi del grafo di connettivita' (vedi cache_percorsi.h):
  // tipo 0 = solo libero, 1 = ignoto ammesso, 2 = solo ignoto.
  CachePercorsi cache_archi_;
  bool percorsoAncoraValido(const std::vector<Eigen::Vector3d>& path, int tipo);
  vector<GridInfo> grid_data_;

  vector<int> relevant_id_;
  // Protegge grid_data_ e i buffer CCL. Ci si arriva da DUE percorsi - il timer
  // di pianificazione dell'FSM e il callback delle frontiere - e solo uno teneva
  // un lucchetto esterno: due thread dentro getCCLcenters sullo stesso grid_id
  // corrompevano l'heap, con l'abort che arrivava molto dopo in una malloc
  // qualsiasi (firma: updateGridData -> updateGridInfo -> getCCLcenters ->
  // clear -> malloc_printerr). Le guardie sugli indici, aggiunte prima, non sono
  // mai scattate: l'indice era valido, il problema era la concorrenza.
  // Ricorsivo perche' i metodi pubblici si richiamano fra loro.
  mutable std::recursive_mutex data_mutex_;

  // path_finder_ e' un'ALTRA istanza di A* condivisa, come ViewNode::astar_:
  // stato mutabile (liste aperte/chiuse, pool di nodi) usato da piu' thread.
  // computeUnknownCostBBox e computeCostUnknown la usano da percorsi che non
  // passano tutti per i metodi protetti da data_mutex_.
  // Lucchetto FOGLIA: dentro non si prende nient'altro, cosi' non puo' creare
  // inversioni di ordine - errore gia' commesso due volte oggi, con due stalli.
  mutable std::mutex path_finder_mutex_;
  unordered_map<int, int> relevant_map_;
  bool initialized_;

  Eigen::Vector3d resolution_;
  Eigen::Vector3d min_, max_;
  Eigen::Vector3i grid_num_;
  int level_;
  int ccl_step_;

  int z_samples_;              // Z samples for free/unknown estimation
  double w_unknown_;
  double grid_size_;
  double cg_astar_resolution_;
  double convex_split_hull_area_ratio_;

  // Swarm tf
  Eigen::Matrix3d rot_sw_;
  Eigen::Vector3d trans_sw_;
  bool use_swarm_tf_;

  static constexpr int kNodeStride = 10000;

  ConnectivityGraph::Ptr connectivity_graph_;
  double connectivity_graph_t = 0.0;
  // Contatori di diagnosi del grafo di connettivita' (vedi updateConnectivityGraph).
  int cg_diag_edges_ = 0, cg_diag_portal_ = 0, cg_diag_free_ = 0;
  std::atomic<int> cg_diag_astar_{0};
  std::vector<LavoratoreCG> lavoratori_cg_;
  // Protegge cache_archi_ quando gli archi si calcolano in parallelo.
  std::mutex cache_archi_mutex_;

  vector<vector<unordered_set<int>>> ccl_voxels_addr_;
  vector<vector<Vector3d>> ccl_voxels_color_;
  vector<vector<pair<int, int>>> ccl_free_unknown_states_and_centers_idx_;

  // Updated box for visualization
  Eigen::Vector3d updated_box_min_, updated_box_max_;
  vector<Eigen::Vector3d> chunk_box_mins_, chunk_box_maxs_;

  double resolution;

  friend HGrid;
};

}  // namespace c2_expl
#endif
