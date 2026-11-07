#ifndef _CACHE_PERCORSI_H_
#define _CACHE_PERCORSI_H_

#include <Eigen/Eigen>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace c2_expl {

// MEMORIA DEI PERCORSI per il giro delle zone (ottobre 2026).
//
// A ogni pianificazione il grafo di connettivita' e la matrice dei costi
// rifacevano da zero centinaia di ricerche A* fra gli STESSI centri delle
// celle: su cubicle_office 35-255 ricerche per costruzione del grafo, fino a
// 1.95 s, e la matrice fino a 1.7 s. La pianificazione durava 1-3 s contro gli
// 0.3 s di office_small, il server di traiettoria fermava il drone ad
// aspettarla e ne nasceva il dondolio avanti-indietro.
//
// Qui si ricorda l'esito di ogni ricerca (coppia di estremi + tipo di
// ricerca):
//  * percorso trovato: si riusa finche' e' ancora percorribile (controllo
//    cella per cella lungo il percorso, molto piu' rapido di una ricerca) e
//    al massimo per durata_trovato_ secondi, cosi' ogni tanto si ricerca un
//    percorso migliore nello spazio appena scoperto;
//  * ricerca fallita: si ricorda per durata_fallito_ secondi. E' la parte che
//    costa di piu', perche' una ricerca che fallisce consuma tutto il tempo
//    massimo (astar.max_search_time) prima di arrendersi.
// I costi servono solo a ORDINARE le zone da visitare; il volo vero si
// pianifica sempre con una ricerca nuova verso il punto di vista.
//
// SCADENZA IN GIRI, NON SOLO IN SECONDI (Jetson Orin Nano, ottobre 2026). Le
// durate in secondi erano tarate sul PC, che pianifica circa una volta al
// secondo: 8 s ~ 7 pianificazioni, 4 s ~ 3. Sul Jetson una costruzione del grafo
// arrivava a 7 s, quindi alla pianificazione dopo le voci erano gia' tutte
// scadute ("memoria 0/556"), ogni ricerca si rifaceva, la pianificazione
// durava ancora di piu': un circolo vizioso (grafo in media 1.57 s contro
// 0.30 s sul PC, run V07 cubicle_office). Ora una voce scade solo quando ha
// superato SIA la durata in secondi SIA giri_* costruzioni (un giro =
// una chiamata a statistiche(), cioe' una costruzione del grafo o della
// matrice): sul PC non cambia nulla, sul Jetson una voce vive lo stesso numero
// di pianificazioni che sul PC.
class CachePercorsi {
public:
  struct Voce {
    double costo = 1000.0;
    double extra_a = 0.0;  // costi accessori (cambio di velocita'/quota) se servono
    double extra_b = 0.0;
    std::vector<Eigen::Vector3d> path;
    double t = 0.0;
    uint64_t giro = 0;
  };

  CachePercorsi(double durata_trovato = 8.0, double durata_fallito = 4.0, double passo_chiave = 0.25,
      uint64_t giri_trovato = 7, uint64_t giri_fallito = 3)
    : durata_trovato_(durata_trovato), durata_fallito_(durata_fallito), passo_chiave_(passo_chiave),
      giri_trovato_(giri_trovato), giri_fallito_(giri_fallito) {}

  // Vero se c'e' una voce utilizzabile per (a, b, tipo); 'ancora_valido' verifica
  // sulla mappa attuale che il percorso memorizzato sia ancora percorribile.
  bool cerca(const Eigen::Vector3d& a, const Eigen::Vector3d& b, int tipo,
      const std::function<bool(const std::vector<Eigen::Vector3d>&)>& ancora_valido, Voce& out) {
    auto it = mappa_.find(chiave(a, b, tipo));
    if (it == mappa_.end()) {
      ++mancati_;
      return false;
    }
    const bool fallita = it->second.costo >= 999.0;
    const double eta = adesso() - it->second.t;
    const uint64_t giri = giro_ - it->second.giro;
    const bool scaduta = fallita ? (eta > durata_fallito_ && giri > giri_fallito_)
                                 : (eta > durata_trovato_ && giri > giri_trovato_);
    if (scaduta || (!fallita && !ancora_valido(it->second.path))) {
      mappa_.erase(it);
      ++mancati_;
      return false;
    }
    out = it->second;
    ++colpiti_;
    return true;
  }

  void salva(const Eigen::Vector3d& a, const Eigen::Vector3d& b, int tipo, Voce v) {
    if (mappa_.size() > 50000) mappa_.clear();  // tetto alla memoria, di fatto mai raggiunto
    v.t = adesso();
    v.giro = giro_;
    mappa_[chiave(a, b, tipo)] = std::move(v);
  }

  // Contatori dall'ultima chiamata, per la diagnostica nei log. Si chiama una
  // volta per costruzione: segna anche la fine di un giro (vedi sopra).
  void statistiche(int& colpiti, int& mancati) {
    colpiti = colpiti_;
    mancati = mancati_;
    colpiti_ = mancati_ = 0;
    ++giro_;
  }

private:
  using Chiave = std::array<int32_t, 7>;
  struct HashChiave {
    size_t operator()(const Chiave& k) const {
      size_t h = 1469598103934665603ULL;
      for (int32_t v : k) h = (h ^ static_cast<uint32_t>(v)) * 1099511628211ULL;
      return h;
    }
  };
  // Estremi quantizzati a passo_chiave_ (25 cm). Con 10 cm la memoria serviva
  // poco proprio dove serviva di piu': entrando in una cella l'esplorazione
  // sposta di pochi centimetri i suoi centri, e una costruzione del grafo da
  // 370 archi ne ha trovati in memoria 42 (3.2 s). Per ORDINARE le zone un
  // errore di qualche centimetro sul costo e' irrilevante, e il percorso
  // riusato viene comunque verificato sulla mappa attuale.
  Chiave chiave(const Eigen::Vector3d& a, const Eigen::Vector3d& b, int tipo) const {
    const double inv = 1.0 / passo_chiave_;
    auto q = [inv](double x) { return static_cast<int32_t>(std::lround(x * inv)); };
    return { q(a.x()), q(a.y()), q(a.z()), q(b.x()), q(b.y()), q(b.z()), tipo };
  }
  static double adesso() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  std::unordered_map<Chiave, Voce, HashChiave> mappa_;
  double durata_trovato_, durata_fallito_, passo_chiave_;
  uint64_t giri_trovato_, giri_fallito_;
  uint64_t giro_ = 0;
  int colpiti_ = 0, mancati_ = 0;
};

}  // namespace c2_expl

#endif
