#include <exploration_manager/copertura_visita.h>
#include <plan_env/sdf_map.h>

#include <algorithm>
#include <cmath>
#include <deque>

namespace c2_expl {

CoperturaVisita::CoperturaVisita(std::shared_ptr<SDFMap> map, const Parametri& p)
  : map_(map), p_(p) {
  Eigen::Vector3d bmin, bmax;
  map_->getBox(bmin, bmax);
  res_ = map_->getResolution();
  origine_ = bmin;
  nx_ = std::max(1, static_cast<int>(std::ceil((bmax.x() - bmin.x()) / res_)));
  ny_ = std::max(1, static_cast<int>(std::ceil((bmax.y() - bmin.y()) / res_)));
  visitata_.assign(static_cast<size_t>(nx_) * ny_, 0);
}

// Cella in cui il drone puo' stare: libera e fuori dalla fascia di gonfiaggio
// del pianificatore, alla quota di volo.
bool CoperturaVisita::percorribile(int i, int j) const {
  const Eigen::Vector3d p(origine_.x() + (i + 0.5) * res_, origine_.y() + (j + 0.5) * res_, p_.quota);
  if (!map_->isInMap(p)) return false;
  return map_->getOccupancy(p) == SDFMap::FREE && map_->getInflateOccupancy(p) == 0;
}

bool CoperturaVisita::vistaLibera(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
  const Eigen::Vector3d d = b - a;
  const double len = d.norm();
  for (double s = res_; s < len; s += res_) {
    const Eigen::Vector3d q = a + d * (s / len);
    if (!map_->isInMap(q) || map_->getOccupancy(q) == SDFMap::OCCUPIED) return false;
  }
  return true;
}

void CoperturaVisita::aggiorna(const Eigen::Vector3d& pos) {
  const Eigen::Vector3d a(pos.x(), pos.y(), p_.quota);
  const int ci = static_cast<int>(std::floor((pos.x() - origine_.x()) / res_));
  const int cj = static_cast<int>(std::floor((pos.y() - origine_.y()) / res_));
  const int r = static_cast<int>(p_.raggio / res_);
  for (int di = -r; di <= r; ++di)
    for (int dj = -r; dj <= r; ++dj) {
      if (di * di + dj * dj > r * r) continue;
      const int i = ci + di, j = cj + dj;
      if (i < 0 || j < 0 || i >= nx_ || j >= ny_) continue;
      uint8_t& v = visitata_[static_cast<size_t>(i) * ny_ + j];
      if (v) continue;
      const Eigen::Vector3d b(origine_.x() + (i + 0.5) * res_, origine_.y() + (j + 0.5) * res_, p_.quota);
      if (vistaLibera(a, b)) v = 1;
    }
}

bool CoperturaVisita::visitata(const Eigen::Vector3d& p) const {
  const int i = static_cast<int>(std::floor((p.x() - origine_.x()) / res_));
  const int j = static_cast<int>(std::floor((p.y() - origine_.y()) / res_));
  if (i < 0 || j < 0 || i >= nx_ || j >= ny_) return false;
  return visitata_[static_cast<size_t>(i) * ny_ + j] != 0;
}

int CoperturaVisita::nonVisitateEntro(const Eigen::Vector3d& p, double r) const {
  const int ci = static_cast<int>(std::floor((p.x() - origine_.x()) / res_));
  const int cj = static_cast<int>(std::floor((p.y() - origine_.y()) / res_));
  const int n = static_cast<int>(r / res_);
  int conta = 0;
  for (int di = -n; di <= n; ++di)
    for (int dj = -n; dj <= n; ++dj) {
      if (di * di + dj * dj > n * n) continue;
      const int i = ci + di, j = cj + dj;
      if (i < 0 || j < 0 || i >= nx_ || j >= ny_) continue;
      if (!visitata_[static_cast<size_t>(i) * ny_ + j] && percorribile(i, j)) ++conta;
    }
  return conta;
}

void CoperturaVisita::zone(std::vector<ZonaDaVisitare>& out, double& frazione) {
  zone(out, frazione, nullptr);
}

void CoperturaVisita::zone(std::vector<ZonaDaVisitare>& out, double& frazione,
    const std::function<int(const Eigen::Vector3d&)>& proprietario) {
  out.clear();
  std::vector<uint8_t> perc(visitata_.size(), 0);
  int totale = 0, visitate = 0;
  for (int i = 0; i < nx_; ++i)
    for (int j = 0; j < ny_; ++j) {
      if (!percorribile(i, j)) continue;
      const size_t k = static_cast<size_t>(i) * ny_ + j;
      ++totale;
      if (visitata_[k]) ++visitate;
      else perc[k] = 1;
    }
  frazione = totale > 0 ? static_cast<double>(visitate) / totale : 1.0;

  // Proprietario di ogni cella (solo con la spartizione a strisce): le zone non lo superano.
  std::vector<int> padrone;
  if (proprietario) {
    padrone.assign(perc.size(), -1);
    for (int i = 0; i < nx_; ++i)
      for (int j = 0; j < ny_; ++j) {
        const size_t k = static_cast<size_t>(i) * ny_ + j;
        if (perc[k])
          padrone[k] = proprietario(Eigen::Vector3d(origine_.x() + (i + 0.5) * res_,
              origine_.y() + (j + 0.5) * res_, p_.quota));
      }
  }
  // Raggruppamento dello spazio percorribile non visitato (8-connesso).
  std::vector<uint8_t> vista(perc.size(), 0);
  for (int i0 = 0; i0 < nx_; ++i0)
    for (int j0 = 0; j0 < ny_; ++j0) {
      const size_t k0 = static_cast<size_t>(i0) * ny_ + j0;
      if (!perc[k0] || vista[k0]) continue;
      std::deque<std::pair<int, int>> coda{ { i0, j0 } };
      vista[k0] = 1;
      std::vector<std::pair<int, int>> gruppo;
      while (!coda.empty()) {
        const auto c = coda.front();
        coda.pop_front();
        gruppo.push_back(c);
        for (int di = -1; di <= 1; ++di)
          for (int dj = -1; dj <= 1; ++dj) {
            const int i = c.first + di, j = c.second + dj;
            if (i < 0 || j < 0 || i >= nx_ || j >= ny_) continue;
            const size_t k = static_cast<size_t>(i) * ny_ + j;
            if (perc[k] && !vista[k] &&
                (padrone.empty() ||
                 padrone[k] == padrone[static_cast<size_t>(c.first) * ny_ + c.second])) {
              vista[k] = 1;
              coda.push_back({ i, j });
            }
          }
      }
      if (static_cast<int>(gruppo.size()) < p_.min_celle) continue;
      ZonaDaVisitare z;
      z.celle = static_cast<int>(gruppo.size());
      Eigen::Vector3d somma = Eigen::Vector3d::Zero();
      std::vector<Eigen::Vector3d> punti;
      punti.reserve(gruppo.size());
      for (const auto& c : gruppo) {
        punti.emplace_back(origine_.x() + (c.first + 0.5) * res_,
            origine_.y() + (c.second + 0.5) * res_, p_.quota);
        somma += punti.back();
      }
      z.centro = somma / static_cast<double>(punti.size());
      // Mete candidate: le celle della zona dalla piu' vicina al centro (in una
      // zona a L il centro puo' cadere fuori), al massimo 40 a passo regolare.
      std::sort(punti.begin(), punti.end(), [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return (a - z.centro).squaredNorm() < (b - z.centro).squaredNorm();
      });
      const size_t passo = std::max<size_t>(1, punti.size() / 40);
      for (size_t k = 0; k < punti.size() && z.mete.size() < 40; k += passo) z.mete.push_back(punti[k]);
      out.push_back(z);
    }
}

}  // namespace c2_expl
