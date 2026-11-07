#include <exploration_manager/copertura_camera.h>
#include <plan_env/sdf_map.h>

#include <algorithm>
#include <cmath>
#include <deque>

namespace c2_expl {

CoperturaCamera::CoperturaCamera(std::shared_ptr<SDFMap> map, const Parametri& p)
  : map_(map), p_(p) {
  vista_.assign(static_cast<size_t>(map_->getVoxelNum()), 0);
  cx_ = p_.larghezza / 2.0;
  cy_ = p_.altezza / 2.0;
  fx_ = cx_ / std::tan(p_.hfov / 2.0);
  fy_ = fx_;
  vfov_ = 2.0 * std::atan(cy_ / fy_);
}

// Terna della telecamera: corpo ruotato del pitch attorno a y (x avanti, z in
// alto; pitch positivo = muso in giu', come la posa in model.sdf).
Eigen::Matrix3d CoperturaCamera::rotCamera(const Eigen::Matrix3d& r_corpo) const {
  return r_corpo * Eigen::AngleAxisd(p_.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

void CoperturaCamera::aggiorna(const Eigen::Vector3d& pos, const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d rb = q.normalized().toRotationMatrix();
  const Eigen::Matrix3d rc = rotCamera(rb);
  const Eigen::Vector3d origine = pos + rb * p_.offset;
  const double passo = 0.5 * map_->getResolution();
  Eigen::Vector3i id;
  for (int iu = 0; iu < p_.raggi_h; ++iu) {
    const double u = (iu + 0.5) * p_.larghezza / p_.raggi_h;
    for (int iv = 0; iv < p_.raggi_v; ++iv) {
      const double v = (iv + 0.5) * p_.altezza / p_.raggi_v;
      const Eigen::Vector3d d =
          rc * Eigen::Vector3d(1.0, -(u - cx_) / fx_, -(v - cy_) / fy_).normalized();
      for (double s = 0.3; s <= p_.portata; s += passo) {
        const Eigen::Vector3d pt = origine + d * s;
        if (!map_->isInMap(pt)) break;
        map_->posToIndex(pt, id);
        if (map_->getOccupancy(id) != SDFMap::OCCUPIED) continue;
        // Cella colpita e le occupate attorno (raggio di una cella): a qualche
        // metro due raggi vicini distano piu' di una cella, e senza questo la
        // superficie resterebbe "a buchi".
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz) {
              const Eigen::Vector3i n = id + Eigen::Vector3i(dx, dy, dz);
              if (!map_->isInMap(n)) continue;
              if (map_->getOccupancy(n) == SDFMap::OCCUPIED) vista_[map_->toAddress(n)] = 1;
            }
        break;
      }
    }
  }
}

bool CoperturaCamera::lineaLibera(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
  const Eigen::Vector3d d = b - a;
  const double len = d.norm();
  const double passo = 0.5 * map_->getResolution();
  Eigen::Vector3i id, id_b;
  map_->posToIndex(b, id_b);
  // L'ultimo tratto (una cella e mezza) e' la superficie stessa: non conta.
  for (double s = 0.3; s < len - 1.5 * map_->getResolution(); s += passo) {
    const Eigen::Vector3d pt = a + d * (s / len);
    if (!map_->isInMap(pt)) return false;
    map_->posToIndex(pt, id);
    if ((id - id_b).cwiseAbs().maxCoeff() <= 1) break;
    if (map_->getOccupancy(id) == SDFMap::OCCUPIED) return false;
  }
  return true;
}

bool CoperturaCamera::visibileDa(
    const Eigen::Vector3d& pos_corpo, double yaw, const Eigen::Vector3d& p) const {
  const Eigen::Matrix3d rb = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d rc = rotCamera(rb);
  const Eigen::Vector3d origine = pos_corpo + rb * p_.offset;
  const Eigen::Vector3d d = rc.transpose() * (p - origine);
  const double dist = d.norm();
  if (dist > p_.portata || dist < 0.4 || d.x() <= 0.0) return false;
  // Margine del 10% sui bordi dell'immagine.
  if (std::abs(std::atan2(d.y(), d.x())) > 0.45 * p_.hfov) return false;
  if (std::abs(std::atan2(d.z(), d.x())) > 0.45 * vfov_) return false;
  return lineaLibera(origine, p);
}

int CoperturaCamera::contaNonViste(const Eigen::Vector3d& pos_corpo, double yaw) const {
  const Eigen::Matrix3d rb = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d rc = rotCamera(rb);
  const Eigen::Vector3d origine = pos_corpo + rb * p_.offset;
  const double passo = map_->getResolution();
  const double portata = std::min(p_.portata, 5.0);
  const int nh = 12, nv = 6;
  int n = 0;
  Eigen::Vector3i id;
  for (int iu = 0; iu < nh; ++iu) {
    const double u = (iu + 0.5) * p_.larghezza / nh;
    for (int iv = 0; iv < nv; ++iv) {
      const double v = (iv + 0.5) * p_.altezza / nv;
      const Eigen::Vector3d d =
          rc * Eigen::Vector3d(1.0, -(u - cx_) / fx_, -(v - cy_) / fy_).normalized();
      for (double s = 0.3; s <= portata; s += passo) {
        const Eigen::Vector3d pt = origine + d * s;
        if (!map_->isInMap(pt)) break;
        map_->posToIndex(pt, id);
        if (map_->getOccupancy(id) != SDFMap::OCCUPIED) continue;
        if (pt.z() >= p_.z_min && pt.z() <= p_.z_max && !vista_[map_->toAddress(id)]) ++n;
        break;
      }
    }
  }
  return n;
}

void CoperturaCamera::zone(std::vector<ZonaNonInquadrata>& out, double& frazione) {
  out.clear();
  Eigen::Vector3d bmin, bmax;
  map_->getBox(bmin, bmax);
  bmin.z() = std::max(bmin.z(), p_.z_min);
  bmax.z() = std::min(bmax.z(), p_.z_max);
  Eigen::Vector3i imin, imax;
  map_->posToIndex(bmin, imin);
  map_->posToIndex(bmax, imax);
  static const int vic6[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 },
    { 0, 0, -1 } };

  // Celle di superficie: occupate con almeno una vicina libera (il lato che si
  // affaccia sullo spazio percorribile, l'unico che una telecamera puo' vedere).
  std::vector<int> mancanti;
  std::vector<uint8_t> candidata(vista_.size(), 0);
  int superficie = 0, viste = 0;
  Eigen::Vector3i id;
  for (id.x() = imin.x(); id.x() <= imax.x(); ++id.x())
    for (id.y() = imin.y(); id.y() <= imax.y(); ++id.y())
      for (id.z() = imin.z(); id.z() <= imax.z(); ++id.z()) {
        if (map_->getOccupancy(id) != SDFMap::OCCUPIED) continue;
        bool affaccio = false;
        for (const auto& v : vic6) {
          const Eigen::Vector3i n = id + Eigen::Vector3i(v[0], v[1], v[2]);
          if (map_->isInMap(n) && map_->getOccupancy(n) == SDFMap::FREE) {
            affaccio = true;
            break;
          }
        }
        if (!affaccio) continue;
        ++superficie;
        const int adr = map_->toAddress(id);
        if (vista_[adr]) {
          ++viste;
        } else {
          candidata[adr] = 1;
          mancanti.push_back(adr);
        }
      }
  frazione = superficie > 0 ? static_cast<double>(viste) / superficie : 1.0;

  // Raggruppamento delle celle mancanti (26-connesse).
  std::vector<uint8_t> visitata(vista_.size(), 0);
  for (int seme : mancanti) {
    if (visitata[seme]) continue;
    std::deque<int> coda{ seme };
    visitata[seme] = 1;
    std::vector<int> gruppo;
    while (!coda.empty()) {
      const int adr = coda.front();
      coda.pop_front();
      gruppo.push_back(adr);
      Eigen::Vector3i c;
      map_->addressToIndex(adr, c);
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
          for (int dz = -1; dz <= 1; ++dz) {
            const Eigen::Vector3i n = c + Eigen::Vector3i(dx, dy, dz);
            if (!map_->isInMap(n)) continue;
            const int a = map_->toAddress(n);
            if (candidata[a] && !visitata[a]) {
              visitata[a] = 1;
              coda.push_back(a);
            }
          }
    }
    if (static_cast<int>(gruppo.size()) < p_.min_celle) continue;
    ZonaNonInquadrata z;
    z.celle = static_cast<int>(gruppo.size());
    Eigen::Vector3d somma = Eigen::Vector3d::Zero(), norm = Eigen::Vector3d::Zero();
    const int passo = std::max(1, static_cast<int>(gruppo.size()) / 40);
    for (size_t k = 0; k < gruppo.size(); ++k) {
      Eigen::Vector3i c;
      map_->addressToIndex(gruppo[k], c);
      Eigen::Vector3d pc;
      map_->indexToPos(c, pc);
      somma += pc;
      for (const auto& v : vic6) {
        const Eigen::Vector3i n = c + Eigen::Vector3i(v[0], v[1], v[2]);
        if (map_->isInMap(n) && map_->getOccupancy(n) == SDFMap::FREE)
          norm += Eigen::Vector3d(v[0], v[1], v[2]);
      }
      if (static_cast<int>(k) % passo == 0) z.campioni.push_back(pc);
    }
    z.centro = somma / static_cast<double>(gruppo.size());
    if (norm.norm() > 1e-6) z.normale = norm.normalized();
    out.push_back(z);
  }
}

}  // namespace c2_expl
