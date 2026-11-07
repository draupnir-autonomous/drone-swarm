#ifndef _COPERTURA_CAMERA_H_
#define _COPERTURA_CAMERA_H_

#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace c2_expl {
class SDFMap;

// COPERTURA DELLA TELECAMERA (ottobre 2026).
//
// L'esplorazione si ferma quando il LIDAR ha visto tutto, ma il lidar vede a
// 360 gradi mentre la telecamera frontale ne vede 90: a esplorazione finita il
// 10% della mappa 3D (17% delle pareti) non era mai stato inquadrato e restava
// grigio (run del 6 ottobre 2026, office_small) - l'interno della stanza a
// nord-est, la parete est della stanza centrale, alcune facce delle pareti
// diagonali. Questa classe tiene il conto delle superfici della mappa che la
// telecamera ha davvero inquadrato (raggi tracciati dal suo centro ottico nel
// suo campo visivo, fermandosi alla prima cella occupata) e raggruppa quelle
// che mancano, perche' l'FSM ci porti il drone a guardarle prima di rientrare.
struct ZonaNonInquadrata {
  Eigen::Vector3d centro = Eigen::Vector3d::Zero();
  Eigen::Vector3d normale = Eigen::Vector3d::UnitZ();  // verso lo spazio libero
  int celle = 0;
  std::vector<Eigen::Vector3d> campioni;  // un sottoinsieme delle celle, per i controlli di vista
};

class CoperturaCamera {
public:
  struct Parametri {
    double hfov = 1.5708;          // campo orizzontale (rad), model.sdf
    double larghezza = 640.0;      // pixel, per il rapporto d'aspetto
    double altezza = 480.0;
    double pitch = 0.15;           // inclinazione verso il basso (rad)
    Eigen::Vector3d offset = Eigen::Vector3d(0.10, 0.0, 0.12);  // dal corpo
    double portata = 6.0;          // oltre, il dettaglio della foto e' poco utile
    int raggi_h = 80;
    int raggi_v = 60;
    int min_celle = 15;            // zone piu' piccole si ignorano (0.15 m2)
    double z_min = -0.15;          // fascia di quote considerata (pavimento compreso)
    double z_max = 2.00;           // il soffitto (2.27 m) resta fuori: la telecamera,
                                   // inclinata in basso, non lo inquadra bene
  };

  CoperturaCamera(std::shared_ptr<SDFMap> map, const Parametri& p);

  // Segna le superfici inquadrate dalla telecamera con il corpo in (pos, q).
  void aggiorna(const Eigen::Vector3d& pos, const Eigen::Quaterniond& q);

  // Zone di superficie non ancora inquadrate, e quota di superficie inquadrata.
  void zone(std::vector<ZonaNonInquadrata>& out, double& frazione_inquadrata);

  // Il punto p si vedrebbe dalla telecamera di un drone in pos (in piano) con
  // imbardata yaw? Campo visivo, portata e linea di vista libera.
  bool visibileDa(const Eigen::Vector3d& pos_corpo, double yaw, const Eigen::Vector3d& p) const;

  // Quante superfici NON ancora inquadrate la telecamera vedrebbe da pos (in
  // piano) con imbardata yaw: raggi radi (12x6) nel campo visivo, fino alla
  // prima cella occupata, entro 5 m. Serve a scegliere l'imbardata durante
  // l'esplorazione (vedi FastPlannerManager::punteggio_imbardata_).
  int contaNonViste(const Eigen::Vector3d& pos_corpo, double yaw) const;

  const Parametri& parametri() const { return p_; }

private:
  bool lineaLibera(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
  Eigen::Matrix3d rotCamera(const Eigen::Matrix3d& r_corpo) const;

  std::shared_ptr<SDFMap> map_;
  Parametri p_;
  std::vector<uint8_t> vista_;
  double fx_, fy_, cx_, cy_, vfov_;
};

}  // namespace c2_expl

#endif
