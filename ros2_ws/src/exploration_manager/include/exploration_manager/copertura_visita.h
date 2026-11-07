#ifndef _COPERTURA_VISITA_H_
#define _COPERTURA_VISITA_H_

#include <Eigen/Eigen>
#include <functional>
#include <memory>
#include <vector>

namespace c2_expl {
class SDFMap;

// COPERTURA DI VISITA (ottobre 2026).
//
// L'esplorazione a frontiere finisce quando il lidar ha visto tutto, e il lidar
// vede a 360 gradi e fino a decine di metri: un corridoio cieco o una stanza
// vista dalla porta risultano "esplorati" senza che il drone ci entri mai. Su
// cubicle_office il drone passava entro 2 m da circa l'80% dello spazio
// percorribile: il corridoio a nord-ovest, la stanza attorno alla T, gli spazi
// fra i divisori non venivano mai visitati (V31). Questa classe tiene la pianta
// della VISITA alla quota di volo - celle dove il drone e' passato entro
// 'raggio', con la vista libera - e raggruppa lo spazio percorribile che manca,
// perche' l'FSM ci porti il drone prima di rientrare.
struct ZonaDaVisitare {
  Eigen::Vector3d centro = Eigen::Vector3d::Zero();
  int celle = 0;
  std::vector<Eigen::Vector3d> mete;  // celle della zona, dalla piu' vicina al centro
};

class CoperturaVisita {
public:
  struct Parametri {
    double raggio = 2.0;   // entro questa distanza (vista libera) una cella e' visitata
    double quota = 1.2;    // quota della pianta (fascia di volo)
    int min_celle = 100;   // zone piu' piccole si ignorano (1 m2 a 10 cm)
  };

  CoperturaVisita(std::shared_ptr<SDFMap> map, const Parametri& p);

  // Segna visitate le celle attorno al drone.
  void aggiorna(const Eigen::Vector3d& pos);

  // Zone di spazio percorribile (libero, fuori dalla fascia di gonfiaggio, alla
  // quota di volo) non ancora visitate, e quota di spazio percorribile visitato.
  void zone(std::vector<ZonaDaVisitare>& out, double& frazione);
  // Come sopra, ma due celle stanno nella stessa zona solo se 'proprietario' da' lo stesso
  // valore per entrambe (sciame: le zone si tagliano ai confini delle strisce).
  void zone(std::vector<ZonaDaVisitare>& out, double& frazione,
      const std::function<int(const Eigen::Vector3d&)>& proprietario);

  // La cella che contiene p e' gia' visitata?
  bool visitata(const Eigen::Vector3d& p) const;

  // Celle percorribili non ancora visitate entro r (in orizzontale) da p.
  int nonVisitateEntro(const Eigen::Vector3d& p, double r) const;

  const Parametri& parametri() const { return p_; }

private:
  bool percorribile(int i, int j) const;
  bool vistaLibera(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;

  std::shared_ptr<SDFMap> map_;
  Parametri p_;
  Eigen::Vector3d origine_;
  double res_;
  int nx_, ny_;
  std::vector<uint8_t> visitata_;
};

}  // namespace c2_expl

#endif
