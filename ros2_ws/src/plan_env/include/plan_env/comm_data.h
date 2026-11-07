#ifndef COMM_DATA_H
#define COMM_DATA_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <string>

using Eigen::Vector3d;
using std::string;
using std::vector;

namespace c2_expl {
enum class LinkType { NO_CONNECTION, CONNECTED };

struct CommunicationData {
  int ego_id_;
  Eigen::Vector3d ego_position_;

  bool have_odom_;
  rclcpp::Time last_odom_time_;

  vector<string> linktype_str_;
};

struct NeighborInfo {
  int neighbor_id_;
  Eigen::Vector3d neighbor_position_;

  rclcpp::Time last_update_time_;
  LinkType connection_type_;
};

struct HeartbeatMsg {
  int drone_id_;

  Eigen::Vector3d position_;
  rclcpp::Time heartbeat_timestamp_;
};

struct CommunicationParams {
  // Portata massima del collegamento. Con un collegamento ottico (laser) non e'
  // piu' il vincolo stringente: in aria libera un laser arriva a centinaia di
  // metri, mentre un muro lo interrompe del tutto. Il vincolo vero e' la linea
  // di vista, vedi los_enabled_.
  double connection_threshold_;

  // Il collegamento richiede che il segmento fra i due droni sia libero.
  //
  // Modella un collegamento OTTICO invece che radio. La differenza non e' di
  // grado ma di natura: due droni a 3 m in stanze adiacenti NON si vedono,
  // mentre due droni a 25 m lungo lo stesso corridoio si'. Con la sola distanza
  // la topologia risultante e' una sfera, che dentro un edificio non somiglia a
  // niente di reale.
  bool los_enabled_;

  // Se lo spazio ANCORA IGNOTO interrompe il collegamento.
  //
  // Falso di default, e la ragione e' pratica: a inizio missione la mappa e'
  // vuota, quindi trattare l'ignoto come opaco lascerebbe i droni scollegati
  // proprio quando sono vicini e in vista reale. Solo cio' che si e' misurato
  // essere occupato blocca. Metterlo a vero da' il modello prudente - utile per
  // capire quanto il coordinamento dipenda da collegamenti non ancora
  // verificati.
  bool los_unknown_blocks_;

  double heartbeat_interval_;
  double neighbor_timeout_;
};

}  // namespace c2_expl
#endif
