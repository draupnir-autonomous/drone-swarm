#include "bspline/non_uniform_bspline.h"
#include <nav_msgs/msg/odometry.hpp>
#include <c2x_msgs/msg/bspline.hpp>
#include <c2x_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <active_perception/perception_utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}
using c2_expl::NonUniformBspline;
using c2_expl::PerceptionUtils;

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub, traj_pub;
rclcpp::Publisher<c2x_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traj_vel_pub;

nav_msgs::msg::Odometry odom;
c2x_msgs::msg::PositionCommand cmd;

// Info of generated traj
vector<NonUniformBspline> traj_;
double traj_duration_;
// Must default-construct with RCL_ROS_TIME explicitly: rclcpp::Time's
// default constructor uses RCL_SYSTEM_TIME, but replanCallback() can fire
// (via /planning/replan) before the first real Bspline ever arrives and
// assigns a proper value, and subtracting against node_->now() (RCL_ROS_TIME)
// throws if the sources don't match.
rclcpp::Time start_time_(0, 0, RCL_ROS_TIME);
// Virtual trajectory time. Advanced by the elapsed wall time scaled down by how
// far the vehicle has fallen behind the setpoint - see cmdCallback.
double traj_t_ = 0.0;
// Velocita' della traiettoria all'ultimo ciclo, PRIMA della scalatura del tempo
// (vedi cmdCallback): da' la direzione di marcia anche a orologio fermo.
Eigen::Vector3d vel_traiettoria_ = Eigen::Vector3d::Zero();
rclcpp::Time last_cmd_time_(0, 0, RCL_ROS_TIME);
bool traj_time_scaling_ = true;
double track_err_free_ = 0.3;   // below this the trajectory runs at normal speed
double track_err_stop_ = 1.0;   // at this it stops advancing and waits
double brake_acc_ = 2.0;        // frenata oltre la fine della traiettoria (vedi cmdCallback)
int traj_id_ = 0;
int pub_traj_id_, pub_traj_vel_id_;

shared_ptr<PerceptionUtils> percep_utils_;

// Info of replan
bool receive_traj_ = false;
double replan_time_;

// Executed traj, commanded and real ones
vector<Eigen::Vector3d> traj_cmd_, traj_real_, vel_cmd_;

// Data for benchmark comparison. Must default-construct with RCL_ROS_TIME
// explicitly: rclcpp::Time's default constructor uses RCL_SYSTEM_TIME, but
// these get compared against node_->now() (RCL_ROS_TIME) before they're
// ever assigned a real value (e.g. `end_time - start_time` on the very
// first cmdCallback tick, before end_time has been set) - subtracting
// rclcpp::Time values with different time sources throws at runtime. The
// original ROS1 code had the same latent bug (ros::Time() has no time-source
// concept, so it just silently computed a garbage duration); ROS2 surfaces
// it as a hard crash instead.
rclcpp::Time start_time(0, 0, RCL_ROS_TIME), end_time(0, 0, RCL_ROS_TIME),
    last_time(0, 0, RCL_ROS_TIME);
double energy;

// Loop correction
Eigen::Matrix3d R_loop;
Eigen::Vector3d T_loop;
bool isLoopCorrection = false;

// Swarm
int drone_id_;
double color_r, color_g, color_b, color_a;
double max_vel_;

double calcPathLength(const vector<Eigen::Vector3d>& path) {
  if (path.empty()) return 0;
  double len = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

Eigen::Vector4d getColor(const double& h, const Eigen::Vector4d& color, double alpha) {
  double h1 = h;
  if (h1 < 0.0) {
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;

  if (h1 >= -1e-4 && h1 < 1.0 / 2) {
    lambda = (h1 - 0.0) * 2;
    color1 = Eigen::Vector4d(1, 1, 1, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else {
    lambda = (h1 - 1.0 / 2) * 2;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}

void displayTrajWithColor(
    vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = node_->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;
  traj_pub->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub->publish(mk);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void displayTrajVel(const vector<Eigen::Vector3d> positions,
    const vector<Eigen::Vector3d> velocities, Eigen::Vector4d color, int id) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.points.resize(positions.size());

  for (int i = 0; i < int(positions.size()); i++) {
    cloud.points[i].x = positions[i](0);
    cloud.points[i].y = positions[i](1);
    cloud.points[i].z = positions[i](2);

    double vel_norm = std::min(std::max(velocities[i].norm() / max_vel_, 0.0), 1.0);
    Eigen::Vector4d c = getColor(vel_norm, color, 1.0);
    cloud.points[i].r = c(0) * 255;
    cloud.points[i].g = c(1) * 255;
    cloud.points[i].b = c(2) * 255;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = node_->now();
  traj_vel_pub->publish(cloud_msg);
}

void drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = node_->now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::msg::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  Eigen::Vector4d color(0, 0, 0, 1);

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::msg::Marker::DELETE;
  cmd_vis_pub->publish(mk);

  if (list1.size() == 0) return;

  // Pub new marker
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::msg::Marker::ADD;
  cmd_vis_pub->publish(mk);
}

void replanCallback(const std_msgs::msg::Empty::ConstSharedPtr /*msg*/) {
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.2;
  rclcpp::Time time_now = node_->now();
  double t_stop = (time_now - start_time_).seconds() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(const std_msgs::msg::Empty::ConstSharedPtr /*msg*/) {
  // Clear the executed traj data
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O") return;
  odom = *msg;
  traj_real_.push_back(Eigen::Vector3d(
      odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void pgTVioCallback(const geometry_msgs::msg::Pose::ConstSharedPtr msg) {
  // World to odom
  Eigen::Quaterniond q = Eigen::Quaterniond(
      msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  R_loop = q.toRotationMatrix();
  T_loop << msg->position.x, msg->position.y, msg->position.z;
}

void visCallback() {
  // Draw the executed traj (desired state)
  displayTrajWithColor(
      traj_cmd_, 0.3, Eigen::Vector4d(color_r, color_g, color_b, color_a), pub_traj_id_);
  displayTrajVel(
      traj_cmd_, vel_cmd_, Eigen::Vector4d(color_r, color_g, color_b, color_a), pub_traj_vel_id_);
}

void bsplineCallback(const c2x_msgs::msg::Bspline::ConstSharedPtr msg) {
  // Received traj should have ascending traj_id
  if (msg->traj_id <= traj_id_) {
    RCLCPP_ERROR(node_->get_logger(), "out of order bspline.");
    return;
  }

  // Parse the msg
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }
  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  if (!pos_traj.setKnot(knots)) {
    // Knot vector inconsistent with the control points: evaluating such a
    // spline reads past the end of its own arrays. Drop the message.
    RCLCPP_ERROR(node_->get_logger(),
        "Malformed bspline: %zu knots for %zu control points at order %d, expected %zu - dropped",
        msg->knots.size(), msg->pos_pts.size(), msg->order,
        msg->pos_pts.size() + msg->order + 1);
    return;
  }

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (size_t i = 0; i < msg->yaw_pts.size(); ++i) yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  // Must match node_->now()'s time source (RCL_ROS_TIME) explicitly: the
  // default clock type for this constructor otherwise doesn't necessarily
  // agree, and rclcpp::Time subtraction throws if the two operands' time
  // sources differ (surfaced as "can't subtract times with different time
  // sources" - not a compile-time error, only at runtime on first use).
  start_time_ = rclcpp::Time(msg->start_time, RCL_ROS_TIME);
  // A fresh trajectory starts from its own beginning; any lag accumulated
  // against the previous one is not inherited.
  traj_t_ = std::max(0.0, (node_->now() - start_time_).seconds());
  last_cmd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;

  // Record the start time of flight
  if (start_time.nanoseconds() == 0) {
    RCLCPP_WARN(node_->get_logger(), "start flight");
    start_time = node_->now();
  }
}

void cmdCallback() {
  // No publishing before receive traj data
  if (!receive_traj_) return;

  rclcpp::Time time_now = node_->now();

  // Advance trajectory time against the vehicle, not against the wall clock.
  //
  // Evaluating the B-spline at (now - start_time) assumes the drone is wherever
  // the plan says it should be. With PX4 it is not: measured tracking error ran
  // past a metre, and once the setpoint is that far ahead the separation the
  // optimizer computed between *trajectories* no longer describes the distance
  // between *drones* - which is the guarantee the whole multi-UAV scheme rests
  // on. Upstream never hits this because MARSIM's ideal cascade PID tracks to
  // within centimetres.
  //
  // So slow the trajectory clock down as the error grows, and stop it entirely
  // at track_err_stop: the plan waits for the vehicle instead of running away
  // from it. If the drone genuinely cannot catch up the trajectory simply
  // holds, and the FSM's periodic replan re-anchors it from the real position.
  double t_cur;
  double scale = 1.0;  // d(tempo della traiettoria)/d(tempo vero), vedi sotto
  if (traj_time_scaling_ && last_cmd_time_.nanoseconds() > 0) {
    const double dt = std::max(0.0, (time_now - last_cmd_time_).seconds());
    const Eigen::Vector3d cur_pos(
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    const Eigen::Vector3d cmd_pos(cmd.position.x, cmd.position.y, cmd.position.z);
    // Direzione dalla velocita' della TRAIETTORIA, non da quella pubblicata: con
    // l'orologio fermo quella pubblicata e' nulla (vedi sotto) e non ha direzione.
    const Eigen::Vector3d cmd_vel = vel_traiettoria_;
    // Conta solo il drone INDIETRO rispetto al riferimento (piu' lo scarto di
    // lato), non quello AVANTI. Con la distanza pura, un drone davanti al
    // riferimento - riferimento ripartito alle sue spalle, o superato in una
    // frenata - faceva rallentare il riferimento, che restava ancora piu'
    // indietro e se lo riportava dietro: uno degli anelli del dondolio della
    // run dell'utente in cubicle_office (5 ottobre 2026). Aspettare ha senso
    // solo per chi e' rimasto indietro.
    const Eigen::Vector3d scarto = cmd_pos - cur_pos;
    double err = scarto.norm();
    if (cmd_vel.norm() > 0.1) {
      const Eigen::Vector3d dir = cmd_vel.normalized();
      const double lungo = scarto.dot(dir);  // > 0: drone indietro
      const double lato = (scarto - lungo * dir).norm();
      err = std::hypot(std::max(0.0, lungo), lato);
    }
    if (err > track_err_free_) {
      scale = (track_err_stop_ - err) / std::max(1e-3, track_err_stop_ - track_err_free_);
      scale = std::min(1.0, std::max(0.0, scale));
    }
    traj_t_ += dt * scale;
    t_cur = traj_t_;
  } else {
    t_cur = (time_now - start_time_).seconds();
    traj_t_ = t_cur;
  }
  last_cmd_time_ = time_now;

  Eigen::Vector3d pos, vel, acc, jer;
  double yaw, yawdot;

  if (t_cur < 0.0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "[Traj server]: traj start in future (%.3fs), hold last cmd", -t_cur);
    pos = Eigen::Vector3d(cmd.position.x, cmd.position.y, cmd.position.z);
    vel = Eigen::Vector3d(cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
    acc = Eigen::Vector3d(cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z);
    yaw = cmd.yaw;
    yawdot = cmd.yaw_dot;

    jer.setZero();
  } else if (t_cur < traj_duration_) {
    // Current time within range of planned traj
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);
  } else {
    // Current time exceed range of planned traj
    // keep publishing the final position and yaw
    //
    // ...MA FRENANDO, se alla fine la traiettoria ha ancora velocita': una
    // traiettoria troncata (planning/replan) o un tratto corto partito veloce
    // verso una meta vicina finiscono a ~1 m/s, e tenere fermo l'ultimo punto
    // era una frenata a gradino. Il drone lo superava di 0.7 m (V28,
    // cubicle_office: 0.32 m da una parete) e tornava indietro a 1.8 m/s. Qui
    // il riferimento prosegue nella stessa direzione decelerando a brake_acc
    // fino a fermarsi: 0.25 m in piu' da 1 m/s, senza gradino.
    const Eigen::Vector3d p_fine = traj_[0].evaluateDeBoorT(traj_duration_);
    const Eigen::Vector3d v_fine = traj_[1].evaluateDeBoorT(traj_duration_);
    const double vn = v_fine.norm();
    const double tau = t_cur - traj_duration_;
    if (brake_acc_ > 0.0 && vn > 0.05) {
      const Eigen::Vector3d dir = v_fine / vn;
      const double t_stop = vn / brake_acc_;
      const double tt = std::min(tau, t_stop);
      pos = p_fine + dir * (vn * tt - 0.5 * brake_acc_ * tt * tt);
      if (tau < t_stop) {
        vel = dir * (vn - brake_acc_ * tau);
        acc = -brake_acc_ * dir;
      } else {
        vel.setZero();
        acc.setZero();
      }
    } else {
      pos = p_fine;
      vel.setZero();
      acc.setZero();
    }
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;
    jer.setZero();
  }

  // NIENTE ANTICIPO A OROLOGIO FERMO.
  //
  // Con l'orologio rallentato o FERMO si pubblicavano velocita' e accelerazione
  // PIENE dell'istante corrente: una posizione immobile con un anticipo che
  // spingeva altrove, e PX4 segue l'anticipo.
  // Misurato (parco, run 2026-10-18_1248, 18 ottobre 2026): drone rimasto >1 m
  // indietro dopo un cambio di piano, orologio fermo per ~2 s con l'istante
  // congelato a vz -0.46 m/s e az -1.05 m/s2. Con MPC_Z_P = 1 PX4 si assesta
  // dove la correzione di posizione pareggia l'anticipo, 0.46-0.51 m SOTTO il
  // riferimento: quota vera da 0.86 a 0.13 m (corpo appoggiato = 0). In
  // orizzontale l'anticipo congelato (ay -1.65) annullava la correzione verso il
  // riferimento, quindi il drone non recuperava lo scarto e l'orologio restava
  // fermo: un blocco rotto solo dal piano successivo, 2.4 s dopo.
  // L'anticipo si sfuma quindi a zero quando l'orologio rallenta oltre la meta'
  // (scale < 0.5, cioe' scarto oltre 0.65 m): velocita' * g, accelerazione * g^2,
  // con g = scale / 0.5. PROVATO E SCARTATO: le derivate esatte del riferimento
  // rallentato, v*scale e a*scale^2, da scarto 0.3 m in su. Toglievano la spinta
  // di recupero proprio al drone rimasto indietro (PX4 lo riporta sul riferimento
  // con la sola correzione di posizione): run 1427, parco, scarto orizzontale
  // riferimento-drone mediana 0.22 m e oltre 0.3 m nel 24% del volo, contro
  // 0.07-0.13 m e 2-11% nelle 8 run precedenti; ripianificazioni "drone indietro"
  // 38 contro 4-8. Fino a scarto 0.65 m l'anticipo resta pieno come prima.
  if (t_cur >= 0.0) {
    vel_traiettoria_ = vel;
    const double g = std::min(1.0, scale / 0.5);
    vel *= g;
    acc *= g * g;
    yawdot *= g;
  }

  // The upstream C2-Explorer repo hardcodes yaw=0/yawdot=0 here unconditionally (same
  // comment, same override, verified against reference/C2-Explorer's own traj_server.cpp)
  // - discarding the yaw active_perception carefully computed per viewpoint for
  // informative viewing. Re-enabled here to match the paper's demonstrated behavior, now
  // that the actual instability root cause (position_command_bridge's OffboardControlMode
  // declaring position-only while trajectory_setpoint carried real velocity/acceleration
  // too) is fixed. If this reintroduces oscillation/instability, that's the signal the
  // upstream override was hiding a real problem rather than just being stale debug code.

  // Report info of the whole flight
  double len = calcPathLength(traj_cmd_);
  double flight_t = (end_time - start_time).seconds();
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "\033[1;33mDrone %d [%lf,%lf,%lf,%lf], [time, length, vel, energy] \033[1;33m", drone_id_,
      flight_t, len, len / flight_t, energy);

  if (isLoopCorrection) {
    pos = R_loop.transpose() * (pos - T_loop);
    vel = R_loop.transpose() * vel;
    acc = R_loop.transpose() * acc;

    Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
    yaw_dir = R_loop.transpose() * yaw_dir;
    yaw = atan2(yaw_dir[1], yaw_dir[0]);
  }

  cmd.header.stamp = time_now;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;
  pos_cmd_pub->publish(cmd);

  percep_utils_->setPose(pos, yaw);
  vector<Eigen::Vector3d> l1, l2;
  percep_utils_->getFOV(l1, l2);
  drawFOV(l1, l2);

  // Record info of the executed traj
  if (traj_cmd_.size() == 0) {
    // Add the first position
    traj_cmd_.push_back(pos);
    vel_cmd_.push_back(vel);
  } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    // Add new different commanded position
    traj_cmd_.push_back(pos);
    vel_cmd_.push_back(vel);
    double dt = (time_now - last_time).seconds();
    energy += jer.squaredNorm() * dt;
    end_time = node_->now();

    if (energy > 10000) {
      std::cout << "jer: " << jer.transpose() << ", dt: " << dt << std::endl;
    }
  }
  last_time = time_now;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("traj_server");

  auto bspline_sub = node_->create_subscription<c2x_msgs::msg::Bspline>(
      "planning/bspline", 10, bsplineCallback);
  auto replan_sub =
      node_->create_subscription<std_msgs::msg::Empty>("planning/replan", 10, replanCallback);
  auto new_sub = node_->create_subscription<std_msgs::msg::Empty>("planning/new", 10, newCallback);
  auto odom_sub =
      node_->create_subscription<nav_msgs::msg::Odometry>("odom_world", 50, odomCallbck);
  auto pg_T_vio_sub = node_->create_subscription<geometry_msgs::msg::Pose>(
      "loop_fusion/pg_T_vio", 10, pgTVioCallback);

  cmd_vis_pub =
      node_->create_publisher<visualization_msgs::msg::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node_->create_publisher<c2x_msgs::msg::PositionCommand>("position_cmd", 50);
  traj_pub = node_->create_publisher<visualization_msgs::msg::Marker>("planning/travel_traj", 10);
  traj_vel_pub =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("planning/travel_traj_vel", 10);

  auto cmd_timer =
      node_->create_wall_timer(std::chrono::duration<double>(0.01), cmdCallback);
  auto vis_timer =
      node_->create_wall_timer(std::chrono::duration<double>(0.25), visCallback);

  node_->declare_parameter("traj_server.pub_traj_id", -1);
  node_->declare_parameter("traj_server.pub_traj_vel_id", -1);
  node_->declare_parameter("fsm.replan_time", 0.1);
  node_->declare_parameter("loop_correction.isLoopCorrection", false);
  node_->declare_parameter("traj_server.drone_id", 1);
  node_->declare_parameter("traj_server.max_vel", 2.0);
  traj_time_scaling_ = node_->declare_parameter("traj_server.time_scaling", true);
  track_err_free_ = node_->declare_parameter("traj_server.track_err_free", 0.3);
  track_err_stop_ = node_->declare_parameter("traj_server.track_err_stop", 1.0);
  brake_acc_ = node_->declare_parameter("traj_server.brake_acc", 2.0);
  node_->declare_parameter("color_r", 1.0);
  node_->declare_parameter("color_g", 0.0);
  node_->declare_parameter("color_b", 0.0);
  node_->declare_parameter("color_a", 1.0);
  node_->get_parameter("traj_server.pub_traj_id", pub_traj_id_);
  node_->get_parameter("traj_server.pub_traj_vel_id", pub_traj_vel_id_);
  node_->get_parameter("fsm.replan_time", replan_time_);
  node_->get_parameter("loop_correction.isLoopCorrection", isLoopCorrection);
  node_->get_parameter("traj_server.drone_id", drone_id_);
  node_->get_parameter("traj_server.max_vel", max_vel_);
  node_->get_parameter("color_r", color_r);
  node_->get_parameter("color_g", color_g);
  node_->get_parameter("color_b", color_b);
  node_->get_parameter("color_a", color_a);

  RCLCPP_WARN(node_->get_logger(), "[Traj server]: init...");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Control parameter
  cmd.kx = { 5.7, 5.7, 6.2 };
  cmd.kv = { 3.4, 3.4, 4.0 };

  cmd.header.stamp = node_->now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = c2x_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = -4.5;
  cmd.position.y = -3.0;
  cmd.position.z = 0.0;
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = 0.0;
  cmd.yaw_dot = 0.0;

  percep_utils_.reset(new PerceptionUtils(node_));

  R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
  T_loop = Eigen::Vector3d(0, 0, 0);

  RCLCPP_WARN(node_->get_logger(), "[Traj server]: ready.");
  rclcpp::spin(node_);

  return 0;
}
