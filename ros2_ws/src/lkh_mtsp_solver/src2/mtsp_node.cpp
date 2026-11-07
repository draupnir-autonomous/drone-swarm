#include <rclcpp/rclcpp.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <sys/wait.h>

#include <lkh_mtsp_solver/lkh3_interface.h>
#include <c2x_msgs/srv/solve_tsp.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

using std::string;

namespace {
rclcpp::Node::SharedPtr node_;
std::string mtsp_dir1_, mtsp_dir2_, mtsp_dir3_;
int drone_id_, problem_id_;
// Path to the standalone LKH binary used for the ACVRP allocation. Defaults to
// the copy this package builds and installs; override with the lkh_binary
// parameter to point at a system-wide LKH instead.
std::string lkh_binary_;

bool forcePrecisionOne(const std::string& par_file) {
  std::ifstream fin(par_file);
  if (!fin.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open par file for precision patch: %s",
        par_file.c_str());
    return false;
  }

  std::vector<std::string> lines;
  lines.reserve(64);
  std::string line;
  while (std::getline(fin, line)) {
    if (line.rfind("PRECISION", 0) == 0) continue;
    lines.push_back(line);
  }
  fin.close();

  std::ofstream fout(par_file, std::ios::trunc);
  if (!fout.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to rewrite par file for precision patch: %s",
        par_file.c_str());
    return false;
  }
  for (const auto& l : lines) {
    fout << l << "\n";
  }
  fout << "PRECISION = 1\n";
  fout.close();
  return true;
}

void mtspCallback(const std::shared_ptr<c2x_msgs::srv::SolveTSP::Request> req,
    std::shared_ptr<c2x_msgs::srv::SolveTSP::Response> res) {
  (void)res;
  if (req->prob == 1)
    solveMTSPWithLKH3(mtsp_dir1_.c_str());
  else if (req->prob == 2)
    solveMTSPWithLKH3(mtsp_dir2_.c_str());
  else if (req->prob == 3) {
    if (!forcePrecisionOne(mtsp_dir3_)) {
      return;
    }
    string cmd = "\"" + lkh_binary_ + "\" \"" + mtsp_dir3_ + "\"";
    const int ret = system(cmd.c_str());
    if (ret != 0) {
      if (WIFSIGNALED(ret)) {
        RCLCPP_ERROR(node_->get_logger(), "LKH process terminated by signal %d (par: %s)",
            WTERMSIG(ret), mtsp_dir3_.c_str());
      } else {
        RCLCPP_ERROR(
            node_->get_logger(), "LKH process failed with status %d (par: %s)", ret, mtsp_dir3_.c_str());
      }
      return;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown MTSP request type: %u", req->prob);
    return;
  }
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("mtsp_node");

  std::string mtsp_dir = node_->declare_parameter("exploration.mtsp_dir", std::string("null"));
  drone_id_ = node_->declare_parameter("exploration.drone_id", 1);
  problem_id_ = node_->declare_parameter("exploration.problem_id", 1);

  std::string default_lkh;
  try {
    default_lkh = ament_index_cpp::get_package_prefix("lkh_mtsp_solver") +
                  "/lib/lkh_mtsp_solver/LKH";
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "Could not resolve lkh_mtsp_solver prefix (%s), "
                                     "falling back to /usr/local/bin/LKH", e.what());
    default_lkh = "/usr/local/bin/LKH";
  }
  lkh_binary_ = node_->declare_parameter("lkh_binary", default_lkh);

  mtsp_dir1_ = mtsp_dir + "/amtsp_" + std::to_string(drone_id_) + ".par";
  mtsp_dir2_ = mtsp_dir + "/amtsp2_" + std::to_string(drone_id_) + ".par";
  mtsp_dir3_ = mtsp_dir + "/amtsp3_" + std::to_string(drone_id_) + ".par";

  string service_name;
  if (problem_id_ == 1) {  // TSP
    service_name = "/solve_tsp_" + std::to_string(drone_id_);
  } else if (problem_id_ == 2) {  // ACVRP
    service_name = "/solve_acvrp_" + std::to_string(drone_id_);
  }
  auto mtsp_server = node_->create_service<c2x_msgs::srv::SolveTSP>(service_name, mtspCallback);

  RCLCPP_WARN(node_->get_logger(), "MTSP server %d is ready.", drone_id_);
  rclcpp::spin(node_);
  rclcpp::shutdown();

  return 0;
}
