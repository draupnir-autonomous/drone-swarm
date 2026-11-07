#include <rclcpp/rclcpp.hpp>
#include <sys/prctl.h>
#include <cstdlib>
#include <exploration_manager/c2_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace c2_expl;

int main(int argc, char** argv) {
  // Profilazione: con C2X_PROFILO=1 nell'ambiente qualunque processo dell'utente
  // puo' agganciarsi con gdb per campionare gli stack (ptrace_scope=1 lo permette
  // altrimenti solo ai processi antenati). Di serie spento.
  if (const char* p = std::getenv("C2X_PROFILO"); p && p[0] == '1')
    prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("exploration_node");

  C2ExplorationFSM expl_fsm;
  expl_fsm.init(node);

  rclcpp::sleep_for(std::chrono::seconds(1));

  // MultiThreadedExecutor is required, not optional: C2ExplorationManager's
  // LKH TSP/ACVRP service calls block the FSM timer thread on a future - that
  // future only resolves if a *different* executor thread is free to process
  // the incoming service response concurrently. A single-threaded executor
  // would deadlock the first time the exploration FSM needs a tour solved.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
