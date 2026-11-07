#include <cstdio>
#include <cstdlib>
#include "LIVMapper.h"

int main(int argc, char ** argv)
{
  // C2-Explorer (ottobre 2026): FAST-LIVO2 stampa su stdout a ogni fotogramma una tabella
  // dei tempi e la diagnostica di LIO e VIO (~600 righe al secondo). Sotto ros2 launch
  // stdout e' una pipe letta da un processo Python: se questo non la svuota in tempo, la
  // scrittura blocca il calcolo e la stima arriva in ritardo. Di serie stdout va in
  // /dev/null; i messaggi ROS (RCLCPP_*) vanno su stderr e restano. C2X_LIVO_STDOUT=1
  // lo lascia com'e' per la diagnosi.
  {
    const char * e = std::getenv("C2X_LIVO_STDOUT");
    if (!(e && e[0] == '1') && !std::freopen("/dev/null", "w", stdout)) {
      std::perror("freopen stdout");
    }
  }
  try {
    mp_runtime_init();
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("laserMapping");
    image_transport::ImageTransport it_(node);
    LIVMapper mapper(node, "laserMapping");
    mapper.initializeSubscribersAndPublishers(node, it_);
    mapper.run(node);
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("laserMapping"), "Unhandled exception in main: %s", e.what());
    return 1;
  }
  return 0;
}