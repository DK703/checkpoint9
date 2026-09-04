#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "my_components/approach_service_servermc.hpp"

int main(int argc, char *argv[])
{


  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto approachservice = std::make_shared<my_components::ApproachService>(options);
  exec.add_node(approachservice);
  exec.spin();
  rclcpp::shutdown();
  return 0;


}
