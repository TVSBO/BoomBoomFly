#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "uav_control/flight_control_interface.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    uav_control::FlightControlInterface control;
    rclcpp::spin(control.get_node());
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("uav_control"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
