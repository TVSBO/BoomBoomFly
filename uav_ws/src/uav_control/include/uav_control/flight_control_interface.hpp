#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "uav_control/px4_dds_interface.hpp"
#ifdef BUILD_MAVROS_INTERFACE
#include "uav_control/mavros_interface.hpp"
#endif

namespace uav_control
{

class FlightControlInterface
{
public:
  explicit FlightControlInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    auto config = std::make_shared<rclcpp::Node>("uav_control", options);
    const auto backend = config->declare_parameter<std::string>("backend", "");
    if (backend != "dds" && backend != "mavros") {
      throw std::invalid_argument(
        "backend must be 'dds' or 'mavros'; set uav_control.backend in "
        "uav_bringup/config/uav.yaml, got: " + backend);
    }
#ifndef BUILD_MAVROS_INTERFACE
    if (backend == "mavros") {
      throw std::runtime_error(
        "MAVROS backend is not built; rebuild with -DBUILD_MAVROS_INTERFACE=ON");
    }
#endif
    config.reset();
    if (backend == "dds") {
      node_ = std::make_shared<Px4DdsInterface>(options);
    }
#ifdef BUILD_MAVROS_INTERFACE
    else {
      node_ = std::make_shared<MavrosInterface>(options);
    }
#endif
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    node_->declare_parameter<std::string>("backend", backend, descriptor);
    RCLCPP_INFO(node_->get_logger(), "Selected backend: %s (fixed until restart)", backend.c_str());
  }

  rclcpp::Node::SharedPtr get_node() const {return node_;}

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace uav_control
