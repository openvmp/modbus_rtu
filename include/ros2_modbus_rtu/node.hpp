/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_MODBUS_RTU_NODE_H
#define OPENVMP_MODBUS_RTU_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_modbus_rtu/implementation.hpp"

namespace ros2_modbus_rtu {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  std::shared_ptr<Implementation> impl_;
};

}  // namespace ros2_modbus_rtu

#endif  // OPENVMP_MODBUS_RTU_NODE_H
