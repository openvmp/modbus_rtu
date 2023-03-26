/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_modbus_rtu/node.hpp"

namespace ros2_modbus_rtu {

Node::Node() : rclcpp::Node::Node("modbus_rtu") {
  impl_ = std::make_shared<Implementation>(this);
}

}  // namespace ros2_modbus_rtu
