/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_modbus_rtu/factory.hpp"

#include "remote_modbus/interface_remote.hpp"
#include "remote_modbus_rtu/implementation.hpp"

namespace remote_modbus_rtu {

std::shared_ptr<remote_modbus::Interface> Factory::New(rclcpp::Node *node) {
  rclcpp::Parameter use_remote;
  if (!node->has_parameter("use_remote")) {
    node->declare_parameter("use_remote", true);
  }
  node->get_parameter("use_remote", use_remote);

  rclcpp::Parameter is_remote;
  node->declare_parameter("modbus_is_remote", use_remote.as_bool());
  node->get_parameter("modbus_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<remote_modbus::RemoteInterface>(node);
  } else {
    return std::make_shared<Implementation>(node);
  }
}

}  // namespace remote_modbus_rtu