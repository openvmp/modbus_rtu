/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-25
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_MODBUS_RTU_FACTORY_H
#define OPENVMP_MODBUS_RTU_FACTORY_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "remote_modbus/interface.hpp"

namespace remote_modbus_rtu {

class Factory {
 public:
  static std::shared_ptr<remote_modbus::Interface> New(rclcpp::Node *node);
};

}  // namespace remote_modbus_rtu

#endif  // OPENVMP_MODBUS_RTU_FACTORY_H