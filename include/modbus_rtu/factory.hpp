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

#include "modbus/interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace modbus_rtu {

class Factory {
 public:
  static std::shared_ptr<modbus::Interface> New(rclcpp::Node *node);
};

}  // namespace modbus_rtu

#endif  // OPENVMP_MODBUS_RTU_FACTORY_H