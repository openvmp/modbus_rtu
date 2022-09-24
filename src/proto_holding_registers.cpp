/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include <chrono>
#include <cstdlib>

#include "modbus/protocol.hpp"
#include "modbus_rtu/interface.hpp"
#include "modbus_rtu/node.hpp"
#include "serial/utils.hpp"

using namespace std::chrono_literals;

namespace modbus_rtu {

rclcpp::FutureReturnCode
ModbusRtuInterface::holding_register_read_handler_real_(
    const std::shared_ptr<modbus::srv::HoldingRegisterRead::Request> request,
    std::shared_ptr<modbus::srv::HoldingRegisterRead::Response> response) {
  uint8_t data[] = {
      request->leaf_id,
      MODBUS_FC_READ_HOLDING_REGISTERS,
      (uint8_t)((request->addr & 0xFF00) >> 8),   // high
      (uint8_t)(request->addr & 0xFF),            // low
      (uint8_t)((request->count & 0xFF00) >> 8),  // high
      (uint8_t)(request->count & 0xFF),           // low
      0,                                          // crc high
      0                                           // crclow
  };
  std::string output = modbus_rtu_frame_(data, sizeof(data));

  auto result = send_request_(request->leaf_id, output);
  if (result.length() < 2) {  // 2 = fc + exception_code (or len)
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }

  uint8_t fc = (uint8_t)result[0];
  switch (fc) {
    case MODBUS_FC_READ_HOLDING_REGISTERS:
      // See if we have amount of data that is consistent with length
      response->len = result[1];
      if ((response->len & 1) || result.length() != 2 + response->len) {
        return rclcpp::FutureReturnCode::INTERRUPTED;
      }

      // Read the dynamic part in
      for (int i = 0; i < response->len / 2; i++) {
        response->values.push_back(ntohs(*(uint16_t *)&result[2 + 2 * i]));
      }

      return rclcpp::FutureReturnCode::SUCCESS;

    case 0x80:
      // this is an error report
      response->exception_code = (uint8_t)result[1];
      /* fall through */

    default:
      return rclcpp::FutureReturnCode::INTERRUPTED;
  }
}

rclcpp::FutureReturnCode
ModbusRtuInterface::holding_register_write_handler_real_(
    const std::shared_ptr<modbus::srv::HoldingRegisterWrite::Request> request,
    std::shared_ptr<modbus::srv::HoldingRegisterWrite::Response> response) {
  (void)request;
  (void)response;
  // TODO(clairbee): send the request to the serial line
  return rclcpp::FutureReturnCode::INTERRUPTED;
}

rclcpp::FutureReturnCode
ModbusRtuInterface::holding_register_write_multiple_handler_real_(
    const std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Request>
        request,
    std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Response>
        response) {
  (void)request;
  (void)response;
  // TODO(clairbee): send the request to the serial line
  return rclcpp::FutureReturnCode::INTERRUPTED;
}

}  // namespace modbus_rtu