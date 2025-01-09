/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include <arpa/inet.h>

#include <chrono>
#include <cstdlib>

#include "remote_modbus/protocol.hpp"
#include "remote_modbus_rtu/implementation.hpp"
#include "remote_modbus_rtu/node.hpp"
#include "remote_serial/utils.hpp"

using namespace std::chrono_literals;

namespace remote_modbus_rtu {

rclcpp::FutureReturnCode Implementation::holding_register_read_handler_real_(
    const std::shared_ptr<remote_modbus::srv::HoldingRegisterRead::Request>
        request,
    std::shared_ptr<remote_modbus::srv::HoldingRegisterRead::Response>
        response) {
  static const uint8_t fc = MODBUS_FC_READ_HOLDING_REGISTERS;
  uint8_t data[] = {
      request->leaf_id,
      fc,
      (uint8_t)((request->addr & 0xFF00) >> 8),   // high
      (uint8_t)(request->addr & 0xFF),            // low
      (uint8_t)((request->count & 0xFF00) >> 8),  // high
      (uint8_t)(request->count & 0xFF),           // low
      0,                                          // crc high
      0                                           // crclow
  };
  std::string output = modbus_rtu_frame_(data, sizeof(data));

  auto result = send_request_(request->leaf_id, fc, output);
  if (result.length() < 2) {  // 2 = fc + exception_code (or len)
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }

  uint8_t fc_received = (uint8_t)result[0];
  switch (fc_received) {
    case fc:
      // See if we have amount of data that is consistent with length
      response->len = result[1];
      //check is the response length is odd (Dword must always be even number of bytes)
      if ((response->len & 1) || result.length() != 2 + (size_t)response->len) {
        return rclcpp::FutureReturnCode::INTERRUPTED;
      }

      // Read the dynamic part in
      for (int i = 0; i < response->len / 2; i++) {
        response->values.push_back(ntohs(*(uint16_t *)&result[2 + 2 * i]));
      }

      return rclcpp::FutureReturnCode::SUCCESS;

    case 0x80 | fc:
      // this is an error report
      response->exception_code = (uint8_t)result[1];
      /* fall through */

    default:
      return rclcpp::FutureReturnCode::INTERRUPTED;
  }
}

rclcpp::FutureReturnCode Implementation::holding_register_write_handler_real_(
    const std::shared_ptr<remote_modbus::srv::HoldingRegisterWrite::Request>
        request,
    std::shared_ptr<remote_modbus::srv::HoldingRegisterWrite::Response>
        response) {
  static const uint8_t fc = MODBUS_FC_PRESET_SINGLE_REGISTER;
  uint8_t data[] = {
      request->leaf_id,
      fc,
      (uint8_t)((request->addr & 0xFF00) >> 8),   // high
      (uint8_t)(request->addr & 0xFF),            // low
      (uint8_t)((request->value & 0xFF00) >> 8),  // high
      (uint8_t)(request->value & 0xFF),           // low
      0,                                          // crc high
      0                                           // crclow
  };
  std::string output = modbus_rtu_frame_(data, sizeof(data));

  auto result = send_request_(request->leaf_id, fc, output);
  if (result.length() < 2) {  // 2 = fc + exception_code (or len)
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }

  uint8_t fc_received = (uint8_t)result[0];
  switch (fc_received) {
    case fc:
      // See if we have amount of data that is consistent with length
      response->addr = ntohs(*(uint16_t *)&result[1]);
      response->value = ntohs(*(uint16_t *)&result[3]);

      return rclcpp::FutureReturnCode::SUCCESS;

    case 0x80 | fc:
      // this is an error report
      response->exception_code = (uint8_t)result[1];
      /* fall through */

    default:
      return rclcpp::FutureReturnCode::INTERRUPTED;
  }
}

rclcpp::FutureReturnCode
Implementation::holding_register_write_multiple_handler_real_(
    const std::shared_ptr<
        remote_modbus::srv::HoldingRegisterWriteMultiple::Request>
        request,
    std::shared_ptr<remote_modbus::srv::HoldingRegisterWriteMultiple::Response>
        response) {
  (void)request;
  (void)response;
  // TODO(clairbee): send the request to the serial line

  return rclcpp::FutureReturnCode::INTERRUPTED;
}

}  // namespace remote_modbus_rtu
