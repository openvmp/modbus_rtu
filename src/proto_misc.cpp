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

#include "ros2_modbus/protocol.hpp"
#include "ros2_modbus_rtu/implementation.hpp"
#include "ros2_modbus_rtu/node.hpp"
#include "ros2_serial/utils.hpp"

using namespace std::chrono_literals;

namespace ros2_modbus_rtu {

rclcpp::FutureReturnCode Implementation::get_com_event_log_handler_real_(
    const std::shared_ptr<ros2_modbus::srv::GetComEventLog::Request> request,
    std::shared_ptr<ros2_modbus::srv::GetComEventLog::Response> response) {
  static const uint8_t fc = MODBUS_FC_GET_COM_EVENT_LOG;
  uint8_t data[] = {
      request->leaf_id, fc,
      0,  // crc high
      0   // crclow
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
      // Read the dynamic part in
      for (size_t i = 1; i < result.length(); i++) {
        response->data.push_back(result[i]);
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

rclcpp::FutureReturnCode Implementation::read_device_id_handler_real_(
    const std::shared_ptr<ros2_modbus::srv::ReadDeviceId::Request> request,
    std::shared_ptr<ros2_modbus::srv::ReadDeviceId::Response> response) {
  static const uint8_t fc = MODBUS_FC_READ_DEVICE_ID;

  uint8_t conformity_level = 3;
  for (uint8_t device_id_code = 1;
       device_id_code <=
           conformity_level     // don't go beyond the device capabilities
       && device_id_code <= 3;  // don't go beyond the support of this code
       device_id_code++) {
    uint8_t object_id = 0;
    do {
      uint8_t data[] = {
          request->leaf_id,
          fc,
          0x0E,  // "MEI Type" == "Modbus Encapsulated Protocol"
          device_id_code,
          object_id,
          0,  // crc high
          0   // crclow
      };
      std::string output = modbus_rtu_frame_(data, sizeof(data));

      auto result = send_request_(request->leaf_id, fc, output);
      if (result.length() < 7) {  // 2 = fc + exception_code (or len)
        return rclcpp::FutureReturnCode::INTERRUPTED;
      }

      uint8_t fc_received = (uint8_t)result[0];
      switch (fc_received) {
        case fc: {
          // TODO(clairbee): check mei type in result[1]
          // TODO(clairbee): check device_id_code in result[2]
          conformity_level = result[3];
          uint8_t more_follows = result[4];
          object_id = result[5];  // next object id

          // See if we have amount of data that is consistent with length
          int8_t num = result[6];
          response->objects_num += num;

          uint8_t offset = 7;
          while (offset < 255 && num > 0) {
            uint8_t len = result[offset + 1];
            if (result.length() < (size_t)offset + 2 + len) {
              return rclcpp::FutureReturnCode::INTERRUPTED;
            }

            // Read the dynamic part in
            for (uint8_t i = 0; i < 2 + len; i++) {
              response->data.push_back(result[offset + i]);
            }
            offset += 2 + len;
            num--;
          }

          if (more_follows) {
            continue;  // restart "do {} while()"
          }
          break;
        }

        case 0x80 | fc:
          // this is an error report
          response->exception_code = (uint8_t)result[1];
          /* fall through */

        default:
          return rclcpp::FutureReturnCode::INTERRUPTED;
      }
    } while (0);
  }
  return rclcpp::FutureReturnCode::SUCCESS;
}

}  // namespace ros2_modbus_rtu
