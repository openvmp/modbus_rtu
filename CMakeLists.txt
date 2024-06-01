cmake_minimum_required(VERSION 3.10)
project(remote_modbus_rtu)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-fPIC)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(remote_modbus REQUIRED)
find_package(remote_serial REQUIRED)

# Logic shared by all targets
include_directories(
  include
  ${remote_serial_INCLUDE_DIRS}
  ${remote_modbus_INCLUDE_DIRS}
)

set(project_SOURCE_FILES
  src/node.cpp
  src/implementation.cpp
  src/proto_holding_registers.cpp
  src/proto_misc.cpp
  src/coil_read_write.cpp
)

set(project_LIBRARIES
  ${rclcpp_LIBRARIES}
  ${remote_serial_LIBRARIES}
  ${remote_serial_native_LIBRARIES}
  ${remote_modbus_LIBRARIES}
)

set(project_DEPENDENCIES
  rclcpp
  std_msgs
  remote_serial
  remote_modbus
)

# Executable target
add_executable(${PROJECT_NAME}_standalone src/main.cpp ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_standalone ${project_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_standalone ${project_DEPENDENCIES})
install(TARGETS
  ${PROJECT_NAME}_standalone
  DESTINATION lib/${PROJECT_NAME}
)

# Library target
add_library(${PROJECT_NAME}_native src/factory.cpp ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_native ${project_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_native ${project_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_native_library HAS_LIBRARY_TARGET)
ament_export_dependencies(${project_DEPENDENCIES} ${remote_modbus_DEPENDENCIES})
ament_export_include_directories(include)
install(TARGETS ${PROJECT_NAME}_native
  EXPORT ${PROJECT_NAME}_native_library
  DESTINATION lib
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
