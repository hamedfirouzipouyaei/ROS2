cmake_minimum_required(VERSION 3.8)
project(cpp_template_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(EXECUTABLE_NAME cpp_template_node)
add_executable(${EXECUTABLE_NAME} src/template_node.cpp)
ament_target_dependencies(${EXECUTABLE_NAME} rclcpp)  

# Install the executable
install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
