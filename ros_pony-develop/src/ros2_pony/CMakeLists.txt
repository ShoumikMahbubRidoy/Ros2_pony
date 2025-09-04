cmake_minimum_required(VERSION 3.8)
project(ros2_pony)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate the message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "ros2_pony/msg/MotorFeedback.msg"
  "ros2_pony/msg/SystemStatus.msg"
  "ros2_pony/msg/InitializationStatus.msg"
  "ros2_pony/msg/AimDegree.msg"
  "ros2_pony/msg/MotorFeedbackArray.msg"
  "ros2_pony/msg/JointNames.msg"
  DEPENDENCIES can_msgs sensor_msgs std_srvs
)

# Install msg files
install(DIRECTORY ros2_pony/msg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 