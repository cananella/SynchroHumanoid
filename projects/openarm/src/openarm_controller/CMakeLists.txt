cmake_minimum_required(VERSION 3.8)
project(openarm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(moveit_servo_test src/moveit_servo_test.cpp)
ament_target_dependencies(moveit_servo_test rclcpp std_msgs moveit_servo)

install(
  TARGETS moveit_servo_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    ${PROJECT_NAME}/action_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

########## include 디렉토리 설치 ##########
install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()
