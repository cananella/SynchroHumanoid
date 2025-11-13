cmake_minimum_required(VERSION 3.8)
project(openarm_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
    ${PROJECT_NAME}/servo_monitor.py
    ${PROJECT_NAME}/keyboard_servo_right.py
    ${PROJECT_NAME}/keyboard_servo_left.py
  DESTINATION lib/${PROJECT_NAME}
)

########## launch, config 디렉토리 설치 ##########
# ros2 launch 에서 찾을 수 있게 share/${PROJECT_NAME}로
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
