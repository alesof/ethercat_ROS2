cmake_minimum_required(VERSION 3.8)
project(ethercat_zeroerr)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethercat_driver REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(ethercat_generic_cia402_drive REQUIRED)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
