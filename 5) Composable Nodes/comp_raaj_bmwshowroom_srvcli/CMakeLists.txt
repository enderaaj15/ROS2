cmake_minimum_required(VERSION 3.8)
project(comp_raaj_bmwshowroom_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(car_server_comp SHARED src/bmw_server.cpp)
ament_target_dependencies(car_server_comp rclcpp rcl_interfaces std_srvs std_msgs rclcpp_components)
rclcpp_components_register_node(car_server_comp PLUGIN "comp_raaj_bmwshowroom_srvcli::BuyCarServer" EXECUTABLE car_server)

add_library(car_client_comp SHARED src/bmw_client.cpp)
ament_target_dependencies(car_client_comp rclcpp rcl_interfaces std_srvs std_msgs rclcpp_components)
rclcpp_components_register_node(car_client_comp PLUGIN "comp_raaj_bmwshowroom_srvcli::BuyCarClient" EXECUTABLE car_client)


install(TARGETS
  car_server_comp car_client_comp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


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
