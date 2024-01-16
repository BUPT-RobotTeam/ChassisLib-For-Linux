cmake_minimum_required(VERSION 3.8)
project(chassis_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(BUILD_EXAMPLE OFF CACHE BOOL "Build example")
set(BUILD_TESTING ON CACHE BOOL "Build testing")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bupt_can REQUIRED)
find_package(motor_lib REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
)

add_library(base_chassis 
    STATIC 
    src/base_chassis.cpp
    include/chassis_lib/base_chassis.h
)
ament_target_dependencies(base_chassis rclcpp geometry_msgs)
target_include_directories(base_chassis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(base_chassis PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_library(omni_chassis 
    STATIC 
    src/omni_chassis.cpp
    include/chassis_lib/omni_chassis.h
)
target_link_libraries(omni_chassis PRIVATE base_chassis motor_lib)
target_include_directories(omni_chassis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(omni_chassis PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_library(steering_wheel
    STATIC 
    src/steering_wheel.cpp 
    include/chassis_lib/steering_wheel.h
)
target_link_libraries(steering_wheel PRIVATE bupt_can)
target_include_directories(steering_wheel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(steering_wheel PUBLIC c_std_99 cxx_std_17)

add_library(steering_chassis
    STATIC   
    src/steering_chassis.cpp
    include/chassis_lib/steering_chassis.h 
)
ament_target_dependencies(steering_chassis rclcpp geometry_msgs)
target_link_libraries(steering_chassis base_chassis steering_wheel)
target_include_directories(steering_chassis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(steering_chassis PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

if (BUILD_EXAMPLE)
    add_executable(chassis_example example/chassis_example.cpp)
    ament_target_dependencies(chassis_example PUBLIC rclcpp PUBLIC geometry_msgs)
    target_link_libraries(chassis_example PRIVATE omni_chassis steering_chassis bupt_can)
    target_compile_features(chassis_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS chassis_example
        DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(chassis_node src/chassis_node.cpp)
ament_target_dependencies(chassis_node PUBLIC rclcpp PUBLIC geometry_msgs)
target_link_libraries(chassis_node PRIVATE omni_chassis steering_chassis bupt_can)
target_include_directories(chassis_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS chassis_node 
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

if (BUILD_TESTING)
    add_executable(SW_TEST test/SW_test.cpp)
    target_link_libraries(SW_TEST PRIVATE bupt_can steering_wheel)
    target_include_directories(SW_TEST PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    target_compile_features(SW_TEST PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS SW_TEST 
        DESTINATION lib/${PROJECT_NAME})

    add_executable(SW_CHASSIS_TEST test/SWChassis_test.cpp)
    ament_target_dependencies(SW_CHASSIS_TEST PUBLIC rclcpp)
    target_link_libraries(SW_CHASSIS_TEST PRIVATE bupt_can steering_wheel steering_chassis)
    target_include_directories(SW_CHASSIS_TEST PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    target_compile_features(SW_CHASSIS_TEST PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS SW_CHASSIS_TEST 
        DESTINATION lib/${PROJECT_NAME})
    
    add_executable(cmdvel_sender test/cmdvel_sender.cpp)
    ament_target_dependencies(cmdvel_sender PUBLIC rclcpp geometry_msgs)
    target_include_directories(cmdvel_sender PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    target_compile_features(cmdvel_sender PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS cmdvel_sender 
        DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
