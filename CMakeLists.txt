cmake_minimum_required(VERSION 3.8)
project(chassis_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(BUILD_EXAMPLE OFF CACHE BOOL "Build example")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bupt_can REQUIRED)
find_package(motor_lib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
)

add_library(utils 
    STATIC
    include/utils/utils.h
)
set_target_properties(utils PROPERTIES LINKER_LANGUAGE CXX)
target_compile_features(utils PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

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
target_link_libraries(omni_chassis PRIVATE utils base_chassis)
target_include_directories(omni_chassis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(omni_chassis PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

if (BUILD_EXAMPLE)
    add_executable(omni_chassis_example src/omni_chassis_example.cpp)
    target_link_libraries(omni_chassis_example PRIVATE omni_chassis)
    target_compile_features(omni_chassis_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS omni_chassis_example
        DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(chassis_node src/chassis_node.cpp)
target_include_directories(chassis_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS chassis_node 
  DESTINATION lib/${PROJECT_NAME})

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
