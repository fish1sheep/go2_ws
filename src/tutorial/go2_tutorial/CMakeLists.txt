cmake_minimum_required(VERSION 3.8)
project(go2_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(go2_tutorial_inter REQUIRED)

# go2_ctrl
add_executable(go2_ctrl src/go2_ctrl.cpp)
target_compile_features(go2_ctrl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  go2_ctrl
  "rclcpp"
  "unitree_go"
  "unitree_api"
)

install(TARGETS go2_ctrl
  DESTINATION lib/${PROJECT_NAME})

# go2_state
add_executable(go2_state src/go2_state.cpp)

ament_target_dependencies(
  go2_state
  "rclcpp"
  "nav_msgs"
)

# go2_cruising_service go2_cruising_client
add_executable(go2_cruising_service src/go2_cruising_service.cpp)
add_executable(go2_cruising_client src/go2_cruising_client.cpp)

ament_target_dependencies(
  go2_cruising_service
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
  "geometry_msgs"
)

ament_target_dependencies(
  go2_cruising_client
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
)

install(TARGETS
  go2_cruising_service
  go2_cruising_client
  DESTINATION lib/${PROJECT_NAME})


target_include_directories(go2_ctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(go2_cruising_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(go2_cruising_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
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
