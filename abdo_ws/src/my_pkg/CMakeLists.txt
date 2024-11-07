cmake_minimum_required(VERSION 3.8)
project(my_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(moveit_ros_move_group REQUIRED)

install(PROGRAMS
  scripts/calibration/calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/calibration/sub_realsense.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/anygrasp_bridging/anygrasp_application.py
  DESTINATION lib/${PROJECT_NAME}
)


project(my_pkg)

# Find dependencies
find_package(rclcpp REQUIRED)


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
