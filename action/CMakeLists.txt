cmake_minimum_required(VERSION 3.8)
project(action)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/action_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/polynomial_trajectory.py
  scripts/rigid_transform.py
  scripts/vision_pose_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
