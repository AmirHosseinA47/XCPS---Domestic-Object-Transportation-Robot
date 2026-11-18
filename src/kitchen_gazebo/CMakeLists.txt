cmake_minimum_required(VERSION 3.16)
project(kitchen_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds config urdf rviz maps models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/make_gazebo_urdfs.py
  scripts/make_kitchen_map.py
  scripts/make_nav2_params.py
  scripts/nav_test.py
  scripts/make_bridge_config.py
  scripts/pr2_ik.py
  scripts/kitchen_skills.py
  scripts/skills_demo.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
