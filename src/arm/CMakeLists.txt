cmake_minimum_required(VERSION 3.8)
project(arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config meshes rviz models worlds sdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/twist_to_arm.py
  scripts/combined_ik.py
  DESTINATION lib/${PROJECT_NAME}
)






ament_package()
