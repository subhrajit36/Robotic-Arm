cmake_minimum_required(VERSION 3.5)
project(final_final_design_for_urdf)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install URDF, XACRO, and other configuration files
install(DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

# Finalize package setup
ament_package()
