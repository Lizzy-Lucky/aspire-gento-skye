cmake_minimum_required(VERSION 3.22)
project(gento_skye_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch vendor
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
