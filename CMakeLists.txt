cmake_minimum_required(VERSION 3.8)
project(locomotion_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  ${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/locomotion_controller_node
    scripts/locomotion_controller_simulator
    scripts/totarget_navigation_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    README.md
    impedancepara.yaml
    impedancepara_default.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "nuc.env" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
