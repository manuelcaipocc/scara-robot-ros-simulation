cmake_minimum_required(VERSION 3.8)
project(scara_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Instalar carpeta launch/
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos URDF/XACRO
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Instalar archivos RViz
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# (Opcional) Instalar scripts si tienes archivos Python
# install(PROGRAMS
#   scripts/mi_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
