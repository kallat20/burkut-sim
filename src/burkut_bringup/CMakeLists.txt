cmake_minimum_required(VERSION 3.8)
project(burkut_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config arayuz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  arayuz/arayuz.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arayuz
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
