cmake_minimum_required(VERSION 3.8)
project(project2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the launch directory to make launch files accessible
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

# Install the worlds directory for the .wbt files
install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds)

ament_package()

