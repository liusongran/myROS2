cmake_minimum_required(VERSION 3.5)
project(test_interIntra)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(interIntra_pub src/interIntra_pub.cpp)
ament_target_dependencies(interIntra_pub rclcpp std_msgs)

add_executable(interIntra_sub0 src/interIntra_sub0.cpp)
ament_target_dependencies(interIntra_sub0 rclcpp std_msgs)

install(TARGETS
    interIntra_pub
    interIntra_sub0
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()