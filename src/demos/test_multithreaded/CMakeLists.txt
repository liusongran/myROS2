cmake_minimum_required(VERSION 3.5)
project(my_test)

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

add_executable(test_pub src/test_pub.cpp)
ament_target_dependencies(test_pub rclcpp std_msgs)

add_executable(test_sub0 src/test_sub0.cpp)
ament_target_dependencies(test_sub0 rclcpp std_msgs)

add_executable(test_sub1 src/test_sub1.cpp)
ament_target_dependencies(test_sub1 rclcpp std_msgs)

add_executable(test_sub2 src/test_sub2.cpp)
ament_target_dependencies(test_sub2 rclcpp std_msgs)

add_executable(test_sub3 src/test_sub3.cpp)
ament_target_dependencies(test_sub3 rclcpp std_msgs)

add_executable(test_sub4 src/test_sub4.cpp)
ament_target_dependencies(test_sub4 rclcpp std_msgs)

install(TARGETS
    test_pub
    test_sub0
    test_sub1
    test_sub2
    test_sub3
    test_sub4
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()