CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(bridge_ex)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(std_msgs REQUIRED)

# INCLUDE_DIRECTORIES(include)

ADD_EXECUTABLE(bridge
    src/bridge_test.cpp
)

AMENT_TARGET_DEPENDENCIES(bridge
    rclcpp
    std_msgs
)

INSTALL(TARGETS
    bridge
    DESTINATION lib/${PROJECT_NAME}
)

ADD_EXECUTABLE(bridge_des
    src/bridge_des.cpp
)

AMENT_TARGET_DEPENDENCIES(bridge_des
    rclcpp
    std_msgs
)

INSTALL(TARGETS
    bridge_des
    DESTINATION lib/${PROJECT_NAME}
)

AMENT_PACKAGE()