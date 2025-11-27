CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(dynamic_remap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(std_msgs REQUIRED)

# INCLUDE_DIRECTORIES(include)

ADD_EXECUTABLE(pub_remap
    src/pub_remap.cpp
)

AMENT_TARGET_DEPENDENCIES(pub_remap
    rclcpp
    std_msgs
)

INSTALL(TARGETS
    pub_remap
    DESTINATION lib/${PROJECT_NAME}
)

AMENT_PACKAGE()