# Minimum CMake version
cmake_minimum_required(VERSION 3.16)

# Project name and language
project(smcmpcquad LANGUAGES C CXX)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Compiler flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/qpoases
    ${CMAKE_SOURCE_DIR}/qpoases/INCLUDE
    ${CMAKE_SOURCE_DIR}/qpoases/SRC
    ${CMAKE_SOURCE_DIR}/include
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
include_directories(/usr/local/include/acado)

set(SOURCES
    ./qpoases/SRC/Bounds.cpp
    ./qpoases/SRC/Constraints.cpp
    ./qpoases/SRC/CyclingManager.cpp
    ./qpoases/SRC/Indexlist.cpp
    ./qpoases/SRC/MessageHandling.cpp
    ./qpoases/SRC/QProblem.cpp
    ./qpoases/SRC/QProblemB.cpp
    ./qpoases/SRC/SubjectTo.cpp
    ./qpoases/SRC/Utils.cpp
    ./qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
    ./include/acado_qpoases_interface.cpp
    ./include/acado_integrator.c
    ./include/acado_solver.c
    ./include/acado_auxiliary_functions.c
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(smcmpcquad ${SOURCES} src/smcmpcquad.cpp)
target_link_libraries(smcmpcquad m stdc++)
ament_target_dependencies(smcmpcquad rclcpp std_msgs tf2_ros nav_msgs)

add_executable(EKFnode src/EKFnode.cpp)
ament_target_dependencies(EKFnode rclcpp std_msgs tf2_ros gazebo_msgs nav_msgs)
target_include_directories(EKFnode PRIVATE ${EIGEN3_INCLUDE_DIR})

add_executable(getdata src/getdata.cpp)
ament_target_dependencies(getdata rclcpp std_msgs tf2_ros gazebo_msgs nav_msgs)

add_library(uavplugin SHARED src/uavplugin.cc)
ament_target_dependencies(uavplugin rclcpp std_msgs)
target_link_libraries(uavplugin ${GAZEBO_LIBRARIES} rclcpp::rclcpp)

# Install the node
install(TARGETS
    smcmpcquad
    uavplugin
    EKFnode
    getdata
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
