cmake_minimum_required(VERSION 3.16)

project(smcnmpccube LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

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

add_executable(smcnmpccube ${SOURCES} src/smcnmpccube.cpp)
target_link_libraries(smcnmpccube m stdc++)
ament_target_dependencies(smcnmpccube rclcpp std_msgs tf2_ros nav_msgs)

add_library(cubeplugin SHARED src/cubeplugin.cc)
ament_target_dependencies(cubeplugin rclcpp std_msgs)
target_link_libraries(cubeplugin ${GAZEBO_LIBRARIES} rclcpp::rclcpp)

install(TARGETS
    smcnmpccube
    cubeplugin
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
