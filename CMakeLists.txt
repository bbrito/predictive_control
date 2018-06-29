cmake_minimum_required(VERSION 2.8.3)
project(predictive_control)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    cmake_modules
    control_msgs
    cob_srvs
    eigen_conversions
    geometry_msgs
    kdl_conversions
    kdl_parser
    moveit_core
    nav_msgs
    roscpp
    roslint
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    trajectory_msgs
    urdf
    visualization_msgs
    shape_msgs
    moveit_msgs
    obstacle_feed
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

find_package(ACADO REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

## Generate messages in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    traj.action
    move.action
)

## Generate messages in the 'msg' folder
#add_message_files(
#  DIRECTORY
#    msg
#  FILES
#)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
     srv
  FILES
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
            actionlib_msgs
            geometry_msgs
            sensor_msgs
            std_msgs
            shape_msgs
            visualization_msgs
            moveit_msgs
            obstacle_feed
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs shape_msgs moveit_core obstacle_feed
  DEPENDS Boost ACADO
  INCLUDE_DIRS include ${ACADO_INCLUDE_DIRS} include/${PROJECT_NAME}
  LIBRARIES  predictive_configuration kinematic_calculations self_collision_detection collision_avoidance predictive_trajectory_generator predictive_controller
)

### BUILD ###
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${ACADO_INCLUDE_DIRS}
    include/${PROJECT_NAME}
    #${ACADO_INCLUDE_PACKAGES}
    )

FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)

add_library(predictive_configuration src/predictive_configuration.cpp)
add_dependencies(predictive_configuration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_configuration
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${libacado}
    ${CASADI_LIBRARIES}
    )

add_library(predictive_trajectory_generator src/predictive_trajectory_generator.cpp)
add_dependencies(predictive_trajectory_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_trajectory_generator
    predictive_configuration
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${libacado}
    )

add_library(predictive_controller src/mpcc_controller.cpp)
add_dependencies(predictive_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_controller
    predictive_configuration
    predictive_trajectory_generator
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${libacado}
    )

add_executable(predictive_control_node src/predictive_control_node.cpp)
add_dependencies(predictive_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_control_node
    predictive_controller
    ${catkin_LIBRARIES}
    )

### Test Case ####

install(
  DIRECTORY include/predictive_control/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
        predictive_trajectory_generator
        predictive_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)