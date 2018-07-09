cmake_minimum_required(VERSION 2.8.3)
project(predictive_control)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    cmake_modules
    control_msgs
    dynamic_reconfigure
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
find_package(Eigen3 REQUIRED)

find_package(ACADO REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
        cfg/PredictiveController.cfg
)

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
  LIBRARIES  predictive_configuration predictive_trajectory_generator predictive_controller
)

### BUILD ###
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${ACADO_INCLUDE_DIRS}
    include/${PROJECT_NAME}
    include/${PROJECT_NAME}/generated_mpc
    qpoases/INCLUDE
    qpoases/SRC
    )

FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)

add_library(predictive_configuration src/predictive_configuration.cpp)
add_dependencies(predictive_configuration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_configuration
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${CASADI_LIBRARIES}
    )

add_library(predictive_controller
        src/mpcc_controller.cpp
        src/predictive_configuration.cpp
        qpoases/SRC/Bounds.cpp
        qpoases/SRC/Constraints.cpp
        qpoases/SRC/CyclingManager.cpp
        qpoases/SRC/Indexlist.cpp
        qpoases/SRC/MessageHandling.cpp
        qpoases/SRC/QProblem.cpp
        qpoases/SRC/QProblemB.cpp
        qpoases/SRC/SubjectTo.cpp
        qpoases/SRC/Utils.cpp
        qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
        src/generated_mpc/acado_qpoases_interface.cpp
        src/generated_mpc/acado_hessian_regularization.c
        src/generated_mpc/acado_integrator.c
        src/generated_mpc/acado_solver.c
        src/generated_mpc/acado_auxiliary_functions.c)

target_include_directories(predictive_controller PUBLIC
                qpoases
                qpoases/INCLUDE
                qpoases/SRC)

add_executable(predictive_control_node src/predictive_control_node.cpp)
target_link_libraries(predictive_control_node
                    predictive_controller
                    ${catkin_LIBRARIES})

add_dependencies(predictive_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

### Test Case ####

install(
  DIRECTORY include/predictive_control/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
        predictive_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
