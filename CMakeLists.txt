cmake_minimum_required(VERSION 2.8.3)
project(lmpcc)

add_compile_options(-std=c++11)
add_compile_options(-fstack-protector)
set(CMAKE_CXX_FLAGS "-O3 -Wall -DCMAKE_BUILD_TYPE=RELEASE")

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
    lmpcc_msgs
    carla_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3)

add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
        cfg/PredictiveController.cfg
)


# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Control.msg
  control_feedback.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#  DIRECTORY
#     srv
#  FILES
#)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    nav_msgs
    shape_msgs
    visualization_msgs
    lmpcc_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs shape_msgs lmpcc_msgs
  DEPENDS Boost
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES  predictive_configuration predictive_controller
)

include_directories(include ${CMAKE_SOURCE_DIR}/FORCESNLPsolver/include)

### BUILD ###
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    include/${PROJECT_NAME}
    )


add_library(tkspline_lib
        src/spline.cpp
        )
add_dependencies(tkspline_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tkspline_lib ${CATKIN_LIBRARIES})

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
        src/Clothoid.cpp
        src/predictive_control_node.cpp
        )
add_dependencies(predictive_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lmpcc_node src/predictive_control_node.cpp FORCESNLPsolver/FORCESNLPsolver_model_1.c
        FORCESNLPsolver/FORCESNLPsolver_model_20.c
        FORCESNLPsolver/FORCESNLPsolver_casadi2forces.c)

target_link_libraries(lmpcc_node
        predictive_controller
        predictive_configuration
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        tkspline_lib
        ${CMAKE_SOURCE_DIR}/FORCESNLPsolver/lib/libFORCESNLPsolver.so
        )

add_dependencies(lmpcc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

### Test Case ####
install(
  DIRECTORY include/lmpcc/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/predictions_3d.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
