cmake_minimum_required(VERSION 2.8.3)
project(predictive_control)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS cmake_modules cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs roscpp roslint sensor_msgs std_msgs tf tf_conversions trajectory_msgs urdf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

find_package(Ceres REQUIRED)
find_package(ACADO REQUIRED)
message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

SET(ACADO_DIR /home/bfb-ws/ACADOtoolkit/)
SET(ACADO_INCLUDE_PACKAGES ${ACADO_DIR} ${ACADO_DIR}/acado ${ACADO_DIR}/external_packages)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY 
		srvs
	FILES
		GetFrameTrackingInfo.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		actionlib_msgs
		geometry_msgs
		sensor_msgs
		std_msgs
)

catkin_package(
  CATKIN_DEPENDS cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs
  DEPENDS Boost CERES ACADO
  INCLUDE_DIRS include ${ACADO_INCLUDE_PACKAGES}
  LIBRARIES  kinematic_calculations predictive_trajectory_generator predictive_controller 
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${ACADO_INCLUDE_PACKAGES})

FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)

add_library(kinematic_calculations src/kinematic_calculations.cpp)
add_dependencies(kinematic_calculations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic_calculations ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${CERES_LIBRARIES} ${libacado}) 

add_library(predictive_trajectory_generator src/predictive_trajectory_generator.cpp)
add_dependencies(predictive_trajectory_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_trajectory_generator ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${CERES_LIBRARIES} ${libacado})

add_library(predictive_controller src/predictive_controller.cpp)
add_dependencies(predictive_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_controller kinematic_calculations  predictive_trajectory_generator ${catkin_LIBRARIES})

add_executable(predictive_control_node src/predictive_control_node.cpp)
add_dependencies(predictive_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_control_node predictive_controller ${catkin_LIBRARIES})

### Test Case ####
add_executable(kinematic_calculations_test test/kinematic_calculations_test.cpp)
add_dependencies(kinematic_calculations_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic_calculations_test kinematic_calculations ${catkin_LIBRARIES} )

add_executable(predictive_config_test test/predictive_config_test.cpp)
add_dependencies(predictive_config_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_config_test predictive_trajectory_generator ${catkin_LIBRARIES} )

add_executable(predictive_controller_test test/predictive_controller_test.cpp)
add_dependencies(predictive_controller_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(predictive_controller_test predictive_controller ${catkin_LIBRARIES} )

### INSTALL ###
install(TARGETS kinematic_calculations predictive_trajectory_generator predictive_controller
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
