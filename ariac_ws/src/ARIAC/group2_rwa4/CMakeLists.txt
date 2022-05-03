cmake_minimum_required(VERSION 3.0.2)
project(group2_rwa4)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nist_gear
  roscpp
  rospy
  sensor_msgs
  std_srvs
  trajectory_msgs
  message_generation
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools

)

find_package(Boost REQUIRED system filesystem date_time thread)


add_message_files(
  FILES
  Order.msg
  KittingShipment.msg
  AssemblyShipment.msg
  Product.msg
  IncompleteOrder.msg  
  KittingLocation.msg
  FaultyPartPose.msg
  AgvKittingShipmentDetails.msg
  Task.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  assembly_task.srv
  check_exists.srv
  check_agv_faulty_parts.srv
  agv_location.srv
  list_order_details.srv
  list_all_parts.srv
  check_part_pose.srv  
  order_kitting_shipment_details.srv
  kitting_part_details.srv
  dispose_faulty_part.srv
  order_completion_status.srv
  assembly_part_details.srv
  order_assembly_shipment_details.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES group2_rwa1
#  CATKIN_DEPENDS nist_gear roscpp rospy sensor_msgs std_srvs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(main_handler_node src/nodes/main_handler_node.cpp src/main_handler.cpp)
# add_executable(agv_handler_node src/nodes/agv_handler_node.cpp src/agv.cpp)
add_executable(bins_handler_node src/nodes/bins_handler_node.cpp src/bins_handler.cpp)
add_executable(order_handler_node src/nodes/order_handler_node.cpp src/order_handler.cpp)
# add_executable(assembly_handler_node src/nodes/assembly_handler_node.cpp src/assembly_handler.cpp)
add_executable(sensor_array_node src/nodes/sensor_array_node.cpp src/sensor_array.cpp)

add_executable(rwa4_node 
                    src/nodes/rwa4_node.cpp 
                    src/main_handler.cpp 
                    src/order_handler.cpp 
                    src/bins_handler.cpp 
                    src/agv.cpp 
                    src/kitting_handler.cpp 
                    src/kitting_robot.cpp 
                    src/conveyor.cpp 
                    src/gantry_robot.cpp 
                    src/part_2.cpp 
                    src/utils.cpp 
                    src/agv.cpp 
                    src/assembly_handler.cpp)

add_dependencies(bins_handler_node group2_rwa4_gencpp)
# add_dependencies(agv_handler_node group2_rwa4_gencpp)
add_dependencies(order_handler_node group2_rwa4_gencpp)
add_dependencies(sensor_array_node group2_rwa4_gencpp)
add_dependencies(rwa4_node group2_rwa4_gencpp)
# add_dependencies(kitting_handler group2_rwa4_gencpp)


target_link_libraries(main_handler_node
  ${catkin_LIBRARIES}
)
# target_link_libraries(conveyor_handler_node
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(agv_handler_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(bins_handler_node
  ${catkin_LIBRARIES}
)
target_link_libraries(order_handler_node
  ${catkin_LIBRARIES}
)
# target_link_libraries(assembly_handler_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(rwa4_node
${catkin_LIBRARIES}
)

target_link_libraries(sensor_array_node
${catkin_LIBRARIES}
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
  # PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_group2_rwa1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
