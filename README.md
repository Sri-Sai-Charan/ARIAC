# Wiki | Home


## ARIAC 2021.

![ariac-2021](wiki/figures/ariac2021_environment.jpeg)

```diff
- NOTE: Always use the master branch to retrieve the latest updates.
```

```diff
- NOTE: Always review the Updates section when new code is pushed.
```
```diff
- NOTE: 'ariac_ws' is a catkin workspace that has all required packages and tools installed
```
<!---<img src="wiki/figures/ariac2020_3.jpg" alt="alt text" width="600" class="center">-->

## [Installation](wiki/tutorials/installation.md)

- Steps to install and run ARIAC.

## [Terminology](wiki/misc/terminology.md)

- This section describes the terminology used in this wiki. If you are new to ARIAC it is strongly suggested that you visit this page first.

## [Documentation](wiki/documentation/documentation.md)

- Specifications of the NIST Agile Robotics for Industrial Automation Competition (ARIAC) and the Gazebo Environment for Agile Robotics (GEAR) software.

## [Tutorials](wiki/tutorials/tutorials.md)

- A set of tutorials to help you get started with the NIST Agile Robotics for Industrial Automation Competition (ARIAC).
## Folder Structure:

```
π¦ARIAC
 β£ πariac_ws
 β β£ π.catkin_tools
 β β πsrc
 β β β£ πARIAC
 β β β β£ πariac
 β β β β£ πariac_example
 β β β β£ πgantry_moveit_config
 β β β β£ πgroup2_rwa4
 β β β β β£ πconfig
 β β β β β β£ πtrial_config
 β β β β β β β£ πrwa2_trial.yaml
 β β β β β β β£ πrwa3_trial.yaml
 β β β β β β β£ πrwa4.sh
 β β β β β β β πrwa4.yaml
 β β β β β β πuser_config
 β β β β β β β πgroup1_config.yaml
 β β β β β£ πinclude
 β β β β β β£ πagv.h
 β β β β β β£ πassembly_handler.h
 β β β β β β£ πbins_handler.h
 β β β β β β£ πconveyor.h
 β β β β β β£ πgantry_robot.h
 β β β β β β£ πkitting_handler.h
 β β β β β β£ πkitting_robot.h
 β β β β β β£ πmain_handler.h
 β β β β β β£ πorder_handler.h
 β β β β β β£ πpart_2.h
 β β β β β β£ πsensor_array.h
 β β β β β β πutils.h
 β β β β β£ πlaunch
 β β β β β β πariac.launch
 β β β β β£ πmsg
 β β β β β β£ πAgvKittingShipmentDetails.msg
 β β β β β β£ πAssemblyShipment.msg
 β β β β β β£ πFaultyPartPose.msg
 β β β β β β£ πIncompleteOrder.msg
 β β β β β β£ πKittingLocation.msg
 β β β β β β£ πKittingShipment.msg
 β β β β β β£ πOrder.msg
 β β β β β β£ πProduct.msg
 β β β β β β πTask.msg
 β β β β β£ πparam
 β β β β β β πflags.yaml
 β β β β β£ πscript
 β β β β β β πpart_spawner.sh
 β β β β β£ πsrc
 β β β β β β£ πnodes
 β β β β β β β£ πagv_handler_node.cpp
 β β β β β β β£ πassembly_handler_node.cpp
 β β β β β β β£ πbins_handler_node.cpp
 β β β β β β β£ πconveyor_handler_node.cpp
 β β β β β β β£ πmain_handler_node.cpp
 β β β β β β β£ πorder_handler_node.cpp
 β β β β β β β£ πrwa4_node.cpp
 β β β β β β β πsensor_array_node.cpp
 β β β β β β£ πagv.cpp
 β β β β β β£ πassembly_handler.cpp
 β β β β β β£ πbins_handler.cpp
 β β β β β β£ πconveyor.cpp
 β β β β β β£ πgantry_robot.cpp
 β β β β β β£ πkitting_handler.cpp
 β β β β β β£ πkitting_robot.cpp
 β β β β β β£ πmain_handler.cpp
 β β β β β β£ πorder_handler.cpp
 β β β β β β£ πpart_2.cpp
 β β β β β β£ πsensor_array.cpp
 β β β β β β πutils.cpp
 β β β β β£ πsrv
 β β β β β β£ πagv_location.srv
 β β β β β β£ πassembly_part_details.srv
 β β β β β β£ πassembly_task.srv
 β β β β β β£ πcheck_agv_faulty_parts.srv
 β β β β β β£ πcheck_exists.srv
 β β β β β β£ πcheck_part_pose.srv
 β β β β β β£ πdispose_faulty_part.srv
 β β β β β β£ πkitting_part_details.srv
 β β β β β β£ πlist_all_parts.srv
 β β β β β β£ πlist_order_details.srv
 β β β β β β£ πorder_assembly_shipment_details.srv
 β β β β β β£ πorder_completion_status.srv
 β β β β β β£ πorder_kitting_shipment_details.srv
 β β β β β β πorder_state.srv
 β β β β β£ πCMakeLists.txt
 β β β β β πpackage.xml
 β β β β£ πkitting_moveit_config
 β β β β£ πmotion_control
 β β β β£ πnist_gear
 β β β β πtest_ariac
 β β β πariac-gazebo_ros_pkgs
 β£ πwiki
 β β£ πdocumentation
 β β£ πfigures
 β β£ πfinals
 β β£ πmisc
 β β£ πqualifiers
 β β πtutorials
 β πREADME.md
```