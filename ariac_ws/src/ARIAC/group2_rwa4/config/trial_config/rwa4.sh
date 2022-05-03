#!/bin/bash

rosservice call /ariac/start_competition

# spawn a part in the tray on agv4
rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.1 -z 0.05 -R 3.14159 -P 0 -Y 1.57 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_pump_blue_10
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_battery_green_11
sleep 5
rosservice call /ariac/agv4/submit_shipment  "as3" "order_0_kitting_shipment_0" 


# spawn parts in the briefcase at as3
rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame station3::briefcase_3::briefcase_3::briefcase -model assembly_pump_blue_11
rosrun gazebo_ros spawn_model -sdf -x -0.032465 -y 0.174845 -z 0.15 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame station3::briefcase_3::briefcase_3::briefcase -model assembly_battery_green_12
sleep 5
rosservice call /ariac/as3/submit_shipment "order_1_assembly_shipment_0"

# spawn parts in the briefcase at as1
rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame station1::briefcase_1::briefcase_1::briefcase -model assembly_pump_blue_12
rosrun gazebo_ros spawn_model -sdf -x -0.032465 -y 0.174845 -z 0.15 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame station1::briefcase_1::briefcase_1::briefcase -model assembly_battery_green_13
sleep 5
rosservice call /ariac/as1/submit_shipment "order_1_assembly_shipment_1"
