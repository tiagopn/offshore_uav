#!/bin/sh


rosservice call /uav1/odometry/change_alt_estimator_type_string "value: baro"
echo "wait 5 sec"
sleep 5

rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 0.0, 5.0, 0.0]"
echo "wait 5 sec"
sleep 5

for i in 1 2 3; do
    echo ""
    echo "counter: $i / 3 => -1m"
    rosservice call /uav1/control_manager/goto_relative "goal: [0.0, -10.0, 0.0, 0.0]"
    echo "wait 5 sec"
    sleep 5
#rosservice call /uav1/control_manager/goto_relative "goal: [-1.0, 0.0, 0.0, 0.0]"
#rosservice call /uav1/control_manager/goto_relative "goal: [-1.0, -1.0, 0.0, 0.0]"
done

echo "first boat"
rosservice call /uav1/control_manager/goto "goal: [-123136.570269, 1108437.85305, 0.630100856701, -1.62]"
echo "wait 15 sec"
echo "wait 5 sec"
sleep 5
echo "wait 5 sec"
sleep 5
echo "wait 5 sec"
sleep 5
##
##rosservice call /uav1/odometry/change_alt_estimator_type_string "value: HEIGHT"
##echo "wait 5 sec"
##sleep 5
##
##echo "preparando para pousar"
##
##for i in 1 2; do
##    rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 0.0, -4.0, 0.0]"
##    echo "wait 5 sec"
##    sleep 5
##done 
##
##for i in 1 2 3 4 5 6 7 8; do
##    rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 0.0, -0.2, 0.0]"
##    echo "wait 5 sec"
##    sleep 1
##done
##
#rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 0.0, 13.0, 0.0]"

rosservice call /uav1/odometry/change_alt_estimator_type_string "value: baro"


rosservice call /uav1/control_manager/goto "goal: [-123129.739198, 1108420.15897, 0.630100856701, -1.62]"



second_boat_position:
      Hori: GPS
      Vert: BARO
      x: -123129.739198
      y: 1108420.15897
      z: 0.630100856701
 
offshore_landing_position: 
      x: -123144.056766
      y: 1108424.88005
      z: 6.12495578052

Apartir do ponto pouso da offshore_landing_position, o drone vai:
rosservice call /uav1/control_manager/goto_relative "goal: [-40.0, 0.0, 0.0, 0.0]"
rosservice call /uav1/control_manager/goto_relative "goal: [0.0, -40.0, 0.0, 0.0]"
rosservice call /uav1/control_manager/goto_relative "goal: [40.0, 0.0, 0.0, 0.0]"
rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 40.0, 0.0, 0.0]"
