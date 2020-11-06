#!/bin/sh

for i in 1 2 3; do
    echo "counter: $i / 3"
    echo "wait 5 sec"
    sleep 5
    rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 0.0, 2.0, 0.0]"
done
 
echo "wait 5 sec"
sleep 5
rosservice call /uav1/control_manager/goto_relative "goal: [-1.0, -1.0, 0.0, 0.0]"


for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
    echo "counter: $i / 12"
    echo "wait 5 sec"
    sleep 5
    rosservice call /uav1/control_manager/goto_relative "goal: [-1.0, -1.0, 0.0, 0.0]"
done

