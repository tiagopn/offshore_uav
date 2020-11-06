
for i in 1 2 3 4 5 6 7; do
    echo "counter: $i / 7 => -1m"
    echo "wait 5 sec"
    sleep 5
    echo "wait 5 sec"
    sleep 5
    rosservice call /uav1/control_manager/goto_relative "goal: [0.0, -1.0, 0.0, 0.0]"
done

echo "wait 5 sec"
sleep 5
echo "wait 5 sec"
sleep 5


_______________________________________________

rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 0.0, 5.0, 0.0]"
echo "wait 5 sec"
sleep 5


rosservice call /uav1/odometry/change_alt_estimator_type_string "value: BARO"
echo "wait 5 sec"
sleep 5


for i in 1 2 3; do
    rosservice call /uav1/control_manager/goto_relative "goal: [2.0, 0.0, 0.0, 0.0]"
    echo "wait 5 sec"
    sleep 5
done

rosservice call /uav1/control_manager/goto_relative "goal: [0.0, -10.0, 0.0, 0.0]"
echo "wait 5 sec"
sleep 5


rosservice call /uav1/control_manager/goto_relative "goal: [0.0, -5.0, 0.0, 0.0]"
echo "wait 5 sec"
sleep 5

rosservice call /uav1/odometry/change_alt_estimator_type_string "value: HEIGHT"
echo "wait 5 sec"
sleep 5
