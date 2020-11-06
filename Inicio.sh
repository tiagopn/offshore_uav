
rosservice call /uav1/control_manager/goto "goal: [-123120.783862, 1108535.74603, 12.72548772012, 0.0]"
sleep 20
rosservice call /uav1/control_manager/goto "goal: [-123120.783862, 1108535.74603, 1.72548772012, 0.0]"
sleep 20
rosservice call /uav1/control_manager/goto_relative "goal: [0, 0, 0, 1.61]"
sleep 20
rosservice call /uav1/control_manager/goto_relative "goal: [0, 0, 0, 1.61]"
sleep 20
rosservice call /uav1/control_manager/goto_relative "goal: [0, 0, 0, 1.61]"
sleep 20
   