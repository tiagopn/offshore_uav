#include <OffshoreUAV.h>
#include <pluginlib/class_list_macros.h>

namespace offshore_uav
{

  /* onInit() //{ */

  void OffshoreUAV::onInit()
  {
    /* set flags to false */
    is_initialized_ = false;
    hover_mode_ = false;
    swarming_mode_ = false;
    state_machine_running_ = false;

    ros::NodeHandle nh("~");

    ros::Time::waitForValid();

    mrs_lib::ParamLoader param_loader(nh, "Offshore");

    /* load parameters */
    param_loader.loadParam("uav_name", _uav_name_);
    //param_loader.loadParam("frame", _frame_);
    //param_loader.loadParam("desired_height", _desired_height_);
    //param_loader.loadParam("land_at_the_end", _land_end_);
    //
    //param_loader.loadParam("flocking/auto_start", _auto_start_);
    //param_loader.loadParam("flocking/swarming_after_hover", _timeout_state_change_);
    //param_loader.loadParam("flocking/duration", _timeout_flocking_);
    //
    ///* load proximal control parameters */
    //param_loader.loadParam("flocking/proximal/desired_distance", _desired_distance_);
    //param_loader.loadParam("flocking/proximal/strength_potential", _strength_potential_);
    //param_loader.loadParam("flocking/proximal/steepness_potential", _steepness_potential_);
    //param_loader.loadParam("flocking/proximal/range_multiplier", _range_multipler_);
    //
    ///* load motion control parameters */
    //param_loader.loadParam("flocking/motion/K1", _K1_);
    //param_loader.loadParam("flocking/motion/K2", _K2_);
    //param_loader.loadParam("flocking/motion/move_forward", _move_forward_);
    //param_loader.loadParam("flocking/motion/interpolate_coeff", _interpolate_coeff_);
    //param_loader.loadParam("flocking/motion/fixed_heading", _fixed_heading_);

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("[Offshore]: failed to load non-optional parameters!");
      ros::shutdown();
    }

    //
    ///* set remaining parameters using loaded ones */
    //noise_     = _desired_distance_ / pow(2, 1 / _steepness_potential_);
    //max_range_ = _range_multipler_  * _desired_distance_;
    //
    ///* get current heading */
    //nav_msgs::Odometry::ConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/" + _uav_name_ + "/odometry/odom_main", ros::Duration(15));
    //
    ///* set smooth or virtual heading */
    //if (_fixed_heading_) {
    //  initial_heading_ = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();
    //  virtual_heading_ = initial_heading_;
    //
    //  pub_virtual_heading_ = nh.advertise<mrs_msgs::Float64Stamped>("/" + _uav_name_ + "/flocking/virtual_heading", 1);
    //
    //} else {
    //  smooth_heading_ = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();
    //}
    //
    ///* publishers */
    //pub_mode_changed_ = nh.advertise<flocking::ModeStamped>("/" + _uav_name_ + "/flocking/mode_changed", 1);
    //
    ///* message filters */
    //sub_neighbors_info_.subscribe(nh, "/" + _uav_name_ + "/sensor_neighbor/neighbors", 1);
    //sub_odom_.subscribe(nh, "/" + _uav_name_ + "/odometry/odom_main", 1);
    //sync_.reset(new Sync(OffshoreUAVPolicy(10), sub_neighbors_info_, sub_odom_));
    //sync_->registerCallback(boost::bind(&OffshoreUAV::callbackUAVNeighbors, this, _1, _2));
    //
    //
    ///* service servers */
    //srv_server_state_machine_ = nh.advertiseService("start_state_machine_in", &OffshoreUAV::callbackStartStateMachine, this);
    //srv_server_hover_mode_    = nh.advertiseService("start_hover_mode", &OffshoreUAV::callbackStartHoverMode, this);
    //srv_server_swarming_mode_ = nh.advertiseService("start_swarming_mode", &OffshoreUAV::callbackStartSwarmingMode, this);
    //srv_server_close_node_    = nh.advertiseService("close_node", &OffshoreUAV::callbackCloseNode, this);
    //
    
    //

    // | -------------- initialize serviceClients ----------------- |
    ///* service client */
    // Correct but not working
    srv_client_goto_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + _uav_name_ + "/control_manager/reference");
    
    // Wrong but still work
    srv_client_goto_relative_ = nh.serviceClient<mrs_msgs::Vec4>("/uav1/control_manager/goto_relative");
    srv_client_goto_global_ = nh.serviceClient<mrs_msgs::Vec4>("/uav1/control_manager/goto");

    //rosservice call /uav1/odometry/change_alt_estimator_type_string "value: baro"
    srv_client_change_alt_estimator_ = nh.serviceClient<mrs_msgs::String>("/uav1/odometry/change_alt_estimator_type_string");
    
    //srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("/" + _uav_name_ + "/uav_manager/land");

    ///* timers */
    timer_state_machine_ = nh.createTimer(ros::Duration(0.1), &OffshoreUAV::callbackTimerStateMachine, this, true, true);

    ROS_INFO_ONCE("[OffshoreUAV]: initialized");

    is_initialized_ = true;

    ros::spin();
  }

  //}

  // | ----------------------- odometry functions ----------------------- |

  void OffshoreUAV::gotoRelative(mrs_msgs::Vec4 srv)
  {
    // mrs_msgs::Vec4 srv;
    if (srv_client_goto_relative_.call(srv))
    {
      // ROS_INFO("Moving to [%d,%d,%d]...", srv.request.goal.at(0),
      //          srv.request.goal.at(1), srv.request.goal.at(2));
    }
    else
    {
      ROS_ERROR("[Offshore] Failed to call service \"srv_client_goto_relative_\"");
    }
  }

  void OffshoreUAV::gotoGlobal(mrs_msgs::Vec4 srv)
  {
    // mrs_msgs::Vec4 srv;
    // srv.request.goal = {0, 0.5, 0, 0};
    if (srv_client_goto_global_.call(srv))
    {
      // ROS_INFO("Moving to [%d,%d,%d]...", srv.request.goal.at(0),
      //          srv.request.goal.at(1), srv.request.goal.at(2));
    }
    else
    {
      ROS_ERROR("[Offshore] Failed to call service \"srv_client_goto_global_\"");
    }
  }

  void OffshoreUAV::changeEstimator(mrs_msgs::String message)
  {
    if (srv_client_change_alt_estimator_.call(message))
    {
      // ROS_INFO("Moving to [%d,%d,%d]...", srv.request.goal.at(0),
      //          srv.request.goal.at(1), srv.request.goal.at(2));
    }
    else
    {
      ROS_ERROR("[Offshore] Failed to call service \"srv_client_change_alt_estimator_\"");
    }
  }

  void OffshoreUAV::goto_to_fix(mrs_msgs::Vec4 srv)
  {

    /* create reference stamped msg */
    mrs_msgs::ReferenceStampedSrv srv_reference_stamped_msg;

    /* fill in header */
    srv_reference_stamped_msg.request.header.stamp = ros::Time::now();
    srv_reference_stamped_msg.request.header.frame_id = _uav_name_ + "/fcu"; // + _frame_;

    /* fill in reference */
    srv_reference_stamped_msg.request.reference.position.x = 0;
    srv_reference_stamped_msg.request.reference.position.y = 0;
    srv_reference_stamped_msg.request.reference.position.z = 5;
    srv_reference_stamped_msg.request.reference.heading = 0;

    /* request service */
    if (srv_client_goto_.call(srv_reference_stamped_msg))
    {
    }
    else
    {
      ROS_ERROR("Failed to call service.\n");
    }
  }


  // | --------------------------- helper functions ----------------------------- |
  void OffshoreUAV::wait(int time, std::string msg)
  {
    std::string customMessage = "[Offshore]: " + msg + " %d seconds remaining.";

    ROS_INFO_THROTTLE(5, customMessage.c_str(), time);

    ros::Duration(time).sleep();
  }

  // | --------------------------- timer callbacks ----------------------------- |

  /* callbackTimerStateMachine() //{ */

  void OffshoreUAV::callbackTimerStateMachine([[maybe_unused]] const ros::TimerEvent &event)
  {

    //wait(5, "Starting loop");

    if (!is_initialized_) //|| !state_machine_running_)
    {
      return;
    }

    ROS_INFO("Started the loop after if");
    ros::Time now = ros::Time::now();

    wait(5, FRED("ESTIMATOR NOW IS DIFFERENT"));
    //TODO:
    //rosservice call /uav1/odometry/change_alt_estimator_type_string "value: baro"
    //echo "wait 5 sec"
    //sleep 5
    mrs_msgs::String change_alt_estimator;
    change_alt_estimator.request.value = "baro";
    changeEstimator(change_alt_estimator);

    wait(5, FRED("CHANGED ESTIMATOR"));

    ROS_INFO("Increasing altitude for the sand montain");
    mrs_msgs::Vec4 srv;
    srv.request.goal = {0, 0, 5, 0};
    gotoRelative(srv);
    wait();

    for (int i = 1; i <= 3; i++)
    {
      //echo ""
      //echo "counter: $i / 3 => -1m"
      ROS_INFO("LOOP of sand montain");
      //rosservice call /uav1/control_manager/goto_relative "goal: [0.0, -10.0, 0.0, 0.0]"

      srv.request.goal = {0, -10, 0, 0};
      gotoRelative(srv);
      wait();
    }

    ROS_INFO("First boat");

    srv.request.goal = {-123136.570269, 1108437.85305, 0.630100856701, -1.62};
    gotoGlobal(srv);
    wait(15, FBLU("I am going to the boat, look at me please"));
    
    wait(15, FBLU("I am at the boat, look at me please"));
    
    /*
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
    */
    
    //mrs_msgs::String change_alt_estimator;
    change_alt_estimator.request.value = "HEIGHT";
    changeEstimator(change_alt_estimator);

    wait(5, FRED("CHANGED ESTIMATOR"));

    ROS_INFO("Decreasing altitude for the land in the boat");
    for(auto i = 1; i <= 10; i++){
      srv.request.goal = {0, 0, -2, 0};
      gotoRelative(srv);
      wait(3);
    }

    ROS_INFO("Decreasing altitude in a small step for the land in the boat");
    for(auto i = 1; i <= 8; i++){      
      srv.request.goal = {0, 0, -0.2, 0};
      gotoRelative(srv);
      wait(1);
    }

    wait(15, FGRN("LANDED"));
    srv.request.goal = {0, 0, 13, 0};
    gotoRelative(srv);
    wait(15);

    change_alt_estimator.request.value = "BARO";
    changeEstimator(change_alt_estimator);

    wait(5, FRED("CHANGED ESTIMATOR"));


/*
##for i in 1 2 3 4 5 6 7 8; do
##    rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 0.0, -0.2, 0.0]"
##    echo "wait 5 sec"
##    sleep 1
##done
//##rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 0.0, 13.0, 0.0]"

rosservice call /uav1/odometry/change_alt_estimator_type_string "value: baro"

second_boat_position:
rosservice call /uav1/control_manager/goto "goal: [-123129.739198, 1108420.15897, 0.630100856701, -1.62]"
*/

    ROS_INFO("Helipoint");
    //offshore_landing_position:
    srv.request.goal = {-123144.056766, 1108424.88005, 6.12495578052, 1.62};
    gotoGlobal(srv);
    wait(20);

    ROS_INFO("Around the offshore base");
    //Apartir do ponto pouso da offshore_landing_position, o drone vai:
    //rosservice call /uav1/control_manager/goto_relative "goal: [-40.0, 0.0, 0.0, 0.0]"

    srv.request.goal = {-40, 0, 0, 0};
    gotoRelative(srv);
    wait(20);
    //rosservice call /uav1/control_manager/goto_relative "goal: [0.0, -40.0, 0.0, 0.0]"

    //srv.request.goal = {0, -40, 0, 0};
    srv.request.goal = {0, -35, 0, 0};
    gotoRelative(srv);
    wait(20);
    //rosservice call /uav1/control_manager/goto_relative "goal: [40.0, 0.0, 0.0, 0.0]"

    srv.request.goal = {40, 0, 0, 0};
    gotoRelative(srv);
    wait(20);
    //rosservice call /uav1/control_manager/goto_relative "goal: [0.0, 40.0, 0.0, 0.0]"

    //srv.request.goal = {0, 40, 0, 0};
    srv.request.goal = {0, 35, 0, 0};
    gotoRelative(srv);
    wait(20);

    ROS_INFO("Going back to land in helipoint");
    //offshore_landing_position:

    srv.request.goal = {-123144.056766, 1108424.88005, 6.12495578052, 1.62};
    gotoGlobal(srv);
    wait();

    ROS_INFO(FYEL("LAND"));

    return;
  }

  //}

} // namespace offshore_uav

PLUGINLIB_EXPORT_CLASS(offshore_uav::OffshoreUAV, nodelet::Nodelet);
