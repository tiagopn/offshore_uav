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
    state_machine_running_ = false;
    waypoints_loaded_ = false;

    /* first take off, will define if should set mode or not */
    firstTakeoff = true;

    ros::NodeHandle nh("~");

    ros::Time::waitForValid();

    mrs_lib::ParamLoader param_loader(nh, "Offshore");

    /* load parameters */
    param_loader.loadParam("uav_name", _uav_name_);
    //param_loader.loadParam("frame", _frame_);
    param_loader.loadParam("top_of_the_hill/position/x", _top_of_the_hill_.x);
    param_loader.loadParam("top_of_the_hill/position/y", _top_of_the_hill_.y);
    param_loader.loadParam("top_of_the_hill/position/z", _top_of_the_hill_.z);
    param_loader.loadParam("top_of_the_hill/position/heading", _top_of_the_hill_.heading);

    param_loader.loadParam("first_boat/position/x", _first_boat_.x);
    param_loader.loadParam("first_boat/position/y", _first_boat_.y);
    param_loader.loadParam("first_boat/position/z", _first_boat_.z);
    param_loader.loadParam("first_boat/position/heading", _first_boat_.heading);

    param_loader.loadParam("offshore_landing/position/x", _offshore_landing_.x);
    param_loader.loadParam("offshore_landing/position/y", _offshore_landing_.y);
    param_loader.loadParam("offshore_landing/position/z", _offshore_landing_.z);

    param_loader.loadParam("wait_arrival", _wait_arrival_);
    param_loader.loadParam("wait_timer", _wait_timer_);

    param_loader.loadParam("loop_offshore", _loop_offshore_);
    param_loader.loadParam("land_at_the_end", _land_end_);
    param_loader.loadParam("visit_the_boat", _visit_boat_);


    /* load waypoints as a half-dynamic matrix from config file */
    Eigen::MatrixXd waypoint_matrix;
    param_loader.loadMatrixDynamic("waypoints", waypoint_matrix, -1, 4); // -1 indicates the dynamic dimension
    waypoints_ = matrixToPoints(waypoint_matrix);
    n_waypoints_ = waypoints_.size();
    waypoints_loaded_ = true;
    idx_current_waypoint_ = 0;
    c_loop_ = 0;
    ROS_INFO_STREAM_ONCE("[Offshore]: " << n_waypoints_ << " waypoints loaded");

    ROS_INFO_STREAM("[Offshore - Route 1]");
    for (int i = 0; i < 4; i++)
    {
      ROS_INFO_STREAM("[Offshore - Route 1]: " << waypoints_[i].position.x << " " << waypoints_[i].position.y << " " << waypoints_[i].position.z << " " << waypoints_[i].heading << " point relative requested");
    }
    ROS_INFO_STREAM("[Offshore - Route 2]");
    for (int i = 4; i < n_waypoints_; i++)
    {
      ROS_INFO_STREAM("[Offshore - Route 2]: " << waypoints_[i].position.x << " " << waypoints_[i].position.y << " " << waypoints_[i].position.z << " " << waypoints_[i].heading << " point relative requested");
    }

    //param_loader.loadParam("landing")

    if (!param_loader.loadedSuccessfully())
    {
      ROS_ERROR("[Offshore]: failed to load non-optional parameters!");
      ros::shutdown();
    }

    // | -------------- initialize state subscriber ----------------- |
    uavStateSubscriber =
        nh.subscribe("uav_state_in", 1000, &OffshoreUAV::stateInCallback, this);

    // | -------------- initialize serviceClients ----------------- |
    ///* service client */
    // Correct but not working
    srv_client_goto_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + _uav_name_ + "/control_manager/reference");

    // Wrong but still work
    srv_client_goto_relative_ = nh.serviceClient<mrs_msgs::Vec4>("/uav1/control_manager/goto_relative");
    srv_client_goto_global_ = nh.serviceClient<mrs_msgs::Vec4>("/uav1/control_manager/goto");

    //rosservice call /uav1/odometry/change_alt_estimator_type_string "value: baro"
    srv_client_change_alt_estimator_ = nh.serviceClient<mrs_msgs::String>("/uav1/odometry/change_alt_estimator_type_string");

    // landing
    srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("/" + _uav_name_ + "/uav_manager/land");

    // takeoff
    srv_client_arming_ = nh.serviceClient<mavros_msgs::CommandBool>("/" + _uav_name_ + "/mavros/cmd/arming");
    srv_client_setMode_ = nh.serviceClient<mavros_msgs::SetMode>("/" + _uav_name_ + "/mavros/set_mode");
    srv_client_takeoff_ = nh.serviceClient<std_srvs::Trigger>("/" + _uav_name_ + "/uav_manager/takeoff");

    // timers
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

  void OffshoreUAV::stateInCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    {
      std::scoped_lock lock(mutex_odometry_);
      OffshoreUAV::position_x_ = msg->pose.pose.position.x;
      OffshoreUAV::position_x_ = msg->pose.pose.position.y;
      OffshoreUAV::position_z_ = msg->pose.pose.position.z;
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

  void OffshoreUAV::gotoReferenceStamped(mrs_msgs::Vec4 srv)
  {

    /* create reference stamped msg */
    mrs_msgs::ReferenceStampedSrv srv_reference_stamped_msg;

    /* fill in header */
    srv_reference_stamped_msg.request.header.stamp = ros::Time::now();
    srv_reference_stamped_msg.request.header.frame_id = _uav_name_ + "/gps_origin"; // + _frame_;

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

  void OffshoreUAV::waitArrivalAt(double x, double y, double z)
  {
    // Await for a minimal of 10 seconds

    // ROS_INFO("[OffshoreUAV]: Waiting for arrival at [%d,%d,%d]...", x, y, z);
    for (int count = 0; count < _wait_arrival_; count++)
    {
      // Threshold
      const float threshold = 0.25;

      bool x_test =
          (x - threshold) < position_x_ and position_x_ < (x + threshold);

      bool y_test =
          (y - threshold) < position_y_ and position_y_ < (y + threshold);

      bool z_test =
          (z - threshold) < position_z_ and position_z_ < (z + threshold);

      if (x_test and y_test and z_test and (count > 10))
        break;

      // sleep(1);
      ros::Duration(1).sleep();
    }
    ROS_INFO("[OffshoreUAV]: Arrived at the desired destination.");
  }

  void OffshoreUAV::land()
  {
    std_srvs::Trigger srv_trigger;

    if (srv_client_land_.call(srv_trigger))
    {
      ROS_INFO("[OffshoreUAV]: Landing...");
    }
    else
    {
      ROS_ERROR("[OffshoreUAV]: Failed to call service \"land\"");
    }
    wait();
  }

  void OffshoreUAV::takeoff()
  {
    mavros_msgs::CommandBool msgArming;
    mavros_msgs::SetMode msgModeClient;
    std_srvs::Trigger srvTakeoff;

    /* arming */
    msgArming.request.value = 1;
    if (srv_client_arming_.call(msgArming))
    {
      ROS_INFO("[OffshoreUAV]: Arming drone...");
    }
    else
    {
      ROS_ERROR("[OffshoreUAV]: Failed to call service `srv_client_arming_`.");
    }

    wait(10, "Waiting for arming before taking off...");

    /* set_mode */
    if (firstTakeoff)
    {
      firstTakeoff = false;

      // Type: mavros_msgs/SetMode
      // Args: base_mode custom_mode

      msgModeClient.request.base_mode = 0;
      msgModeClient.request.custom_mode = "offboard";

      if (srv_client_setMode_.call(msgModeClient))
      {
        // ROS_INFO("msgModeClient.response.success: %d",
        ROS_INFO("[OffshoreUAV]: Drone mode changed.");
        //          msgModeClient.response.mode_sent);
      }
      else
      {
        ROS_INFO("[OffshoreUAV]: Failed to change mode.");
        // ROS_ERROR("Failed to call service `msgModeClient`.");
      }
    }

    /* takeoff */
    if (srv_client_takeoff_.call(srvTakeoff))
    {
      if (srvTakeoff.response.success)
      {
        // ROS_INFO("srvTakeoff.response.success: %d",
        ROS_INFO("[OffshoreUAV] Drone take off started.");
        // srvTakeoff.response.success);
      }
      else
      {
        // ROS_ERROR("srvTakeoff.response.success: %d",
        // srvTakeoff.response.success);
        ROS_INFO("[OffshoreUAV]: Error on taking off: %d", srvTakeoff.response.success);
      }
    }
    else
    {
      ROS_ERROR("[OffshoreUAV]: Failed to call service \"takeoffClient\"");
    }
  }

  std::vector<mrs_msgs::Reference> OffshoreUAV::matrixToPoints(const Eigen::MatrixXd &matrix)
  {
    std::vector<mrs_msgs::Reference> points;

    for (int i = 0; i < matrix.rows(); i++)
    {

      mrs_msgs::Reference point;
      point.position.x = matrix.row(i)(0);
      point.position.y = matrix.row(i)(1);
      point.position.z = matrix.row(i)(2);
      point.heading = matrix.row(i)(3);

      points.push_back(point);
    }

    return points;
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
    wait(5, FYEL("ESTIMATOR NOW IS DIFFERENT"));
    mrs_msgs::String change_alt_estimator;
    change_alt_estimator.request.value = "baro";
    changeEstimator(change_alt_estimator);
    wait(5, FYEL("CHANGED ESTIMATOR"));

    ROS_INFO("Increasing altitude for the sand montain");
    mrs_msgs::Vec4 srv;

    srv.request.goal = {0, 0, 5, 0};
    gotoRelative(srv);
    wait();

    srv.request.goal = {_top_of_the_hill_.x, _top_of_the_hill_.y, _top_of_the_hill_.z, _top_of_the_hill_.heading};
    gotoGlobal(srv);
    waitArrivalAt(_top_of_the_hill_.x, _top_of_the_hill_.y, _top_of_the_hill_.z);
    //wait(30, FBLU("I am going to the sand montain, look at me please"));

    if (_visit_boat_)
    {

      ROS_INFO("First boat");

      srv.request.goal = {_first_boat_.x, _first_boat_.y, _first_boat_.z, _first_boat_.heading};
      gotoGlobal(srv);
      waitArrivalAt(_first_boat_.x, _first_boat_.y, _first_boat_.z);
      wait(5, FBLU("I am going to the boat, look at me please"));
      wait(15, FBLU("I am at the boat, look at me please"));

      //mrs_msgs::String change_alt_estimator;
      change_alt_estimator.request.value = "HEIGHT";
      changeEstimator(change_alt_estimator);

      wait(5, FYEL("CHANGED ESTIMATOR"));

      ROS_INFO("Decreasing altitude for the land in the boat");
      for (auto i = 1; i <= 10; i++)
      {
        srv.request.goal = {0, 0, -2, 0};
        gotoRelative(srv);
        wait();
      }

      ROS_INFO(FGRN("LANDED"));
      land();
      wait(10);
      takeoff();
      wait(30);

      srv.request.goal = {0, 0, 13, 0};
      gotoRelative(srv);
      wait(15, "Going up to change the altitude estimator");

      change_alt_estimator.request.value = "BARO";
      changeEstimator(change_alt_estimator);

      wait(5, FRED("CHANGED ESTIMATOR"));
    }
    //offshore_landing_position:
    ROS_INFO("Helipoint");
    srv.request.goal = {_offshore_landing_.x, _offshore_landing_.y, _offshore_landing_.z, _offshore_landing_.heading};
    gotoGlobal(srv);
    waitArrivalAt(_offshore_landing_.x, _offshore_landing_.y, _offshore_landing_.z);

    if (_loop_offshore_)
    {

      ROS_INFO("Around the offshore base");

      for (int i = 0; i < 4; i++)
      {
        ROS_INFO_STREAM("[Offshore - Route 1]: " << waypoints_[i].position.x << " " << waypoints_[i].position.y << " " << waypoints_[i].position.z << " " << waypoints_[i].heading << " point relative requested");
        srv.request.goal = {waypoints_[i].position.x, waypoints_[i].position.y, waypoints_[i].position.z, waypoints_[i].heading};
        gotoRelative(srv);
        wait(_wait_timer_);
      }

      ROS_INFO("Going back to land in helipoint");

      ROS_INFO("Helipoint");
      srv.request.goal = {_offshore_landing_.x, _offshore_landing_.y, _offshore_landing_.z, _offshore_landing_.heading};
      gotoGlobal(srv);
      waitArrivalAt(_offshore_landing_.x, _offshore_landing_.y, _offshore_landing_.z);

      ROS_INFO("Around the offshore base towers");

      for (int i = 4; i < n_waypoints_; i++)
      {
        ROS_INFO_STREAM("[Offshore - Route 2]: " << waypoints_[i].position.x << " " << waypoints_[i].position.y << " " << waypoints_[i].position.z << " " << waypoints_[i].heading << " point relative requested");
        srv.request.goal = {waypoints_[i].position.x, waypoints_[i].position.y, waypoints_[i].position.z, waypoints_[i].heading};
        gotoRelative(srv);
        wait(_wait_timer_);
      }

      ROS_INFO("Going back to land in helipoint");
      //offshore_landing_position:

      srv.request.goal = {_offshore_landing_.x, _offshore_landing_.y, _offshore_landing_.z, _offshore_landing_.heading};
      gotoGlobal(srv);
      waitArrivalAt(_offshore_landing_.x, _offshore_landing_.y, _offshore_landing_.z);
    }

    if (_land_end_)
    {
      ROS_INFO(FGRN("Land at the end is true, landing"));
      land();
    }
    else
    {
      ROS_INFO("Land at the end is false, hovering");
    }

    return;
  }

  //}

} // namespace offshore_uav

PLUGINLIB_EXPORT_CLASS(offshore_uav::OffshoreUAV, nodelet::Nodelet);
