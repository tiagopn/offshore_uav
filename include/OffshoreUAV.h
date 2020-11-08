#pragma once
#ifndef OFFSHORE_UAV_H
#define OFFSHORE_UAV_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry_utils.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/Float64Stamped.h>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/String.h>

#include <std_srvs/Trigger.h>

#include <mutex>

/* mavros messages*/
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

/* custom library */
#include <colors.h>


namespace offshore_uav
{

class OffshoreUAV : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_;
  bool hover_mode_;
  bool firstTakeoff;
  bool _land_end_;

  std::mutex mutex_odometry_;


  /* ros parameters */
  std::string _uav_name_;
  bool _loop_offshore_;
  int _wait_arrival_;
  int _wait_timer_;

  struct POI {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0; 
  };

  POI _top_of_the_hill_;
  POI _first_boat_;
  POI _offshore_landing_;

  std::vector<mrs_msgs::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);   
  std::vector<mrs_msgs::Reference> waypoints_;
  int n_waypoints_         ;
  bool waypoints_loaded_   ;
  int idx_current_waypoint_;
  int c_loop_              ;


  void       callbackTimerStateMachine(const ros::TimerEvent& event);
  ros::Timer timer_state_machine_;
  ros::Time  state_change_time_;
  bool       state_machine_running_;
  double     _timeout_state_change_;

  // | ------------------------ service clients callbacks ---------------------- |

  ros::ServiceClient srv_client_goto_;
  ros::ServiceClient srv_client_goto_relative_;
  ros::ServiceClient srv_client_goto_global_;
  
  ros::ServiceClient srv_client_change_alt_estimator_;
  
  ros::ServiceClient srv_client_land_;

  ros::ServiceClient srv_client_arming_;
  ros::ServiceClient srv_client_setMode_;
  ros::ServiceClient srv_client_takeoff_;


  float position_x_ = 0.0;
  float position_y_ = 0.0;
  float position_z_ = 0.0;

  // | ------------------------ subscribers              ----------------------- |
  ros::Subscriber uavStateSubscriber;


  // | ------------------------ msg            callbacks ----------------------- |
  void stateInCallback(const nav_msgs::OdometryConstPtr& msg);

  // | ------------------------ service server callbacks ----------------------- |

  /* start state machine (trigger hover now and wait #s timeout for swarm mode) */
  bool               callbackStartStateMachine(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_state_machine_;

  // | -------------------------- support functions ---------------------------- |

  void gotoRelative(mrs_msgs::Vec4 srv);
  void gotoGlobal(mrs_msgs::Vec4 srv);
  void land();
  void takeoff();
  void gotoReferenceStamped(mrs_msgs::Vec4 srv);

  void changeEstimator(mrs_msgs::String message);
  

  void wait(int time = 5, std::string msg = "Waiting...");
  void waitArrivalAt(double x, double y, double z);

};

}  // namespace offshore_uav
#endif

