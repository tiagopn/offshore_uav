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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* custom library */
#include <colors.h>

using namespace message_filters;

namespace offshore_uav
{

class OffshoreUAV : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_;
  bool hover_mode_;
  bool swarming_mode_;

  /* ros parameters */
  std::string _uav_name_;

  /* publishers */
  ros::Publisher pub_mode_changed_;
  ros::Publisher pub_virtual_heading_;

  // | ---------------------- proximal control parameters ---------------------- |

  double _desired_distance_;
  double _range_multipler_;
  double _steepness_potential_;
  double _strength_potential_;
  
  double max_range_;
  double noise_;
  
  // | ----------------------- motion control parameters ----------------------- |

  double _K1_;  // Linear gain
  double _K2_;  // Angular gain
  double _move_forward_;
  double _interpolate_coeff_;
  bool  _fixed_heading_;

  double virtual_heading_;
  double smooth_heading_;
  double initial_heading_;
  
  // | --------------------------- timer callbacks ----------------------------- |

  /* after start the swarming mode, the node will run for ($_timeout_flocking_) seconds */
  void       callbackTimerAbortFlocking(const ros::TimerEvent& event);
  ros::Timer timer_flocking_end_;
  double     _timeout_flocking_;

  void       callbackTimerStateMachine(const ros::TimerEvent& event);
  ros::Timer timer_state_machine_;
  ros::Time  state_change_time_;
  bool       state_machine_running_;
  double     _timeout_state_change_;

  // | ------------------------ service clients callbacks ---------------------- |

  ros::ServiceClient srv_client_goto_;
  ros::ServiceClient srv_client_land_;
  ros::ServiceClient srv_client_change_alt_estimator_;
  
  

  ros::ServiceClient srv_client_goto_relative_;
  ros::ServiceClient srv_client_goto_global_;

  bool               _land_end_;

  // | ------------------------ service server callbacks ----------------------- |

  /* start state machine (trigger hover now and wait #s timeout for swarm mode) */
  bool               callbackStartStateMachine(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_state_machine_;

  // | -------------------------- support functions ---------------------------- |

  void gotoRelative(mrs_msgs::Vec4 srv);
  void gotoGlobal(mrs_msgs::Vec4 srv);
  void changeEstimator(mrs_msgs::String message);
  
  void goto_to_fix(mrs_msgs::Vec4 srv);

  void wait(int time = 5, std::string msg = "Waiting...");

  
};

}  // namespace offshore_uav
#endif

