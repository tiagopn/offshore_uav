#pragma once
#ifndef wamv_EDGE_DETECT_H
#define wamv_EDGE_DETECT_H

/* includes

/* each ROS nodelet must have these */
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>

/* TF2 related ROS includes */
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

/* camera image messages */
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

/* long unsigned integer message */
#include <std_msgs/Float32.h>
#include <std_msgs/UInt64.h>

/* some STL includes */
#include <stdio.h>
#include <stdlib.h>

#include <mutex>

/* custom helper functions from our library */

/* ROS includes for LaserScan Lib */
#include <sensor_msgs/LaserScan.h>

/* ROS includes for Odometry*/
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <pluginlib/class_list_macros.h>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>

namespace wamv_usv
{
    class wamv:public nodelet::Nodelet
    {
        public:
            /* onInit() is called when nodelet is launched (similar to main() in regular node) */
            virtual void onInit();
        private:
            /* flags */
            
            /* ros parameters */    
            bool _gui_;

            /* variables*/
            std::string _usv_name_;

             // | --------------------- timer callbacks -------------------- |

            void callbackTimerCheckSubscribers(const ros::TimerEvent &te);
            ros::Timer timer_check_subscribers_;
            int _rate_timer_check_subscribers_;

            /* Subscribers */
            ros::Subscriber usvStateSubscriber;
            ros::Subscriber usvMovimentSubscriber;
            ros::Subscriber cameraInfoSubscriber;
            ros::Subscriber lidarSubscriber;

            ros::Subscriber usvGPSSubscriber;
            ros::Subscriber usvIMUSubscriber;
            ros::Subscriber usvP3DSubscriber;

            ros::Publisher pub_mode_changed_;
   
            ros::Publisher rightThrust;
            ros::Publisher leftThrust;
            ros::Publisher rightAngle;
            ros::Publisher leftAngle;

            // | ----------------------- message filters callbacks -----------------------
            // |

            // | ---------------------- msg callbacks --------------------- |

            ros::Timer usvMovimentTimer;            
            void movimentInCallback(const ros::TimerEvent& event);
            void imuInCallback(const sensor_msgs::Imu::ConstPtr &msg);
            void p3dInCallback(const nav_msgs::Odometry::ConstPtr &msg);
            

            // | ------------------- helper functions --------------------- |
            void controlThrustLeft(std_msgs::Float32 force, std_msgs::Float32 angle);
            void controlThrustRight(std_msgs::Float32 force, std_msgs::Float32 angle);
            void goFront(int movementTime);
            void goBack(int movementTime);

            void turnRight(int movementTime);
            void turnLeft(int movementTime);

            void turnFrontAngle(float angle);
            void turnAngle(float angle);

            void gotoGlobal(float x, float y);

            // | ----------------------- Variable Control --------------------------------|

            bool moveCompleted;

            // | ------------------------ service server callbacks -----------------------
            // |
            // | --------------------- WAM-V variables -------------------- |
        
            float position_x_ = 0.0;
            float position_y_ = 0.0;
            float position_z_ = 0.0;
        
            float acceleration_x_ = 0.0;
            float acceleration_y_ = 0.0;
            float acceleration_z_ = 0.0;

            float angular_velocity_x_ = 0.0;
            float angular_velocity_y_ = 0.0;
            float angular_velocity_z_ = 0.0;
            
            float heading_rad;
            float heading_degree;
            
            const float PI = 3.14159265359;

            // | --------------------- Thruster variables -------------------- |

            std_msgs::Float32 forceThrusterRight;
            std_msgs::Float32 forceThrusterLeft;
            std_msgs::Float32 angleThrusterRight;
            std_msgs::Float32 angleThrusterLeft;
       

            ros::ServiceClient srv_client_goto_;
            ros::ServiceClient armingClient;
    };
}
#endif
