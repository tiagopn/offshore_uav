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

/* long unsigned integer message */
#include <std_msgs>
#include <std_msgs/UInt64.h>

/* some STL includes */
#include <stdio.h>
#include <stdlib.h>

#include <mutex>

/* some OpenCV includes */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

/* ROS includes for working with OpenCV and images */
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* ROS includes for LaserScan Lib */
#include <sensor_msgs/LaserScan.h>

/* ROS includes for Odometry*/
#include <math.h>
#include <mrs_msgs/Vec4.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"


namespace wamv_usv
{
    class wamv : public nodelet::Nodelet
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
            image_transport::Subscriber imageSubscriber;
            ros::Subscriber uavStateSubscriber;
            ros::Subscriber uavMovimentSubscriber;
            ros::Subscriber cameraInfoSubscriber;
            ros::Subscriber lidarSubscriber;

            ros::Publisher pub_mode_changed_

            ros::ServiceClient rightThrust;
            ros::ServiceClient leftThrust;
            ros::ServiceClient rightAngle;
            ros::ServiceClient leftAngle;

            // | ----------------------- message filters callbacks -----------------------
            // |

            // | ---------------------- msg callbacks --------------------- |
            ros::Subscriber usvMovimentSubscriber;

            // | ------------------- helper functions --------------------- |
            void wait(int time = 10, std::string msg = "Waiting...");
            void gotoGlobal(mrs_msgs::Vec4 srv);

            // | ------------------------ service server callbacks -----------------------
            // |

            ros::ServiceClient srv_client_goto_;
            ros::ServiceClient armingClient;
    }
}
#endif
