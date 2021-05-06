#include <include/wamv.hh>
/* every nodelet must include macros which export the class as a nodelet plugin
 */
#include <pluginlib/class_list_macros.h>

namespace wamv_usv {
    void wamv::onInit(){
        /* obtain node handle */
        ros::NodeHandle nh("~");

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // new nodehandle for goto calls
        ros::NodeHandle n;

        // NodeHandle for land calls
        ros::NodeHandle nhLanding;

        // NodeHandles for takeoff calls
        ros::NodeHandle nhArming;
        ros::NodeHandle nhSetMode;
        ros::NodeHandle nhTakeoff;

        // | ------------------- load ros parameters ------------------ |

        mrs_lib::ParamLoader param_loader(nh, "wamv");
        param_loader.loadParam("USV_NAME", _usv_name_);
        param_loader.loadParam("gui", _gui_);
        
        if (!param_loader.loadedSuccessfully())
        {
            ROS_ERROR("[WAMV]: failed to load non-optional parameters!");
            ros::shutdown();
        }

        // | ----------------- initialize subscribers ----------------- |

        uavMovimentSubscriber = nh.subscribe("uav_state_in", 1000, &FaseDois::movimentInCallback, this);

        // | -------------- initialize serviceClients ----------------- |

        rightThrust = n.serviceClient<std_msgs::Float32>("/right_thrust_cmd");
        rightAngle = n.serviceClient<std_msgs::Float32>("/right_thrust_angle");
        leftThrust = n.serviceClient<std_msgs::Float32>("/left_thrust_cmd")
        leftAngle = n.serviceClient<std_msgs::Float32>("/left_thrust_angle");

        ros::AsyncSpinner spinner(0); // Use all threads avaliable
        spinner.start();
        ros::waitForShutdown();
    }
}
PLUGINLIB_EXPORT_CLASS(wamv_usv::wamv, nodelet::Nodelet);
