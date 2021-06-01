#include <wamv.h>

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
        
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */

        // mrs_lib::ParamLoader param_loader(nh, "wamv");

        // param_loader.loadParam("UAV_NAME", _uav_name_);
        // param_loader.loadParam("gui", _gui_);
    
        // | ----------------- initialize subscribers ----------------- |

        // usvMovimentSubscriber = nh.subscribe("", 1000, &wamv::movimentInCallback, this);
        usvMovimentTimer = nh.createTimer(20, &wamv::movimentInCallback, this);
        // | -------------- initialize serviceClients ----------------- |

        rightThrust = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);
        rightAngle = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_angle_cmd", 10);
        leftThrust = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);
        leftAngle = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_angle_cmd", 10);
        ROS_INFO_ONCE("[WAMV] - Iniciado programa!");
        
        ros::AsyncSpinner spinner(0); // Use all threads avaliable
        spinner.start();
        ros::waitForShutdown();
    }

    // ███╗   ███╗ ██████╗ ██╗   ██╗██╗███╗   ███╗███████╗███╗   ██╗████████╗    ██╗███╗   ██╗
    // ████╗ ████║██╔═══██╗██║   ██║██║████╗ ████║██╔════╝████╗  ██║╚══██╔══╝    ██║████╗  ██║
    // ██╔████╔██║██║   ██║██║   ██║██║██╔████╔██║█████╗  ██╔██╗ ██║   ██║       ██║██╔██╗ ██║
    // ██║╚██╔╝██║██║   ██║╚██╗ ██╔╝██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║   ██║       ██║██║╚██╗██║
    // ██║ ╚═╝ ██║╚██████╔╝ ╚████╔╝ ██║██║ ╚═╝ ██║███████╗██║ ╚████║   ██║       ██║██║ ╚████║
    // ╚═╝     ╚═╝ ╚═════╝   ╚═══╝  ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝   ╚═╝       ╚═╝╚═╝  ╚═══╝
    void wamv::movimentInCallback(const ros::TimerEvent& event){
        std_msgs::Float32 forceThruster;
        std_msgs::Float32 angleThruster;
        
        forceThruster.data = 1.0f;
        angleThruster.data = 0.0f;
        
        controlThrust(forceThruster, angleThruster);
        wamv::wait(10, "Moving WAM-V");

        forceThruster.data = 0.0f;
        angleThruster.data = 0.0f;

        controlThrust(forceThruster, angleThruster);
        wamv::wait(10, "Stoping WAM-V");
    }    

    void wamv::controlThrust(std_msgs::Float32 force, std_msgs::Float32 angle){
        rightThrust.publish(force);
        leftThrust.publish(force);
        rightAngle.publish(angle);
        leftAngle.publish(angle);
    }
}

PLUGINLIB_EXPORT_CLASS(wamv_usv::wamv, nodelet::Nodelet);
    
