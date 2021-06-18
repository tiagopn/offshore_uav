#include "wamv.h"

/* every nodelet must include macros which export the class as a nodelet plugin
 */



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

        // | ------------------- load ros parameters ------------------ |
        
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */

        mrs_lib::ParamLoader param_loader(nh, "wamv");

        param_loader.loadParam("USV_NAME", _usv_name_);
    
        // | ----------------- initialize subscribers ----------------- |

        usvMovimentTimer = nh.createTimer(20, &wamv::movimentInCallback, this);


        // | -------------- initialize serviceClients ----------------- |
        // | -------------- initialize Sensor ----------------- |
        
        usvIMUSubscriber = nh.subscribe("/wamv/sensors/imu/imu/data", 1, &wamv::imuInCallback, this);
        usvP3DSubscriber = nh.subscribe("/wamv/sensors/position/p3d_wamv", 1, &wamv::p3dInCallback, this);

        

        // | -------------- initialize Motor  ----------------- |
        rightThrust = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1);
        rightAngle = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 1);
        leftThrust = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1);
        leftAngle = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 1);
        
        
        
        ROS_INFO_ONCE("[WAMV] - Iniciado programa!");
        ros::Duration(10.0).sleep();

        ros::AsyncSpinner spinner(0); // Use all threads avaliable
        spinner.start();
        ros::waitForShutdown();
    }

    // ██████╗  ██████╗ ███████╗██╗████████╗██╗ ██████╗ ███╗   ██╗
    // ██╔══██╗██╔═══██╗██╔════╝██║╚══██╔══╝██║██╔═══██╗████╗  ██║
    // ██████╔╝██║   ██║███████╗██║   ██║   ██║██║   ██║██╔██╗ ██║
    // ██╔═══╝ ██║   ██║╚════██║██║   ██║   ██║██║   ██║██║╚██╗██║
    // ██║     ╚██████╔╝███████║██║   ██║   ██║╚██████╔╝██║ ╚████║
    // ╚═╝      ╚═════╝ ╚══════╝╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝
                                                           
    void wamv::imuInCallback(const sensor_msgs::Imu::ConstPtr& msg){
        
        double heading = mrs_lib::AttitudeConverter(msg->orientation).getHeading();
        ROS_INFO("IMU = %.2f", heading);
        
        acceleration_x_ = msg->linear_acceleration.x;
        acceleration_y_ = msg->linear_acceleration.y;
        acceleration_z_ = msg->linear_acceleration.z;

        angular_velocity_x_ = msg->angular_velocity.x;
        angular_velocity_y_ = msg->angular_velocity.y;
        angular_velocity_z_ = msg->angular_velocity.z;
        
        orientation_x_ = msg->orientation.x;
        orientation_y_ = msg->orientation.y;
        orientation_z_ = msg->orientation.z;
        orientation_angle_ = msg->orientation.w;
    }

    void wamv::p3dInCallback(const nav_msgs::Odometry::ConstPtr &msg){
        double heading = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
        
        ROS_INFO("P3D = %.2f", heading);

        position_x_ = msg->pose.pose.position.x;
        position_y_ = msg->pose.pose.position.y;
        position_z_ = msg->pose.pose.position.z;
    }

    // ███╗   ███╗ ██████╗ ██╗   ██╗██╗███╗   ███╗███████╗███╗   ██╗████████╗    ██╗███╗   ██╗
    // ████╗ ████║██╔═══██╗██║   ██║██║████╗ ████║██╔════╝████╗  ██║╚══██╔══╝    ██║████╗  ██║
    // ██╔████╔██║██║   ██║██║   ██║██║██╔████╔██║█████╗  ██╔██╗ ██║   ██║       ██║██╔██╗ ██║
    // ██║╚██╔╝██║██║   ██║╚██╗ ██╔╝██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║   ██║       ██║██║╚██╗██║
    // ██║ ╚═╝ ██║╚██████╔╝ ╚████╔╝ ██║██║ ╚═╝ ██║███████╗██║ ╚████║   ██║       ██║██║ ╚████║
    // ╚═╝     ╚═╝ ╚═════╝   ╚═══╝  ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝   ╚═╝       ╚═╝╚═╝  ╚═══╝
    void wamv::movimentInCallback(const ros::TimerEvent& event){
        // ROS_INFO("[WAMV] - Position \t\t=>\t X = %.2f, Y = %.2f, Z = %.2f", position_x_, position_y_, position_z_);
        // goFront(8);
        // ros::Duration(5.0).sleep();

        turnLeft(8);
        ros::Duration(5.0).sleep();

        // goFront(8);
        // ros::Duration(5.0).sleep();

        // turnRight(8);
        // ros::Duration(5.0).sleep();

        // goBack(16);
        // ros::Duration(5.0).sleep();
        
        /*
        ROS_INFO("[WAMV] - Aceleration \t\t=>\t X = %.2f, Y = %.2f, Z = %.2f", acceleration_x_, acceleration_y_, acceleration_z_);
        ROS_INFO("[WAMV] - Angular Velocity \t=>\t X = %.2f, Y = %.2f, Z = %.2f", angular_velocity_x_, angular_velocity_y_, angular_velocity_z_);
        ROS_INFO("[WAMV] - Orientation  \t\t=>\t X = %.2f, Y = %.2f, Z = %.2f", orientation_x_, orientation_y_, orientation_z_);
        */    }    

    //  ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗         ████████╗██╗  ██╗██████╗ ██╗   ██╗███████╗████████╗███████╗██████╗ ███████╗
    // ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║         ╚══██╔══╝██║  ██║██╔══██╗██║   ██║██╔════╝╚══██╔══╝██╔════╝██╔══██╗██╔════╝
    // ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║            ██║   ███████║██████╔╝██║   ██║███████╗   ██║   █████╗  ██████╔╝███████╗
    // ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║            ██║   ██╔══██║██╔══██╗██║   ██║╚════██║   ██║   ██╔══╝  ██╔══██╗╚════██║
    // ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗       ██║   ██║  ██║██║  ██║╚██████╔╝███████║   ██║   ███████╗██║  ██║███████║
    //  ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝       ╚═╝   ╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝
                                                                                                                                            
    void wamv::controlThrustLeft(std_msgs::Float32 force, std_msgs::Float32 angle){
        leftThrust.publish(force);
        leftAngle.publish(angle);
    }

    void wamv::controlThrustRight(std_msgs::Float32 force, std_msgs::Float32 angle){
        rightThrust.publish(force);
        rightAngle.publish(angle);
    }

    //  ██████╗  ██████╗       ██████╗ ██╗██████╗ ███████╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗
    // ██╔════╝ ██╔═══██╗      ██╔══██╗██║██╔══██╗██╔════╝██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║
    // ██║  ███╗██║   ██║█████╗██║  ██║██║██████╔╝█████╗  ██║        ██║   ██║██║   ██║██╔██╗ ██║
    // ██║   ██║██║   ██║╚════╝██║  ██║██║██╔══██╗██╔══╝  ██║        ██║   ██║██║   ██║██║╚██╗██║
    // ╚██████╔╝╚██████╔╝      ██████╔╝██║██║  ██║███████╗╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║
    //  ╚═════╝  ╚═════╝       ╚═════╝ ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝
                                                                                            
    void wamv::goFront(int movementTime){
        std_msgs::Float32 forceThrusterRight;
        std_msgs::Float32 forceThrusterLeft;
        std_msgs::Float32 angleThrusterRight;
        std_msgs::Float32 angleThrusterLeft;
        
        forceThrusterRight.data = 1.0f;
        angleThrusterRight.data = 0.0f;
        forceThrusterLeft.data = 1.0f;
        angleThrusterLeft.data = 0.0f;
        
        for (int i = 0; i < movementTime; i++)
        {
            controlThrustLeft(forceThrusterLeft, angleThrusterLeft);
            controlThrustRight(forceThrusterRight, angleThrusterRight);
            ros::Duration(1.0).sleep();
        }
    }

    void wamv::goBack(int movementTime){
        std_msgs::Float32 forceThrusterRight;
        std_msgs::Float32 forceThrusterLeft;
        std_msgs::Float32 angleThrusterRight;
        std_msgs::Float32 angleThrusterLeft;
        
        forceThrusterRight.data = -1.0f;
        angleThrusterRight.data = 0.0f;
        forceThrusterLeft.data = -1.0f;
        angleThrusterLeft.data = 0.0f;
        
        for (int i = 0; i < movementTime; i++)
        {
            controlThrustLeft(forceThrusterLeft, angleThrusterLeft);
            controlThrustRight(forceThrusterRight, angleThrusterRight);
            ros::Duration(1.0).sleep();
        }
    }
    void wamv::turnRight(int movementTime){
        std_msgs::Float32 forceThrusterRight;
        std_msgs::Float32 forceThrusterLeft;
        std_msgs::Float32 angleThrusterRight;
        std_msgs::Float32 angleThrusterLeft;
    
        forceThrusterRight.data = -0.2f;
        angleThrusterRight.data = 0.0f;
        forceThrusterLeft.data = 0.2f;
        angleThrusterLeft.data = 0.0f;
        
        for (int i = 0; i < movementTime; i++)
        {
            controlThrustLeft(forceThrusterLeft, angleThrusterLeft);
            controlThrustRight(forceThrusterRight, angleThrusterRight);
            ros::Duration(1.0).sleep();
        }
    }
    void wamv::turnLeft(int movementTime){
        std_msgs::Float32 forceThrusterRight;
        std_msgs::Float32 forceThrusterLeft;
        std_msgs::Float32 angleThrusterRight;
        std_msgs::Float32 angleThrusterLeft;
        
        forceThrusterRight.data = 0.1f;
        angleThrusterRight.data = 0.0f;
        forceThrusterLeft.data = -0.1f;
        angleThrusterLeft.data = 0.0f;
        
        for (int i = 0; i < movementTime; i++)
        {
            controlThrustLeft(forceThrusterLeft, angleThrusterLeft);
            controlThrustRight(forceThrusterRight, angleThrusterRight);
            ros::Duration(1.0).sleep();
            ROS_INFO("[WAMV] - Orientation  \t\t=>\t Angle = %.2f", orientation_angle_);
        }
    }
}

PLUGINLIB_EXPORT_CLASS(wamv_usv::wamv, nodelet::Nodelet);
    
