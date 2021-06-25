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


        // motor_left_thrust_ = 0.0;

        // std::lock_guard mutex_motors_;

        // {
        //     std::scoped_lock lock(mutex_motors_);


        // }

        // timer_pub_motors_ = 

        // pub_motor_left_thrust_ = ;
        // pub_motor_right_thrust_ = ;

        // pub_motor_left_angle_ = ;
        // pub_motor_right_angle_ = ;

        // | -------------- initialize Motor  ----------------- |
        rightThrust = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1);
        rightAngle = nh.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 1);
        leftThrust = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1);
        leftAngle = nh.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 1);
        
        
        ROS_INFO_ONCE("[WAMV] - Starting Program!");
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
        
        heading_rad = mrs_lib::AttitudeConverter(msg->orientation).getHeading();
        heading_degree = heading_rad * (180/PI);
        // ROS_INFO("Angle = %.2f", heading_rad);
        
        acceleration_x_ = msg->linear_acceleration.x;
        acceleration_y_ = msg->linear_acceleration.y;
        acceleration_z_ = msg->linear_acceleration.z;

        angular_velocity_x_ = msg->angular_velocity.x;
        angular_velocity_y_ = msg->angular_velocity.y;
        angular_velocity_z_ = msg->angular_velocity.z;
    }

    void wamv::p3dInCallback(const nav_msgs::Odometry::ConstPtr &msg){
        double heading = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
        
        position_x_ = msg->pose.pose.position.x;
        position_y_ = msg->pose.pose.position.y;
        position_z_ = msg->pose.pose.position.z;
        // ROS_INFO("Position X[%.2f], y[%.2f], z[%.2f]", position_x_, position_y_, position_z_);

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
        
        gotoGlobal(30.0, 30.0);
        gotoGlobal(30.0, 0.0);
        gotoGlobal(0.0, 0.0);
        gotoGlobal(10.0, 0.0);
        gotoGlobal(0.0, 0.0);
        gotoGlobal(0.0, 10.0);
        gotoGlobal(0.0, 0.0);


        // turnAngle(-1.57);
        // turnAngle(1.57);

        // turnFrontAngle(-1.57);
        // turnFrontAngle(1.57);

        // turnLeft(1);
        // ros::Duration(5.0).sleep();

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
        */    
    }    

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
        forceThrusterRight.data = -1.0f;
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
    void wamv::turnAngle(float angle){
        bool moving = true;
        double force;
        double angleT;
        float dAngle =  angle - heading_rad;
        
        if(!(angle > -3.14 && angle < 3.14)){
            return;
        }
        if(fabs(dAngle) < 0.01){
                moving = false;
                return;
        }
        while(moving){
            dAngle =  angle - heading_rad;
            force = sqrt(fabs((dAngle)/PI));
            angleT;
            if ((dAngle) > PI/4){
                angleT = -PI/4;
            }
            else if ((dAngle) < - PI/4) {
                angleT = PI/4;
            }
            else if ((dAngle) > 0){
                if(sqrt(fabs(dAngle)) > PI/4){
                    angleT = -PI/4;
                }
                else{
                    angleT = -(sqrt(fabs(dAngle)));
                }
            }
            else if ((dAngle) < 0){
                if(sqrt(fabs(dAngle)) > PI/4){
                    angleT = PI/4;
                }
                else{
                    angleT = sqrt(fabs(dAngle));
                }
            }
            // ROS_INFO("Angle R [%.2f] L[%.2f]" , angleT, angleT );
            // ROS_INFO("Angle H [%.2f] A[%.2f]" , (heading_rad), (angle)  );
            if(angleT < 0){
                // ROS_INFO("Force R [%.2f] L[%.2f]" , force*0.55, -force );
                forceThrusterRight.data = force*0.55;
                forceThrusterLeft.data = -force;
            }
            else{
                // ROS_INFO("Force R [%.2f] L[%.2f]" , -force, force*0.55 );
                forceThrusterRight.data = -force;
                forceThrusterLeft.data = force*0.55;
            }
            angleThrusterRight.data = angleT;
            angleThrusterLeft.data = angleT;    
            controlThrustLeft(forceThrusterLeft, angleThrusterLeft);
            controlThrustRight(forceThrusterRight, angleThrusterRight);
            if(fabs(dAngle) < 0.005){
                moving = false;
            }
        }
    }
    void wamv::turnFrontAngle(float angle){
        bool moving = true;
        double force;
        double angleT;
        float dAngle =  angle - heading_rad;
        
        if(!(angle > -3.14 && angle < 3.14)){
            return;
        }
        if(fabs(dAngle) < 0.01){
                moving = false;
                return;
        }
        while(moving){
            dAngle =  angle - heading_rad;
            force = sqrt(fabs((dAngle)/PI));
            if ((dAngle) > PI/2){
                angleT = -PI/2;
            }
            else if ((dAngle) < - PI/2) {
                angleT = PI/2;
            }
            else if ((dAngle) > 0){
                if(sqrt(fabs(dAngle)) > PI/2){
                    angleT = -PI/2;
                }
                else{
                    angleT = -(sqrt(fabs(dAngle)));
                }
            }
            else if ((dAngle) < 0){
                if(sqrt(fabs(dAngle)) > PI/2){
                    angleT = PI/2;
                }
                else{
                    angleT = sqrt(fabs(dAngle));
                }
            }
            // ROS_INFO("Force R [%.2f] L[%.2f]" , force/2, force/2 );
            // ROS_INFO("Angle R [%.2f] L[%.2f]" , angleT, angleT );
            // ROS_INFO("Angle H [%.2f] A[%.2f]" , (heading_rad), (angle)  );
            forceThrusterRight.data = force/2;
            forceThrusterLeft.data = force/2;
            angleThrusterRight.data = angleT;
            angleThrusterLeft.data = angleT;    
            controlThrustLeft(forceThrusterLeft, angleThrusterLeft);
            controlThrustRight(forceThrusterRight, angleThrusterRight);
            if(fabs(dAngle) < 0.005){
                moving = false;
            }
        }
    }
    void wamv::turnLeft(int movementTime){
        forceThrusterRight.data = 0.5f;
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
    void wamv::gotoGlobal(float x, float y){
        float totalDistance;
        float angle;
        float forceTurn;
        float forceFront;
        double angleT;
        float dAngle;
        float surplusDistance;
        bool moving = true;
        totalDistance = sqrt(pow((position_x_ - x),2) + pow((position_y_- y),2));

        if (totalDistance < 0.05){
            moving = false;
        }

        angle = atan2((y - position_y_),(x - position_x_));
        dAngle =  angle - heading_rad;
        ROS_INFO("Angle A[%.2f] D[%.2f]" , angle, dAngle);
        turnAngle(angle);

        while(moving){
            surplusDistance = sqrt(pow((position_x_ - x),2) + pow((position_y_- y),2));
            angle = atan2((y - position_y_),(x - position_x_));
            dAngle =  angle - heading_rad;
            if(surplusDistance < (totalDistance*0.5)){
                forceFront = (fabs(surplusDistance/(totalDistance*0.5)))*0.5;
            }
            else{
                forceFront = 0.5;
            }
            forceTurn = (fabs((dAngle)/PI));
            if ((dAngle) > PI/4){
                angleT = -PI/4;
            }
            else if ((dAngle) < - PI/4) {
                angleT = PI/4;
            }
            else if ((dAngle) > 0){
                if(sqrt(fabs(dAngle)) > PI/4){
                    angleT = -PI/4;
                }
                else{
                    angleT = -(sqrt(fabs(dAngle)));
                }
            }
            else if ((dAngle) < 0){
                if(sqrt(fabs(dAngle)) > PI/4){
                    angleT = PI/4;
                }
                else{
                    angleT = sqrt(fabs(dAngle));
                }
            }

            // ROS_INFO("Distance T [%.2f] S[%.2f]" , totalDistance, surplusDistance );
            ROS_INFO("X[%.2f], y[%.2f], D[%.2f], FR [%.2f], FL[%.2f], A[%.2f]", 
                    position_x_, 
                    position_y_, 
                    surplusDistance, 
                    sqrt(forceFront + (0.2 * sin(dAngle))), 
                    sqrt(forceFront - (0.2 * sin(dAngle))), 
                    dAngle );

            // ROS_INFO("Force F [%.2f] T[%.2f]" , forceFront, forceTurn/2);
            // ROS_INFO("Angle T[%.2f] D[%.2f]" , angleT, dAngle);

            forceThrusterRight.data = sqrt(forceFront + (0.2 * sin(dAngle)));
            forceThrusterLeft.data =  sqrt(forceFront - (0.2 * sin(dAngle)));
            angleThrusterRight.data = 0.0f;
            angleThrusterLeft.data = 0.0f;
            controlThrustLeft(forceThrusterLeft, angleThrusterLeft);
            controlThrustRight(forceThrusterRight, angleThrusterRight);
            if (surplusDistance < 0.7){
                moving = false;
            }
        }
    }
}

PLUGINLIB_EXPORT_CLASS(wamv_usv::wamv, nodelet::Nodelet);
    
