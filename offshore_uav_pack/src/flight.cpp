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