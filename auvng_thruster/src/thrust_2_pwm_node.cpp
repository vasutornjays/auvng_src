#include <ros/ros.h>
#include <hg_ros_pololu/Pwm.h>
#include <auvng_thruster/thruster_torque.h>
#include <math.h>

hg_ros_pololu::Pwm pwm;

ros::Publisher pub;

int torque_to_pwm(double t){
    if(t < -0.0272155){
        return int((((0.003687225246674*pow(t,3)) + (0.027256336314212*pow(t,2))
           +(0.138696210404033*t) + 1.466875275577851)) * 1000);
    } else if (t > 0.0589670){
        return int((((0.001239041850837*pow(t,3)) + (-0.013249188582324*pow(t,2))
           +(0.108242941336985*t) + 1.532708016436602)) * 1000);
    } else {
        return 1500;
    }
}

void callback(auvng_thruster::thruster_torque t)
{
    pwm.pwm = {1500,15000,1500,1500,1500,1500,1500,1500};

    for(int i=0; i<8;i++){
        pwm.pwm[i] = torque_to_pwm(t.thruster_torque[i]);
    }
    

    pub.publish(pwm);
    ROS_INFO("pwm[0]: [%d]", pwm.pwm[0]);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "auvng_controller");
    ros::NodeHandle n;

    pub = n.advertise<hg_ros_pololu::Pwm>("pwm", 100);
    ros::Subscriber sub = n.subscribe("thruster_torque", 1000, callback);

    ROS_INFO("Spinning node");
    ros::spin();
    

    return 0;
}