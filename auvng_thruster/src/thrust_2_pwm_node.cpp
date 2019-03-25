#include <ros/ros.h>
#include <hg_ros_pololu/Pwm.h>

ros::Publisher pub;


// msg.header.stamp = ros::Time::now();



int main(int argc, char **argv)
{
    ros::init(argc, argv, "auvng_controller");
    ros::NodeHandle n;

    pub = n.advertise<hg_ros_pololu::Pwm>("pwm", 100);

    hg_ros_pololu::Pwm pwm;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        pwm.pwm = {1500,15000,1500,1500,1500,1500,1500,1500};

        pub.publish(pwm);

        ROS_INFO("Spinning node");

        ros::spinOnce();

        loop_rate.sleep();
    }
    

    return 0;
}