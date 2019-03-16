#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <auvng_controller/pid_paramConfig.h>
#include <std_msgs/Float64.h>


double K_p_x;

ros::Publisher pub;


void callback(nav_msgs::Odometry odom)
{
    double current_x = odom.pose.pose.position.x;
    double current_y = odom.pose.pose.position.y;
    double current_z = odom.pose.pose.position.z;

    std_msgs::Float64 pub_z;

    pub_z.data = current_z * K_p_x;
    
    pub.publish(pub_z);

    ROS_INFO("Output z: [%lf]", pub_z.data);
    // printf("%lf",current_z);
}

void dynamicCallback(auvng_controller::pid_paramConfig &config, uint32_t level){

    K_p_x = config.K_p_x;
    ROS_INFO("Get param: [%lf]", K_p_x);
    // printf("get param !!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auvng_controller");
    ros::NodeHandle n;
    
    pub = n.advertise<std_msgs::Float64>("z_val", 1000);

    dynamic_reconfigure::Server<auvng_controller::pid_paramConfig> server;
    dynamic_reconfigure::Server<auvng_controller::pid_paramConfig>::CallbackType f;

    ros::Subscriber sub = n.subscribe("auvng/state", 1000, callback);
    f = boost::bind(&dynamicCallback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();

    return 0;
}