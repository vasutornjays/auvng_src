#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

class JoyTeleop
{
    public:
       JoyTeleop();

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);        

        ros::NodeHandle nh_;
        ros::Time current_time;
        double l_scale_ = 1;
        double a_scale_ = 1;
        ros::Publisher vel_pub_;
        ros::Subscriber joy_sub_;
};

JoyTeleop::JoyTeleop()
{

  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("joy/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTeleop::joyCallback, this);
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    geometry_msgs::TwistStamped twist;
    current_time = ros::Time::now();
    twist.header.stamp = current_time;
    if(joy->buttons[4] == 1){
        twist.twist.linear.x = l_scale_*joy->axes[1];
        twist.twist.linear.y = -l_scale_*joy->axes[0];
    }
    vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  JoyTeleop joy_teleop;

  ros::spin();
}
