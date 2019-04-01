#include <auvng_controller/pid.h>
ros::Publisher pub;

double K_p[6] = {0,0,0,0,0,0};
double K_i[6] = {0,0,0,0,0,0};
double K_d[6] = {0,0,0,0,0,0};
double current_state[6] = {0,0,0,0,0,0};
double set_point[6] = {0,0,0,0,0,0};
double error[3][6] = {{0,0,0,0,0,0},
                      {0,0,0,0,0,0},
                      {0,0,0,0,0,0}};
double div_error[6] = {0,0,0,0,0,0};
double int_error[6] = {0,0,0,0,0,0};
double control_effort[6] = {0,0,0,0,0,0};
bool control_enabled = 0;



void set_point_callback(geometry_msgs::TwistStamped twist)
{
    set_point[0] = twist.twist.linear.x;
    set_point[1] = twist.twist.linear.y;
    set_point[2] = twist.twist.linear.z;
    set_point[3] = twist.twist.angular.x;
    set_point[4] = twist.twist.angular.y;
    set_point[5] = twist.twist.angular.z;

    // printf("get_new_set_point \n");
}

void current_state_callback(nav_msgs::Odometry odom)
{
    tf::Quaternion quaternion(odom.pose.pose.orientation.x,
                              odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z,
                              odom.pose.pose.orientation.w);
    tfScalar roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    printf("current :: roll: %lf   pitch: %lf    yaw: %lf \n", (roll*180)/3.14, (pitch*180)/3.14, (yaw*180)/3.14);

    current_state[0] = odom.pose.pose.position.x;
    current_state[1] = odom.pose.pose.position.x;
    current_state[2] = odom.pose.pose.position.x;
    current_state[3] = roll;
    current_state[4] = pitch;
    current_state[0] = yaw;

    // printf("get_new_state \n");
}

void enable_control_callback(std_msgs::Bool control_enable_msg)
{
    control_enabled = control_enable_msg.data;
    // printf("get_control_enable \n");
}

void dynamicCallback(auvng_controller::pid_paramConfig &config, uint32_t level)
{

    K_p[0] = config.K_p_x;
    K_p[1] = config.K_p_y;
    K_p[2] = config.K_p_z;
    K_p[3] = config.K_p_roll;
    K_p[4] = config.K_p_pitch;
    K_p[5] = config.K_p_yaw;

    K_i[0] = config.K_i_x;
    K_i[1] = config.K_i_y;
    K_i[2] = config.K_i_z;
    K_i[3] = config.K_i_roll;
    K_i[4] = config.K_i_pitch;
    K_i[5] = config.K_i_yaw;

    K_d[0] = config.K_d_x;
    K_d[1] = config.K_d_y;
    K_d[2] = config.K_d_z;
    K_d[3] = config.K_d_roll;
    K_d[4] = config.K_d_pitch;
    K_d[5] = config.K_d_yaw;

    ROS_INFO("Get param: [%lf]", K_p[0]);
    // printf("get param !!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auvng_controller");
    ros::NodeHandle n;

    pub = n.advertise<std_msgs::Float64>("z_val", 1000);

    dynamic_reconfigure::Server<auvng_controller::pid_paramConfig> server;
    dynamic_reconfigure::Server<auvng_controller::pid_paramConfig>::CallbackType f;

    ros::Subscriber current_sub = n.subscribe("auvng/state", 1000, current_state_callback);
    ros::Subscriber set_point_sub = n.subscribe("auvng/set_point", 1000, set_point_callback);
    ros::Subscriber enable_control = n.subscribe("auvng/enable_control", 1000, enable_control_callback);
    f = boost::bind(&dynamicCallback, _1, _2);
    server.setCallback(f);

    ros::Rate rate(20);

    while (n.ok())
    {
        do_calculate();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}