#include "um232_master_i2c.h"
#include "ros/ros.h"

#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A


u_int16_t g_p_sens;
u_int16_t g_p_offset;
u_int16_t g_tcs;
u_int16_t g_tco;
u_int16_t g_t_ref;
u_int16_t g_t_sens;

u_int16_t C[8];
u_int32_t D1, D2;
u_int32_t TEMP;
u_int32_t P;
u_int8_t _model;

float fluidDensity;



int main(int argc, char **argv){

    InitUm232I2c();

    for ( u_int8_t i = 0 ; i < 7 ; i++ ) {
        u_int16_t temp = ReadPROM(i*2);
		C[i] = temp;
        printf("C[%d] : %d\n",i, C[i]);
	}

    D1 = ReadADC(MS5837_CONVERT_D1_8192);
    D2 = ReadADC(MS5837_CONVERT_D2_8192);

    printf("D1 : %d\n",D1);
    printf("D2 : %d\n",D2);

    // ros::init(argc, argv, "pressure_sensor_pub");
    // ros::NodeHandle n;
    // ros::Publisher pressure_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    // ros::Rate r(100.0);

    // InitUm232I2c();
    // g_pressure_sens = ReadPressureSensitivity();
    // printf("Pressure sensitivity : %d\n", sens);
    // g_pressure_sens = ReadPressureOffset();
    // printf("Pressure offset : %d\n", off);

    // while (n.ok())
    // {   

    //     nav_msgs::Odometry odom;
    //     current_time = ros::Time::now();
    //     odom.header.stamp = current_time;
    //     odom.header.frame_id = "odom";
    //     odom.child_frame_id = "base_link";
    //     odom.pose.pose.position.z = 0.0;
    //     r.sleep();
    // }

    
    return 0;
}