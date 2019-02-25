#include "um232_master_i2c.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A
#define OFFSET_PRESSURE           101300


u_int16_t g_p_sens;
u_int16_t g_p_offset;
u_int16_t g_tcs;
u_int16_t g_tco;
u_int16_t g_t_ref;
u_int16_t g_t_sens;

u_int16_t C[8];
u_int32_t TEMP;
u_int32_t P;
u_int8_t _model;

int32_t dT = 0;
int64_t SENS = 0;
int64_t OFF = 0;
int32_t SENSi = 0;
int32_t OFFi = 0;  
int32_t Ti = 0;
int64_t OFF2 = 0;
int64_t SENS2 = 0;
u_int32_t D1, D2;

float fluidDensity = 1029;

uint8_t Crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}

double CalculateDepth(){


    D1 = ReadADC(MS5837_CONVERT_D1_8192);
    D2 = ReadADC(MS5837_CONVERT_D2_8192);
	
	// Terms called
	dT = D2-uint32_t(C[5])*256l;
	SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
	OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
	P = (D1*SENS/(2097152l)-OFF)/(8192l);
	
	// Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;
	

    if((TEMP/100)<20){         //Low temp
        Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
        OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
        SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
        if((TEMP/100)<-15){    //Very low temp
            OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
            SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
        }
    }
    else if((TEMP/100)>=20){    //High temp
        Ti = 2*(dT*dT)/(137438953472LL);
        OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
        SENSi = 0;
    }

	
	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	TEMP = (TEMP-Ti);
	P = (((D1*SENS2)/2097152l-OFF2)/8192l)/10;

    double pressure = P * 100.0f;
    //double pressure = P * 0.001f; 
    return (pressure - OFFSET_PRESSURE)/(fluidDensity*9.80665);
    //return (pressure - 1.0) ;
}



int main(int argc, char **argv){

    ros::init(argc, argv, "pressure_sensor_pub");
    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    int dt;
    int temp;
    int off;
    int sens;
    int p;
    double depth;
    ros::Time current_time;
    

    InitUm232I2c();

    for ( u_int8_t i = 0 ; i < 7 ; i++ ) {
        u_int16_t temp = ReadPROM(i*2);
		C[i] = temp;
        printf("C[%d] : %d\n",i, C[i]);
	}

    uint8_t crcRead = C[0] >> 12;
    uint8_t crcCalculated = Crc4(C);

    if ( crcCalculated == crcRead ) {
        printf("###### Initialization success ######\n"); // Initialization success
    } else {
        printf("Fail to initial");
    }

    ros::Rate r(10);

    while(n.ok()){
        depth = CalculateDepth();
        nav_msgs::Odometry odom;
        current_time = ros::Time::now();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.z = depth;
        printf("depth: %f\n", depth);
        odom_pub.publish(odom);

        r.sleep();
        //printf("D2 : %d\n",D2);
    }
    
    return 0;
}