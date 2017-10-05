#include <stdio.h>
#include <string.h>
#include "serial.h"
#include "crc32.h"
#include "protocal_uart_sdk.h"
#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3Stamped.h>

#define CAMERA_PAIR_NUM 5
#define UART_PORT 3 

ros::Publisher obstacle_distance_pub;
ros::Publisher velocity_pub;
ros::Publisher ultrasonic_pub;



int callback ()
{
	if ( connect_serial( UART_PORT ) < 0 )
	{
        printf( "connect serial error\n" );
        return 0;
	}
	
	for ( int i = 0; i < 1000; ++i )
	{
		unsigned char data[1000] = {0};
		int max_size = (int)sizeof(data);
		int timeout = 1000;
		int n = read_serial( data, max_size, timeout);
		
		if( n <= 0 )
		{
			continue;
		}

		push( data, sizeof(data) );
		for ( ; ; )
		{
			unsigned int len = 0;
			int has_packet = pop( data, len );
			if ( has_packet )
			{
				if ( len )
				{
					unsigned char cmd_id = data[1];
					
					//printf("event id %u\n",cmd_id);
					
					//no image data posted via UART
/*					if(cmd_id==e_imu){
						printf("imu data received\n");
					}
					else if(cmd_id==e_ultrasonic){
						printf("ultrasonic data received\n");
					}
					else if(cmd_id==e_velocity){
						printf("velocity data received\n");
					}
					else if(cmd_id==e_obstacle_distance){
						printf("obstacle data received\n");
					}
					else if(cmd_id==e_motion){
						printf("motion data received\n");
					}*/
							
					if ( e_imu == cmd_id )
					{
						imu imu_data;
						memcpy( &imu_data, data + 2, sizeof(imu_data) );
/*						printf( "imu:%f %f %f,%f %f %f %f\n", imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, 
							     imu_data.q[0], imu_data.q[1], imu_data.q[2], imu_data.q[3] );*/
						/*printf( "frame index:%d,stamp:%d\n", imu_data.frame_index, imu_data.time_stamp );
						printf( "\n" );*/
					}
					
					if ( e_ultrasonic == cmd_id )
					{
						ultrasonic_data ultrasonic;
						memcpy( &ultrasonic, data + 2, sizeof(ultrasonic) );

						sensor_msgs::LaserScan ultrasonic_dists;
						ultrasonic_dists.ranges.resize(CAMERA_PAIR_NUM);
						ultrasonic_dists.intensities.resize(CAMERA_PAIR_NUM);						
						ultrasonic_dists.header.frame_id  = "guidance_uart";
						ultrasonic_dists.header.stamp	 = ros::Time::now();					
						
						for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
						{
							ultrasonic_dists.ranges[d] = 0.001f * ultrasonic.ultrasonic[d];
							ultrasonic_dists.intensities[d] = 1.0 * ultrasonic.reliability[d];
							//printf( "distance:%f,reliability:%d\n", ultrasonic.ultrasonic[d] * 0.001f, (int)ultrasonic.reliability[d] );
						}
						ultrasonic_pub.publish(ultrasonic_dists);						
						
						/*printf( "frame index:%d,stamp:%d\n", ultrasonic.frame_index, ultrasonic.time_stamp );
						printf( "\n" );*/
					}
					
					if ( e_velocity == cmd_id )
					{
						velocity vo;
						geometry_msgs::Vector3Stamped vec_temp;
						vec_temp.header.frame_id  = "guidance_uart";
						vec_temp.header.stamp	 = ros::Time::now();

						soc2pc_vo_can_output output;
						memcpy( &output, data + 2, sizeof(vo) );
						vec_temp.vector.x = vo.vx = 0.001f * output.m_vo_output.vx ;
						vec_temp.vector.y = vo.vy = 0.001f * output.m_vo_output.vy ;
						vec_temp.vector.z = vo.vz = 0.001f * output.m_vo_output.vz ;
						velocity_pub.publish(vec_temp);							
						
						//printf( "Velocities vx:%f vy:%f vz:%f\n", 0.001f * vo.vx, 0.001f * vo.vy, 0.001f * vo.vz );
/*						printf( "frame index:%d,stamp:%d\n", vo.frame_index, vo.time_stamp );
						printf( "\n" );*/
					}
					
					if ( e_obstacle_distance == cmd_id )
					{
						obstacle_distance oa;
						memcpy( &oa, data + 2, sizeof(oa) );
						sensor_msgs::LaserScan obstacle_dists;
						obstacle_dists.ranges.resize(CAMERA_PAIR_NUM);
						obstacle_dists.header.frame_id  = "guidance_uart";
						obstacle_dists.header.stamp	 = ros::Time::now();
						
						/**
						* obstacle_distance
						* Define obstacle distance calculated by fusing vision and ultrasonic sensors. Unit is `cm`.
						*/
						
						//printf( "obstacle distance:" );
						for ( int direction = 0; direction < CAMERA_PAIR_NUM; ++direction )
						{
							obstacle_dists.ranges[direction] = 0.01f * oa.distance[direction] ;
							//printf( " %f ", 0.01f * oa.distance[direction] );
						}
						obstacle_distance_pub.publish(obstacle_dists);
						
						/*printf( "\n" );
						printf( "frame index:%d,stamp:%d\n", oa.frame_index, oa.time_stamp );
						printf( "\n" );*/
					}
					
					if( e_motion == cmd_id )
					{
						motion mo;
						memcpy( &mo, data + 2, sizeof(mo));
												
/*						printf("corresponding_imu_index %d\n",mo.corresponding_imu_index);
						printf("q0 %f q1 %f q2 %f q3 %f \n",mo.q0,mo.q1,mo.q2,mo.q3);
						printf("attitude status %d \n",mo.attitude_status);
						printf("positions in global %f %f %f \n",mo.position_in_global_x,mo.position_in_global_y,mo.position_in_global_z);
						printf("position status %d\n",mo.position_status);
						printf("velocity_in_global_x  %f %f %f \n",mo.velocity_in_global_x,mo.velocity_in_global_y,mo.velocity_in_global_z);
						printf("velocity_status  %d\n",mo.velocity_status);

						printf("reserve_int %d %d %d %d\n ",mo.reserve_int[0],mo.reserve_int[1],mo.reserve_int[2],mo.reserve_int[3]);
						printf("uncertainty_location %f %f %f %f \n", mo.uncertainty_location[0],mo.uncertainty_location[1],mo.uncertainty_location[2],mo.uncertainty_location[3]);
						printf("uncertainty_velocity %f %f %f %f \n", mo.uncertainty_velocity[0],mo.uncertainty_velocity[1],mo.uncertainty_velocity[2],mo.uncertainty_velocity[3]);
							*/	
						
					}
					
				}
				else
				{
					printf( "err\n" );
				}
			}
			else
			{
				break;
			}
		}
	}

	disconnect_serial();

	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "guidance_uart");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);
	
	velocity_pub  			= n.advertise<geometry_msgs::Vector3Stamped>("/guidance_uart/velocity",1);
	obstacle_distance_pub	= n.advertise<sensor_msgs::LaserScan>("/guidance_uart/obstacle_distance",1);
	ultrasonic_pub			= n.advertise<sensor_msgs::LaserScan>("/guidance_uart/ultrasonic",1);
	
	callback();
	ros::spinOnce();

	return 0;
}

