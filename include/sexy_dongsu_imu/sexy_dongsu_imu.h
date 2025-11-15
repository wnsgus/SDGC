#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>
#include <std_srvs/Empty.h>
#define SERIAL_PORT		"/dev/IMU"
#define SERIAL_SPEED		B115200


typedef struct IMU_DATA
{
	double dQuaternion_x = 0.0;
	double dQuaternion_y = 0.0;
	double dQuaternion_z = 0.0;
	double dQuaternion_w = 1.0;

	double dAngular_velocity_x = 0.0;
	double dAngular_velocity_y = 0.0;
	double dAngular_velocity_z = 0.0;
	
	double dLinear_acceleration_x = 0.0;
	double dLinear_acceleration_y = 0.0;
	double dLinear_acceleration_z = 0.0;
    
	double dEuler_angle_Roll = 0.0;
	double dEuler_angle_Pitch = 0.0;
	double dEuler_angle_Yaw = 0.0;

}IMU_DATA;
IMU_DATA _pIMU_data;

int serial_fd = -1;
double time_offset_in_seconds;
double dSend_Data[10];
double m_dRoll, m_dPitch, m_dYaw;
sensor_msgs::Imu imu_data_msg;
//tf_prefix add
std::string tf_prefix_;
//single_used TF
bool m_bSingle_TF_option = false;



int serial_open ()
{
	printf ("Try to open serial: %s\n", SERIAL_PORT); 

	serial_fd = open(SERIAL_PORT, O_RDWR|O_NOCTTY);
	if (serial_fd < 0) {
		printf ("Error unable to open %s\n", SERIAL_PORT);
		return -1;
	}
  	printf ("%s open success\n", SERIAL_PORT);

  	struct termios tio;
  	tcgetattr(serial_fd, &tio);
  	cfmakeraw(&tio);
	tio.c_cflag = CS8|CLOCAL|CREAD;
  	tio.c_iflag &= ~(IXON | IXOFF);
  	cfsetspeed(&tio, SERIAL_SPEED);
  	tio.c_cc[VTIME] = 0;
  	tio.c_cc[VMIN] = 0;

  	int err = tcsetattr(serial_fd, TCSAFLUSH, &tio);
  	if (err != 0) 
	{
    	printf ("Error tcsetattr() function return error\n");
    	close(serial_fd);
		serial_fd = -1;
    	return -1;
  	}
	return 0;
}

static unsigned long GetTickCount() 
{
    struct timespec ts;
   
    clock_gettime (CLOCK_MONOTONIC, &ts);

    return ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

int SendRecv(const char* command, double* returned_data, int data_length)
{
	#define COMM_RECV_TIMEOUT	30	

	char temp_buff[256];
	read (serial_fd, temp_buff, 256);

	int command_len = strlen(command);
	int n = write(serial_fd, command, command_len);

	if (n < 0) return -1;

	const int buff_size = 1024;
	int  recv_len = 0;
	char recv_buff[buff_size + 1];

	unsigned long time_start = GetTickCount();

	while (recv_len < buff_size) {
		int n = read (serial_fd, recv_buff + recv_len, buff_size - recv_len);
		if (n < 0) 
		{
			return -1;
		}
		else if (n == 0) 
		{
			usleep(1000);
		}
		else if (n > 0) 
		{
			recv_len += n;

			if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n') 
			{
				break;
			}
		}

		unsigned long time_current = GetTickCount();
		unsigned long time_delta = time_current - time_start;

		if (time_delta >= COMM_RECV_TIMEOUT) break;
	}
	recv_buff[recv_len] = '\0';

	if (recv_len > 0) 
	{
		if (recv_buff[0] == '!') 
		{
			return -1;
		}
	}

	if (strncmp(command, recv_buff, command_len - 1) == 0) {
		if (recv_buff[command_len - 1] == '=') {
			int data_count = 0;

			char* p = &recv_buff[command_len];
			char* pp = NULL;

			for (int i = 0; i < data_length; i++) 
			{
				if (p[0] == '0' && p[1] == 'x') 
				{
					returned_data[i] = strtol(p+2, &pp, 16);
					data_count++;
				}
				else 
				{
					returned_data[i] = strtod(p, &pp);
					data_count++;
				}

				if (*pp == ',') 
				{
					p = pp + 1;
				}
				else 
				{
					break;
				}
			}
			return data_count;
		}
	}
	return 0;
}

void my_handler(sig_atomic_t s)
{
    printf("Caught signal %d\n",s);
    exit(1); 
}

bool All_Data_Reset_Command(std_srvs::Empty::Request  &req, 
					    	std_srvs::Empty::Response &res)
{
    double dSend_Data[10];
	SendRecv("rc\n", dSend_Data, 10);
	return true;
}

bool Euler_Angle_Init_Command(std_srvs::Empty::Request  &req, 
					    	  std_srvs::Empty::Response &res)
{
    double dSend_Data[10];
	SendRecv("za\n", dSend_Data, 10);

	return true;
}

bool Euler_Angle_Reset_Command(std_srvs::Empty::Request  &req, 
					    	   std_srvs::Empty::Response &res)
{
    double dSend_Data[10];
	SendRecv("ra\n", dSend_Data, 10);

	return true;
}

bool Pose_Velocity_Reset_Command(std_srvs::Empty::Request  &req, 
					    	     std_srvs::Empty::Response &res)
{
    double dSend_Data[10];
	SendRecv("rp\n", dSend_Data, 10);
	return true;
}

bool Reboot_Sensor_Command(std_srvs::Empty:Request  &req, 
					       std_srvs::Empty:Response &res)
{
	bool bResult = false;

    double dSend_Data[10];
	SendRecv("rd\n", dSend_Data, 10);

    bResult = true;
	res.command_Result = bResult;
	return true;
}
