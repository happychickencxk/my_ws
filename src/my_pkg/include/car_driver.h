#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <thread>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <serial/serial.h>
#include <mutex>
#include <atomic>
#include <vector> 
#include <cmath>


// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include "my_pkg/PID_reconfigConfig.h"
#include <dynamic_reconfigure/server.h>

#include <boost/bind.hpp>

//串口配置参数
#define SERIAL_PORT "/dev/ttyTHS1"
#define BUFFER_SIZE 200

//数据包
#define pack_begin 0x1f
#define pack_addr 0x89
#define pack_end 0x4d
#define ID_CONNECT 0x00
#define ID_GPS 0x01
#define ID_IMU 0x02
#define ID_GET_ADC 0x04
#define ID_VEL 0x05
#define ID_PID 0x08
#define ID_ADC 0x03
#define ID_ACT_VEL 0x09
#define ID_SERVO 0x10
#define JETSON

// 传输时转换比例--------------
#define scaleAccel       0.00478515625f // 加速度 [-16g~+16g]    9.8*16/32768
#define scaleQuat        0.000030517578125f // 四元数 [-1~+1]         1/32768
#define scaleAngle       0.0054931640625f // 角度   [-180~+180]     180/32768
#define scaleAngleSpeed  0.06103515625f // 角速度 [-2000~+2000]    2000/32768
#define scaleMag         0.15106201171875f // 磁场 [-4950~+4950]   4950/32768
#define scaleTemperature 0.01f // 温度
#define scaleAirPressure 0.0002384185791f // 气压 [-2000~+2000]    2000/8388608
#define scaleHeight      0.0010728836f    // 高度 [-9000~+9000]    9000/8388608


//#define Debug_
/* 各个模块调试入口 */
#ifdef Debug_
// #define Debug_IMU_
#endif

#ifdef Debug_
#define Dbp(fmt, ...) 
// #define Dbp(fmt, ...) \
//     do { \
//         if(ros::ok()) { \
//             ROS_INFO(fmt, ##__VA_ARGS__); \
//         } \
//     } while(0)
#else
#define Dbp(fmt, ...) 
#endif
#define pow2(x) ((x)*(x)) // 求平方

typedef signed char            S8;
typedef unsigned char          U8;
typedef signed short           S16;
typedef unsigned short         U16;
typedef signed long            S32;
typedef unsigned long          U32;
typedef float                  F32;

typedef struct{//最好根据时间是否足够新来确定数据包是否有效
	volatile unsigned long timestamp;
	/* 订阅消息有效位 */
    volatile unsigned long sub;
	/* 无重力的加速度 */
	volatile float ax;
	volatile float ay;
	volatile float az;
	/* 含重力的加速度 */
	volatile float AX;
	volatile float AY;
	volatile float AZ;
	/* 角速度 */
	volatile float GX;
	volatile float GY;
	volatile float GZ;
	/* 磁场 */
	volatile float CX;
	volatile float CY;
	volatile float CZ;
	/* 温度,气压,高度 */
	volatile float temperature;
	volatile float airPressure;
	volatile float height;
	/* 四元数 */
	volatile float tw;
	volatile float tx;
	volatile float ty;
	volatile float tz;
	/* 欧拉角 */
	volatile float tangleX;
	volatile float tangleY;
	volatile float tangleZ;
	/* 空间位移 */
	volatile float offsetX;
	volatile float offsetY;
	volatile float offsetZ;
	/* 加速度xyz 去掉了重力 */
	volatile float tasX;
	volatile float tasY;
	volatile float tasZ;
	/* 电池ADC */
	volatile unsigned short adc;
	/* GPIO1 */
	volatile unsigned char GPIO1;
	/* 三轴速度 */
	volatile float vx;
	volatile float vy;
	volatile float vz;
    /* 三轴位移 */
    signed short ox;
    signed short oy;
    signed short oz;
}imu_data;

typedef struct{
	double longitude;//经度
	double latitude;//纬度
    double altitude;//高度
}gps_data;

typedef struct{
	double p_vel;
	double p_ang;
}vel_data;

typedef struct{
	float p;
	float i;
	float d;
}pid_data;

typedef struct{
	float servo_angle;
}servo_data;

class car_driver
{
public:
    car_driver();
    ~car_driver();
    void loop();

private:
    /* 初始化ros节点以及串口配置 */
    bool init();
    /* 获取完整包以及解析包 */
    void read_thread_func();
    void monitor_thread();
    void process_raw_data(const uint8_t* data, size_t len);
    bool validate_packet(const std::vector<uint8_t>& pkt);
	void parse_packet(const std::vector<uint8_t>& pkt);
    void parse_IMU(const uint8_t* buf,const uint8_t DLen);
    /* ros话题发布,数据来源:MCU */
    void pub_imu();																			/* Imu数据 */
	void pub_pose();																		/* 机器人位姿 */
	void pub_imu_adc();																		/* Imu电量 */																		/* 里程计 */	
    void pub_gps(gps_data* data);															/* GPS */
    /* ros订阅回调函数 */
    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
	
    /* 串口命令MCU */
    void check_connent();
    void set_pid(pid_data* data);
    void set_vel(vel_data* data);
	void set_servo_angle(servo_data* data);
    void pack_to_tx(uint8_t* data,uint8_t data_length);
    void safe_write(const uint8_t* data, size_t len);

	/* 动态配置参数回调函数 */
	void dynamic_reconfig_callback(my_pkg::PID_reconfigConfig &config,uint32_t level);

	// 添加odom发布方法
    void update_odometry(const geometry_msgs::Twist& twist_msg);
    void publish_odom_tf(const geometry_msgs::Twist& twist_msg);
	//重置odom坐标系
	void reset_odometry();

    bool connect_state;
    bool running;
	imu_data imuData;
	gps_data gpsData;
	vel_data velData;
	pid_data pidData;
	servo_data servoData;
    float adcData;

	/* 串口参数 */
	int baudrate;
	std::string port_name_;
	struct termios options;
	int serial_fd_;

    /* 线程 */
    std::thread read_thread_;      // 接收线程
    std::atomic<bool> running_{false}; // 线程控制标志
    std::mutex data_mutex_;        // 数据互斥锁
    std::mutex packet_mutex_;

	/* 速度 */
	geometry_msgs::Twist current_twist_;

	/* ODOM坐标转换 */ 
    geometry_msgs::TransformStamped odom_trans_;
    nav_msgs::Odometry odom_msg_;
    tf2_ros::TransformBroadcaster odom_broadcaster_;
    // 位置积分相关变量
    double x_pos_, y_pos_, theta_, first_theta_;
    ros::Time last_odom_time_;
    bool first_odom_;
    
	std::string	odom_frame_;
	std::string	base_frame_;
	std::string	camera_frame_;
	std::string imu_frame_;
    std::string gps_frame_;

	ros::Time now_;
	ros::Time last_time_;
	ros::Time last_twist_time_;
    ros::Time last_valid_packet_;

	ros::Publisher odom_pub_;
	ros::Publisher imu_battery_pub_;	/* imu的电池电量,因为厂商没给计算公式所以就只算百分比了 */
	ros::Publisher imu_pub_;
	ros::Publisher gps_pub_;			
    ros::Publisher adc_pub_;			/* 车载电池ADC */
	ros::Publisher pose_pub_;			/* 机器人位姿发布者 */
	ros::Publisher act_vel_pub_;		/* 实际速度发布者,仅来源于单片机通过霍尔采集的车轮转速 */
	ros::Publisher pur_vel_pub_;		/* 期望速度发布者 */

	ros::Subscriber cmd_sub_;

	sensor_msgs::Imu Imu_pub_data;
	std_msgs::Float32 Imu_adc_pub_data;
	geometry_msgs::Pose pose_pub_data;

	bool first_init_;					/* 动态配置初始化标志 */

	int servo_bias_;          			/* 底盘偏置值 */
};

