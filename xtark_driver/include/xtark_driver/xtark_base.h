#ifndef XTARK_BASE_H
#define XTARK_BASE_H


#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <termios.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "xtark_kinematic.h" 

#include <ros/ros.h>    
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/point_cloud_conversion.h"

#include <tf/tf.h>


#ifndef DEBUG
#define DEBUG  0
#endif

#define MD_HEALTH_NONE                       0x0000      //无错误
#define MD_HEALTH_OVERVOLTAGE                0x0001      //过压
#define MD_HEALTH_LOWVOLTAGE                 0x0002      //欠压
#define MD_HEALTH_OVERCURRENT                0x0004      //过流
#define MD_HEALTH_OVERLOADED                 0x0008      //过载
#define MD_HEALTH_CURRENT_OVER_TOLERANCE     0x0010      //电流超差
#define MD_HEALTH_ENCODER_OVER_TOLERANCE     0x0020      //编码器超差
#define MD_HEALTH_SPEED_OVER_TOLERANCE       0x0050      //速度超差
#define MD_HEALTH_REFVOL                     0x0080      //参考电压出错
#define MD_HEALTH_EEPROM                     0x0100      //EEPROM读写错误
#define MD_HEALTH_HALL                       0x0200      //霍尔出错
#define MD_HEALTH_OVERTEMP                   0x0400      //电机温度过高

#define SERIAL_HEAD_A 0xAA
#define SERIAL_HEAD_B 0x55

#define RT_ENCODER    0x11
#define RT_IMU        0x10
#define RT_SONAR      0x12
#define RT_JOY        0x13
#define RT_STATUS     0x20

#define ST_VELOCITY   0x70
#define ST_CONTROLBIT 0x71
#define ST_CLEARENCODER 0x72
#define ST_LIGHT      0x80

#define SONAR_MAX_RANGE 0.4
#define SONAR_MIN_RANGE 0.05
#define SONAR_HEIGHT    0.1235
#define SONAR_FIELD_OF_VIEW 0.2617
#define SONAR_FILTER_THRESHOLD 0.01

#define BATTERY_FULL_VOLTAGE 25.0
#define BATTERY_EMPTY_VOLTAGE 19.8


typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;
typedef boost::shared_lock<boost::shared_mutex> read_lock;
typedef boost::unique_lock<boost::shared_mutex> write_lock;



enum ControlFunctionBits
{
    ENABLE = 0,
    ESTOP,
    DISABALE
};

enum packetFinderState
{
    waitingForHead1,
    waitingForHead2,
    waitingForPayloadSize,
    waitingForPayloadType,
    waitingForPayload,
    waitingForCheckSum,
    handlePayload
};

struct imu_data{
    float roll;
    float pitch;
    float yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
};

struct SonarRanger
{

    typedef std::shared_ptr<SonarRanger> Ptr;
    typedef std::shared_ptr<const SonarRanger> ConstPtr;
    sensor_msgs::Range range;  
    double offset_x;
    double offset_y;
    double offset_yaw;
    double height;
    double last_range;
    double current_range;

    void init(float field_of_view, float min_range, float max_range,float sonar_height)
    {
        this->range.field_of_view = field_of_view;
        this->range.min_range = min_range;
        this->range.max_range = max_range;
        this->range.radiation_type = sensor_msgs::Range::ULTRASOUND;
        this->height = sonar_height;
    }

};

struct status_data{
    double voltage;
    uint32_t motor_status;
};



class XtarkBase
{
    public:
        XtarkBase(); 
        ~XtarkBase();
        void runTest();
    private:
        bool initSerial();
        bool closeSerial();
        bool resetSerial();
        
        void recv_thread_callback();
        void handle_serial_data(const uint8_t* buffer_date);
        void check_sum(uint8_t* data, size_t len, uint8_t& dest);
        void distribute_data(uint8_t msg_type, uint8_t* buffer_data);

        serialp_ptr sp_;
        boost::system::error_code ec_;
        boost::asio::io_service io_service_;
        packetFinderState state_;
        bool recv_flag_;
        
        int             m_SerialBaudRate_;
        std::string     m_SerialPort_;
        int32_t l_motor_pos_;
        int32_t r_motor_pos_;
        
        int32_t l_motor_pos_offset_;
        int32_t r_motor_pos_offset_;

        bool motor_pos_initial_;


        int16_t l_motor_speed_;
        int16_t r_motor_speed_;
        struct imu_data imu_data_;
        struct status_data status_data_;



        void initNode();
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
        void control_bit_callback(const std_msgs::Int16::ConstPtr& bit);
        void light_callback(const std_msgs::ColorRGBA::ConstPtr& light);
        void left_light_callback(const std_msgs::ColorRGBA::ConstPtr& light);
        void right_light_callback(const std_msgs::ColorRGBA::ConstPtr& light);
        void control_timer_callback(const ros::TimerEvent&);

        XtarkKinematic* kinematic_handle_;

        SonarRanger sonar_sensor_[5];

        std::string odom_frame_;
        std::string base_frame_;
        std::string imu_frame_;

        bool publish_odom_transform_;
        bool enable_sonar_;

        double inflation_dis_;
        double safe_dis_;
        double slow_dis_;

        double robot_radius_;
        double robot_wheel_diameter_;
        double robot_wheel_track_;

        double acc_bias_x_, acc_bias_y_,acc_bias_z_;
        bool imu_ready_;
        


        geometry_msgs::TransformStamped transformStamped_;
        tf2_ros::TransformBroadcaster br_;

        geometry_msgs::Twist current_twist_;
        nav_msgs::Odometry odom_pub_data_;
        std_msgs::Float32 battery_voltage_pub_data_;
        std_msgs::Float32 battery_percentage_pub_data_;
        sensor_msgs::Imu imu_pub_data_;


        ros::Publisher odom_pub_;
        ros::Publisher battery_voltage_pub_;
        ros::Publisher battery_percentage_pub_;
        ros::Publisher sonar_pointcloud_pub_;
        ros::Publisher imu_pub_;

        ros::Publisher sonar_0_pub_;
        ros::Publisher sonar_1_pub_;
        ros::Publisher sonar_2_pub_;
        ros::Publisher sonar_3_pub_;
        ros::Publisher sonar_4_pub_;

        //pcl::PointCloud<pcl::PointXYZ> cloud_;
        sensor_msgs::PointCloud2 sonar_pointcloud_data_;
        sensor_msgs::PointCloud cloud_;

        ros::Subscriber cmd_sub_;
        ros::Subscriber control_bit_sub_;
        ros::Subscriber light_sub_;
        ros::Subscriber left_light_sub_;
        ros::Subscriber right_light_sub_;

        ros::Timer topic_pub_timer_;
        ros::Timer control_timer_;

        boost::mutex cmd_vel_mutex_;

        ros::Time t_now_;
        ros::Time t_last_;
        ros::Time t_last_twist_;

        bool        initRobot();
        bool        closeRobot();
        
        bool        SetControlBit(int8_t  bit);   //设置控制位-  ESTOP  :  CLEARALERT  :  DISABLE  :  ENABLE
        bool        SetMotorSpeed(int16_t l_rpm,int16_t r_rpm);              //设置电机速度   单位(rpm)
        bool        SetEncoderClear();
        bool        SetColor(float r, float g, float b, float a, uint8_t selet); //selet: 0 - both , 1 - left  2 - right
        

};

#endif
