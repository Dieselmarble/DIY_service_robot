#include "xtark_base.h"

XtarkBase::XtarkBase():state_(waitingForHead1),m_SerialBaudRate_(256000),imu_ready_(false),l_motor_pos_offset_(0),r_motor_pos_offset_(0),motor_pos_initial_(false)
{
    initNode();
}
XtarkBase::~XtarkBase()
{
    closeRobot();
}

void XtarkBase::initNode()
{
    ROS_INFO("Initialing Xtark Robot");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_("~");

    cloud_.points.resize(5);
    cloud_.channels.resize(1);
    cloud_.channels[0].name = "intensities";
    cloud_.channels[0].values.resize(5);


    for(int i=0;i<5;i++)
    {
        sonar_sensor_[i].init(SONAR_FIELD_OF_VIEW,SONAR_MIN_RANGE,SONAR_MAX_RANGE,SONAR_HEIGHT);
    }

    sonar_sensor_[0].range.header.frame_id = "sonar0_link";
    sonar_sensor_[1].range.header.frame_id = "sonar1_link";
    sonar_sensor_[2].range.header.frame_id = "sonar2_link";
    sonar_sensor_[3].range.header.frame_id = "sonar3_link";
    sonar_sensor_[4].range.header.frame_id = "sonar4_link";

    nh_p_.param<std::string>("odom_frame",odom_frame_,std::string("odom")); 
    nh_p_.param<std::string>("base_frame",base_frame_,std::string("base_link"));
    nh_p_.param<std::string>("imu_frame",imu_frame_,std::string("imu_link"));
    nh_p_.param<bool>("publish_odom_transform", publish_odom_transform_, true);
    nh_p_.param<bool>("enable_sonar", enable_sonar_, true);
    nh_p_.param<std::string>("port",m_SerialPort_,std::string("/dev/ttyS0"));
    
    nh_p_.param<double>("sonar0_offset_x",sonar_sensor_[0].offset_x,0.0);
    nh_p_.param<double>("sonar0_offset_y",sonar_sensor_[0].offset_y,0.0);
    nh_p_.param<double>("sonar0_offset_yaw",sonar_sensor_[0].offset_yaw,0.0);
    
    nh_p_.param<double>("sonar1_offset_x",sonar_sensor_[1].offset_x,0.0);
    nh_p_.param<double>("sonar1_offset_y",sonar_sensor_[1].offset_y,0.0);
    nh_p_.param<double>("sonar1_offset_yaw",sonar_sensor_[1].offset_yaw,0.0);

    nh_p_.param<double>("sonar2_offset_x",sonar_sensor_[2].offset_x,0.0);
    nh_p_.param<double>("sonar2_offset_y",sonar_sensor_[2].offset_y,0.0);
    nh_p_.param<double>("sonar2_offset_yaw",sonar_sensor_[2].offset_yaw,0.0);
    
    nh_p_.param<double>("sonar3_offset_x",sonar_sensor_[3].offset_x,0.0);
    nh_p_.param<double>("sonar3_offset_y",sonar_sensor_[3].offset_y,0.0);
    nh_p_.param<double>("sonar3_offset_yaw",sonar_sensor_[3].offset_yaw,0.0);
    
    nh_p_.param<double>("sonar4_offset_x",sonar_sensor_[4].offset_x,0.0);
    nh_p_.param<double>("sonar4_offset_y",sonar_sensor_[4].offset_y,0.0);
    nh_p_.param<double>("sonar4_offset_yaw",sonar_sensor_[4].offset_yaw,0.0);

    nh_p_.param<double>("safe_distance",safe_dis_,0.5);
    nh_p_.param<double>("inflation_slow_distance",inflation_dis_,0.5);
    nh_p_.param<double>("robot_radius",robot_radius_,0.226);
    nh_p_.param<double>("robot_wheel_diameter",robot_wheel_diameter_,0.1417);
    nh_p_.param<double>("robot_wheel_track",robot_wheel_track_,0.376);
    
    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel",10,&XtarkBase::cmd_vel_callback,this);
    control_bit_sub_ = nh_.subscribe<std_msgs::Int16>("control_bit",1,&XtarkBase::control_bit_callback,this);
    light_sub_ = nh_.subscribe<std_msgs::ColorRGBA>("light",10,&XtarkBase::light_callback,this);
    left_light_sub_ = nh_.subscribe<std_msgs::ColorRGBA>("left_light",10,&XtarkBase::left_light_callback,this);
    right_light_sub_ = nh_.subscribe<std_msgs::ColorRGBA>("right_light",10,&XtarkBase::right_light_callback,this);
    
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom",10);
    battery_voltage_pub_ = nh_.advertise<std_msgs::Float32>("battery_voltage",10);
    battery_percentage_pub_ = nh_.advertise<std_msgs::Float32>("battery_percentage",10);
    sonar_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sonar_pointcloud",1);
    sonar_0_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar0",1);
    sonar_1_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar1",1);
    sonar_2_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar2",1);
    sonar_3_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar3",1);
    sonar_4_pub_ = nh_.advertise<sensor_msgs::Range>("/sonar4",1);
    imu_pub_     = nh_.advertise<sensor_msgs::Imu>("imu_raw",10);
    control_timer_   = nh_.createTimer(ros::Duration(0.02),&XtarkBase::control_timer_callback,this);

    //safe_dis_ = robot_radius_/abs(sonar_sensor_[0].offset_x)*sqrt(pow(sonar_sensor_[0].offset_x,2)+pow(sonar_sensor_[0].offset_y,2)) - sqrt(pow(sonar_sensor_[0].offset_x,2)+pow(sonar_sensor_[0].offset_y,2));
    slow_dis_ = safe_dis_ + inflation_dis_;
    kinematic_handle_ = new XtarkKinematic(robot_wheel_track_,robot_wheel_diameter_,16384);
    kinematic_handle_->initKinematic();
    ROS_INFO("Initialize Robot Kinematic : \033[32m Done! \033[0m...");
    
    if(!initRobot())
    {
        ros::shutdown();
        return;
    }

    SetControlBit(ControlFunctionBits::ENABLE);
    //SetEncoderClear();
    
    ROS_INFO("Initialize Robot Base : \033[32m Done! \033[0m...");

    ros::spin();
}

bool XtarkBase::SetColor(float r, float g, float b, float a , uint8_t selet)
{
    static uint8_t send_buff[11];
    send_buff[0] = SERIAL_HEAD_A;
    send_buff[1] = SERIAL_HEAD_B;
    send_buff[2] = 0x0a;
    send_buff[3] = ST_LIGHT;
    send_buff[4] = 0x10;
    send_buff[5] = (uint8_t)selet;
    send_buff[6] = (uint8_t)(r*a*255);
    send_buff[7] = (uint8_t)(g*a*255);
    send_buff[8] = (uint8_t)(b*a*255);
    ROS_INFO("Light Control : selet: %d R: %d, G: %d, B: %d",selet,send_buff[6],send_buff[7],send_buff[8]);
    check_sum(send_buff,9,send_buff[9]);
    boost::asio::write(*sp_.get(),boost::asio::buffer(send_buff,10),ec_);
    if(ec_)
    {
        ROS_WARN("Send Color buffer Error! what: [%s] \r\n",ec_.message().c_str());
        ec_.clear();
        return false;
    }
    return true;
}
void XtarkBase::left_light_callback(const std_msgs::ColorRGBA::ConstPtr& light)
{
    SetColor(light->r,light->g,light->b,light->a, 1);
}
void XtarkBase::right_light_callback(const std_msgs::ColorRGBA::ConstPtr& light)
{
    SetColor(light->r,light->g,light->b,light->a, 2);
}
void XtarkBase::light_callback(const std_msgs::ColorRGBA::ConstPtr& light)
{
    SetColor(light->r,light->g,light->b,light->a, 0);
}

bool XtarkBase::initSerial()
{
    if(sp_)
    {
        printf("The SerialPort is already opened!\r\n");
        return false;
    }
     sp_ = serialp_ptr(new boost::asio::serial_port(io_service_));
     sp_->open(m_SerialPort_,ec_);
     if(ec_)
     {
        printf("Open Port: %s Failed! Aboart!",m_SerialPort_.c_str());
        return false;
     }
    //set_serial_baudrate(*sp_,256000);
    sp_->set_option(boost::asio::serial_port_base::baud_rate(230400));
    //sp_->set_option(boost::asio::serial_port_base::baud_rate(m_SerialBaudRate_));
    sp_->set_option(boost::asio::serial_port_base::character_size(8));
    sp_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    sp_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    sp_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    
    
    return true;

}

bool XtarkBase::closeSerial()
{
    if(sp_)
    {
        sp_->cancel();
        sp_->close();
        sp_.reset();
    }
    io_service_.stop();
    io_service_.reset();
    return true;
}

bool XtarkBase::initRobot()
{
    if(initSerial())
    {
        try
        {
            boost::thread recvSerial_thread(boost::bind(&XtarkBase::recv_thread_callback,this));
            return true;
        }
        catch(...)
        {
            return false;
        }
    }
    else
    {
        return false;
    }   
}
void XtarkBase::recv_thread_callback()
{
    uint8_t payload_size, check_num, buffer_data[255],payload_type;
    state_ = waitingForHead1;
    recv_flag_ = true;

    while(recv_flag_)
    {
        switch (state_)
        {
            case waitingForHead1:
                check_num = 0x00;
                boost::asio::read(*sp_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
                state_ = buffer_data[0] == SERIAL_HEAD_A ? waitingForHead2 : waitingForHead1;
                if(state_ == waitingForHead1)
                {
                    ROS_DEBUG("recv head1 error : %02X \r\n",buffer_data[0]);
                }
                break;
            case waitingForHead2:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[1],1),ec_);
                state_ = buffer_data[1] == SERIAL_HEAD_B ? waitingForPayloadSize : waitingForHead1;
                if(state_ == waitingForHead1)
                {
                    ROS_DEBUG("recv head2 error : %02X \r\n",buffer_data[1]);
                }
                break;
            case waitingForPayloadSize:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[2],1),ec_);
                payload_size = buffer_data[2] - 4;
                state_ = waitingForPayload;
                break;
            case waitingForPayload:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[3],payload_size),ec_);
                payload_type = buffer_data[3];
                state_ = waitingForCheckSum;
                break;
            case waitingForCheckSum:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[3+payload_size],1),ec_);
                check_sum(buffer_data,3+payload_size,check_num);
                state_ = buffer_data[3+payload_size] == check_num ? handlePayload : waitingForHead1;
                if(state_ == waitingForHead1)
                {
                    ROS_DEBUG("check sum error : %02X  cala is %02x\r\n",buffer_data[3+payload_size],check_num);
                }
                break;
            case handlePayload:
                distribute_data(payload_type, buffer_data);
                state_ = waitingForHead1;
                break;
            default:
                state_ = waitingForHead1;
                break;
        }
    }
}
void XtarkBase::control_bit_callback(const std_msgs::Int16::ConstPtr& bit)
{
    SetControlBit((int8_t)bit->data);
}
void XtarkBase::distribute_data(uint8_t msg_type, uint8_t* buffer_data)
{
      if(msg_type == RT_ENCODER)
      {
        

        static double kinematic_odom[3];
        static double kinematic_vel[2];
          
        l_motor_speed_ = (int16_t)(buffer_data[4]*256+buffer_data[5]);
        r_motor_speed_ = (int16_t)(buffer_data[6]*256+buffer_data[7]);



        l_motor_pos_   = (uint32_t)(buffer_data[8]*16777216+buffer_data[9]*65536+buffer_data[10]*256+buffer_data[11]);
        r_motor_pos_   = (uint32_t)(buffer_data[12]*16777216+buffer_data[13]*65536+buffer_data[14]*256+buffer_data[15]);
        if(!motor_pos_initial_)
        {
            l_motor_pos_offset_ =  l_motor_pos_;
            r_motor_pos_offset_ =  r_motor_pos_;
            motor_pos_initial_ = true;
            return;
        }

        kinematic_handle_->updateOdom(l_motor_pos_-l_motor_pos_offset_,r_motor_pos_-r_motor_pos_offset_);
        kinematic_handle_->updateVelocity(l_motor_speed_,r_motor_speed_,kinematic_vel);

        kinematic_handle_->getOdom(kinematic_odom);

        transformStamped_.header.frame_id = odom_frame_;
        transformStamped_.child_frame_id  = base_frame_;
        transformStamped_.header.stamp    = ros::Time::now();
        transformStamped_.transform.translation.x = kinematic_odom[0];
        transformStamped_.transform.translation.y = kinematic_odom[1];
        transformStamped_.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0,0,kinematic_odom[2]);
        transformStamped_.transform.rotation.x = q.x();
        transformStamped_.transform.rotation.y = q.y();
        transformStamped_.transform.rotation.z = q.z();
        transformStamped_.transform.rotation.w = q.w();
        if(publish_odom_transform_)
            br_.sendTransform(transformStamped_);
    
        odom_pub_data_.header.frame_id = odom_frame_;
        odom_pub_data_.child_frame_id  = base_frame_;
        odom_pub_data_.header.stamp    = ros::Time::now();
        odom_pub_data_.pose.pose.position.x = kinematic_odom[0];
        odom_pub_data_.pose.pose.position.y = kinematic_odom[1];
        odom_pub_data_.pose.pose.position.z = 0;
        odom_pub_data_.pose.pose.orientation.x = q.getX();
        odom_pub_data_.pose.pose.orientation.y = q.getY();
        odom_pub_data_.pose.pose.orientation.z = q.getZ();
        odom_pub_data_.pose.pose.orientation.w = q.getW();
        odom_pub_data_.twist.twist.linear.x = kinematic_vel[0];
        odom_pub_data_.twist.twist.angular.z = kinematic_vel[1];
        odom_pub_data_.twist.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0, 
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0, 
                              0, 0, 0, 0, 1e6, 0, 
                              0, 0, 0, 0, 0, 0.1 };
        odom_pub_data_.pose.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0, 
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0, 
                              0, 0, 0, 0, 1e6, 0, 
                              0, 0, 0, 0, 0, 1e3 };
        odom_pub_.publish(odom_pub_data_);
      }
      else if(msg_type == RT_IMU)
      {
          static int imu_calib_count = 100;  
            

          imu_data_.accel_x =  ((double)((int16_t)(buffer_data[4]*256+buffer_data[5]))/32768*16*9.81);
          imu_data_.accel_y =  ((double)((int16_t)(buffer_data[6]*256+buffer_data[7]))/32768*16*9.81);
          imu_data_.accel_z =  ((double)((int16_t)(buffer_data[8]*256+buffer_data[9]))/32768*16*9.81);

          imu_data_.gyro_x  = ((double)((int16_t)(buffer_data[10]*256+buffer_data[11]))/32768*2000/180*3.1415926);
          imu_data_.gyro_y  = ((double)((int16_t)(buffer_data[12]*256+buffer_data[13]))/32768*2000/180*3.1415926);
          imu_data_.gyro_z  = ((double)((int16_t)(buffer_data[14]*256+buffer_data[15]))/32768*2000/180*3.1415926);

          imu_data_.roll    = ((double)((int16_t)(buffer_data[16]*256+buffer_data[17]))/32768*3.1415926);
          imu_data_.pitch   = ((double)((int16_t)(buffer_data[18]*256+buffer_data[19]))/32768*3.1415926);
          imu_data_.yaw    = ((double)((int16_t)(buffer_data[20]*256+buffer_data[21]))/32768*3.1415926);
          if(!imu_ready_)
          {
            if(imu_calib_count)
            {
              acc_bias_x_ += imu_data_.accel_x;
              acc_bias_y_ += imu_data_.accel_y;
              acc_bias_z_ += (imu_data_.accel_z - 9.81);
              imu_calib_count --;
            }
            else
            {
                acc_bias_x_ = acc_bias_x_ / 100;
                acc_bias_y_ = acc_bias_y_ / 100;
                acc_bias_z_ = acc_bias_z_ / 100;
                imu_ready_ = true;
            }
          }       
        

	  //ROS_INFO("IMU: %.3f",imu_data_.yaw/3.1415926*180);
          //imu_pub_data_.orientation = tf::createQuaternionMsgFromRollPitchYaw(imu_data_.roll,imu_data_.pitch,imu_data_.yaw);
	  
          imu_pub_data_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,imu_data_.yaw);
          imu_pub_data_.header.stamp = ros::Time::now();
          imu_pub_data_.header.frame_id = imu_frame_;
          imu_pub_data_.angular_velocity.x = imu_data_.gyro_x;
          imu_pub_data_.angular_velocity.y = imu_data_.gyro_y;
          imu_pub_data_.angular_velocity.z = imu_data_.gyro_z;
          imu_pub_data_.linear_acceleration.x = imu_data_.accel_x - acc_bias_x_;
          imu_pub_data_.linear_acceleration.y = imu_data_.accel_y - acc_bias_y_;
          imu_pub_data_.linear_acceleration.z = imu_data_.accel_z - acc_bias_z_;
          imu_pub_data_.orientation_covariance = {1e6, 0, 0,
                                            0, 1e6, 0,
                                            0, 0, 0.05};
          imu_pub_data_.angular_velocity_covariance = {1e6, 0, 0,
                                                 0, 1e6, 0,
                                                 0, 0, 1e6};
          imu_pub_data_.linear_acceleration_covariance = {1e-2, 0, 0,
                                                     0, 0, 0,
                                                     0, 0, 0};
       	  if(imu_ready_)
          {
            imu_pub_.publish(imu_pub_data_);
          }
      }
      else if(msg_type == RT_SONAR)
      {
          sonar_sensor_[0].current_range = (double)((uint8_t)buffer_data[4])/100;
          sonar_sensor_[1].current_range = (double)((uint8_t)buffer_data[5])/100;
          sonar_sensor_[2].current_range = (double)((uint8_t)buffer_data[6])/100;
          sonar_sensor_[3].current_range = (double)((uint8_t)buffer_data[7])/100;
          sonar_sensor_[4].current_range = (double)((uint8_t)buffer_data[8])/100;


          for(int i=0;i<5;i++)
        {

            if(std::abs(sonar_sensor_[i].current_range - sonar_sensor_[i].last_range) > SONAR_FILTER_THRESHOLD)
            {
                sonar_sensor_[i].range.range = sonar_sensor_[i].last_range;
            }
            else
            {
                sonar_sensor_[i].range.range = sonar_sensor_[i].current_range;
            }
            sonar_sensor_[i].last_range = sonar_sensor_[i].current_range;


            if(sonar_sensor_[i].range.range < SONAR_MIN_RANGE)
            {
                sonar_sensor_[i].range.range = SONAR_MIN_RANGE;
                cloud_.points[i].x = sonar_sensor_[i].offset_x + sonar_sensor_[i].range.range * cos(sonar_sensor_[i].offset_yaw);
                cloud_.points[i].y = sonar_sensor_[i].offset_y + sonar_sensor_[i].range.range * sin(sonar_sensor_[i].offset_yaw);
            }
            else if(sonar_sensor_[i].range.range > SONAR_MAX_RANGE)
            {
                sonar_sensor_[i].range.range =  SONAR_MAX_RANGE;
                cloud_.points[i].x = std::numeric_limits<float>::infinity();
                cloud_.points[i].y = std::numeric_limits<float>::infinity();
            }
            else
            {
                cloud_.points[i].x = sonar_sensor_[i].offset_x + sonar_sensor_[i].range.range * cos(sonar_sensor_[i].offset_yaw);
                cloud_.points[i].y = sonar_sensor_[i].offset_y + sonar_sensor_[i].range.range * sin(sonar_sensor_[i].offset_yaw);
            }

            cloud_.points[i].z = sonar_sensor_[i].height;
            //cloud_.points[i].z = 0; 
            cloud_.channels[0].values[i] = 0;
        }

        sensor_msgs::convertPointCloudToPointCloud2(cloud_,sonar_pointcloud_data_);

        sonar_pointcloud_data_.header.frame_id = "base_link";
        sonar_pointcloud_data_.header.stamp = ros::Time::now();
        sonar_pointcloud_pub_.publish(sonar_pointcloud_data_);
        sonar_sensor_[0].range.header.stamp = ros::Time::now();
        sonar_sensor_[1].range.header.stamp = ros::Time::now();
        sonar_sensor_[2].range.header.stamp = ros::Time::now();
        sonar_sensor_[3].range.header.stamp = ros::Time::now();
        sonar_sensor_[4].range.header.stamp = ros::Time::now();
        sonar_0_pub_.publish(sonar_sensor_[0].range);
        sonar_1_pub_.publish(sonar_sensor_[1].range);
        sonar_2_pub_.publish(sonar_sensor_[2].range);
        sonar_3_pub_.publish(sonar_sensor_[3].range);
        sonar_4_pub_.publish(sonar_sensor_[4].range);
      }
      else if(msg_type == RT_STATUS)
      {
          status_data_.voltage = (double)((int16_t)(buffer_data[4]*256+buffer_data[5]))/100;
          status_data_.motor_status = buffer_data[6]<<24|buffer_data[7]<<16|buffer_data[8]<<8|buffer_data[9];

          battery_voltage_pub_data_.data = status_data_.voltage;
          battery_percentage_pub_data_.data = (status_data_.voltage - BATTERY_EMPTY_VOLTAGE)/(BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE) * 100;
          battery_percentage_pub_.publish(battery_percentage_pub_data_);
          battery_voltage_pub_.publish(battery_voltage_pub_data_);
      }
      else
      {
          printf("XtarkBase: Unknown DataType: %02x \r\n",msg_type);
      }

}
void XtarkBase::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    try
    {
        cmd_vel_mutex_.lock();
        t_last_twist_ = ros::Time::now();
        current_twist_ = *msg.get();
        cmd_vel_mutex_.unlock();
    }
    catch(...)
    {
        cmd_vel_mutex_.unlock();
    }
}
void XtarkBase::control_timer_callback(const ros::TimerEvent&)
{
    int16_t wheel_target_speed[2];
    if((ros::Time::now() - t_last_twist_).toSec() <= 1.0)
    {
        cmd_vel_mutex_.lock();

        if(enable_sonar_)   
        {
            if(sonar_sensor_[0].range.range > safe_dis_ && sonar_sensor_[0].range.range <= slow_dis_ ) //左前超声波
            {
                if(current_twist_.linear.x > 0)
                {
                    current_twist_.linear.x = 0.6 * current_twist_.linear.x;
                    
                    if(current_twist_.angular.z > 0)
                        current_twist_.angular.z = current_twist_.angular.z * 0.6;
                }
            
            }
            else if(sonar_sensor_[0].range.range < safe_dis_)
            {
                if(current_twist_.linear.x > 0)
                    current_twist_.linear.x = 0;
            }
            
            if(sonar_sensor_[1].range.range > safe_dis_ && sonar_sensor_[1].range.range <= slow_dis_ ) //前超声波
            {
                if(current_twist_.linear.x > 0)
                    current_twist_.linear.x = 0.6 * current_twist_.linear.x;
            }
            else if(sonar_sensor_[1].range.range < safe_dis_)
            {
                if(current_twist_.linear.x > 0)
                    current_twist_.linear.x = 0;
            }
            
            if(sonar_sensor_[2].range.range > safe_dis_ && sonar_sensor_[2].range.range <= slow_dis_ ) //右前超声波
            {
                if(current_twist_.linear.x > 0)
                {
                    current_twist_.linear.x = 0.6 * current_twist_.linear.x;
                    if(current_twist_.angular.z < 0)
                        current_twist_.angular.z = current_twist_.angular.z * 0.6;
                }
            }
            else if(sonar_sensor_[2].range.range < safe_dis_)
            {
                if(current_twist_.linear.x > 0)
                    current_twist_.linear.x = 0;
            }
            
            if(sonar_sensor_[3].range.range > safe_dis_ && sonar_sensor_[3].range.range <= slow_dis_ ) //右后超声波
            {
                if(current_twist_.linear.x < 0)
                {
                    current_twist_.linear.x = 0.6 * current_twist_.linear.x;
                    if(current_twist_.angular.z > 0)
                    {
                        current_twist_.angular.z = current_twist_.angular.z * 0.6;
                    }
                }
            }
            else if(sonar_sensor_[3].range.range < safe_dis_)
            {
                if(current_twist_.linear.x < 0)
                    current_twist_.linear.x = 0;
            }
         
            if(sonar_sensor_[4].range.range > safe_dis_ && sonar_sensor_[4].range.range <= slow_dis_ ) //右后超声波
            {
                if(current_twist_.linear.x < 0)
                {
                    current_twist_.linear.x = 0.6 * current_twist_.linear.x;
                    if(current_twist_.angular.z < 0)
                    {
                        current_twist_.angular.z = current_twist_.angular.z * 0.6;
                    }
                }
            }
            else if(sonar_sensor_[4].range.range < safe_dis_)
            {
                if(current_twist_.linear.x < 0)
                    current_twist_.linear.x = 0;
            }
        }

        kinematic_handle_->updateWheelSpeed(current_twist_, wheel_target_speed);
        cmd_vel_mutex_.unlock();
        SetMotorSpeed(wheel_target_speed[0],wheel_target_speed[1]);
    }
    else
    {
        SetMotorSpeed(0,0);
    }
}

bool XtarkBase::closeRobot()
{
        if(!closeSerial())               
            return false;

        return true;
}

bool XtarkBase::SetEncoderClear()
{
    static uint8_t send_buff[6];
    send_buff[0] = SERIAL_HEAD_A;
    send_buff[1] = SERIAL_HEAD_B;
    send_buff[2] = 0x06;
    send_buff[3] = ST_CLEARENCODER;
    send_buff[4] = 0x55;
    check_sum(send_buff,5,send_buff[5]);
    boost::asio::write(*sp_.get(),boost::asio::buffer(send_buff,6),ec_);
    if(ec_)
    {
        ROS_WARN("Send ClearEncoder buffer Error! what: [%s] \r\n",ec_.message().c_str());
        ec_.clear();
        return false;
    }
    return true;
}
bool XtarkBase::SetControlBit(int8_t bit)
{
    static uint8_t send_buff[6];
    send_buff[0] = SERIAL_HEAD_A;
    send_buff[1] = SERIAL_HEAD_B;
    send_buff[2] = 0x06;
    send_buff[3] = ST_CONTROLBIT;
    send_buff[4] = bit;
    check_sum(send_buff,5,send_buff[5]);
    for(int i = 0;i<5;i++)
    {
    boost::asio::write(*sp_.get(),boost::asio::buffer(send_buff,6),ec_);
    if(ec_)
    {
        ROS_WARN("Send ControlBit buffer Error! what: [%s] \r\n",ec_.message().c_str());
        ec_.clear();
        return false;
    }
    }
    return true;
}
bool XtarkBase::SetMotorSpeed(int16_t l_rpm,int16_t r_rpm)
{
    static uint8_t send_buff[10];
    send_buff[0] = SERIAL_HEAD_A;
    send_buff[1] = SERIAL_HEAD_B;
    send_buff[2] = 0x09;
    send_buff[3] = ST_VELOCITY;
    send_buff[4] = (l_rpm>>8)&0xff;
    send_buff[5] = l_rpm&0xff;
    send_buff[6] = (r_rpm>>8)&0xff;
    send_buff[7] = r_rpm&0xff;
    check_sum(send_buff,8,send_buff[8]);
    boost::asio::write(*sp_.get(),boost::asio::buffer(send_buff,9),ec_);
    if(ec_)
    {
        ROS_DEBUG("Send Motor Speed buffer Error! what: [%s] \r\n",ec_.message().c_str());
        ec_.clear();
        return false;
    }

    return true;
}

bool XtarkBase::resetSerial()
{
    if(!closeSerial())
        return false;
    
    if(!initSerial())
        return false;
    
    return true;
}

void XtarkBase::check_sum(uint8_t* data, size_t len,uint8_t& dest)
{
    dest = 0x00;
    for(int i=0;i<len;i++)
    {
        dest += *(data + i);
    }
}

void XtarkBase::runTest()
{
    initRobot();

    SetMotorSpeed(0,0);
    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}
int main(int argc, char** argv)
{
    ros::init(argc,argv,"xtark_driver");
    XtarkBase xtark_driver;
    return 0;
}



