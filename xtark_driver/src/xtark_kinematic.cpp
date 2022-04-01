#include "xtark_kinematic.h"

XtarkKinematic::XtarkKinematic(double wheelTrack, double wheelDiameter,double encoderResolution):wheelTrack_(wheelTrack),wheelDiameter_(wheelDiameter),encoderResolution_(encoderResolution)
{
}

XtarkKinematic::~XtarkKinematic()
{

}
void XtarkKinematic::setKinematicParams(double wheelTrack,double wheelDiameter, double encoderResolution)
{
    wheelTrack_ = wheelTrack;
    wheelDiameter_ = wheelDiameter;
    encoderResolution_ = encoderResolution;
}
void XtarkKinematic::updateWheelSpeed(geometry_msgs::Twist& vel,int16_t* wheelspeed)
{
    double left_unit,right_unit;

    double linear_speed,angular_speed;

    linear_speed = vel.linear.x;
    angular_speed = vel.angular.z;

    if(linear_speed > XTARK_LINEAR_SPEED_LIMIT)
        linear_speed = XTARK_LINEAR_SPEED_LIMIT;
    if(linear_speed < (-XTARK_LINEAR_SPEED_LIMIT))
        linear_speed = -XTARK_LINEAR_SPEED_LIMIT;

    if(angular_speed > XTARK_ANGULAR_SPEED_LIMIT)
        angular_speed = XTARK_ANGULAR_SPEED_LIMIT;
    if(angular_speed < -XTARK_ANGULAR_SPEED_LIMIT)
        angular_speed = -XTARK_ANGULAR_SPEED_LIMIT;

    left_unit  = linear_speed - angular_speed*wheelTrack_/2.0;
    right_unit = linear_speed + angular_speed*wheelTrack_/2.0;

    wheelspeed[0] = static_cast<int16_t>(left_unit*60/meterPerRound_);
    wheelspeed[1] = static_cast<int16_t>(right_unit*60/meterPerRound_);
}

void XtarkKinematic::updateOdom(const int32_t leftCounts, const int32_t rightCounts)
{

    static double delta_left,delta_right;
    static double delta_xy_ave_,delta_th_;

    static double odom_delta_x_,odom_delta_y_;
    

    recv_left_ = static_cast<int64_t>(leftCounts);
    recv_right_ = static_cast<int64_t>(rightCounts);

    if(recv_left_ < encoder_low_wrap_ && last_left_ > encoder_high_wrap_)
        l_encoder_multi_++;
    else if(recv_left_ > encoder_high_wrap_ && last_left_ < encoder_low_wrap_)
        l_encoder_multi_--;
    else
        l_encoder_multi_ = 0;

    if(recv_right_ < encoder_low_wrap_ && last_right_ > encoder_high_wrap_)
        r_encoder_multi_++;
    else if(recv_right_ > encoder_high_wrap_ && recv_right_ < encoder_low_wrap_)
        r_encoder_multi_--;
    else
        r_encoder_multi_ = 0;

    delta_left  = 1.0*(recv_left_  + l_encoder_multi_*(XTARK_ENCODER_MAX-XTARK_ENCODER_MIN) - last_left_ )/encoderResolution_ * meterPerRound_;           //ticks -> meter
    delta_right = 1.0*(recv_right_ + r_encoder_multi_*(XTARK_ENCODER_MAX-XTARK_ENCODER_MIN) - last_right_)/encoderResolution_ * meterPerRound_;      

    last_left_  = recv_left_;
    last_right_ = recv_right_;

    delta_xy_ave_ = (delta_left  + delta_right)/2.0;
    delta_th_     = (delta_right - delta_left)/wheelTrack_;

    if(delta_xy_ave_ != 0)
    {
        odom_delta_x_ = cos(delta_th_)*delta_xy_ave_;
        odom_delta_y_ = -sin(delta_th_)*delta_xy_ave_;

        accumulation_x_ += (cos(accumulation_th_)*odom_delta_x_ - sin(accumulation_th_)*odom_delta_y_);
        accumulation_y_ += (sin(accumulation_th_)*odom_delta_x_ + cos(accumulation_th_)*odom_delta_y_);
    }
    if(delta_th_ != 0)
    {
        accumulation_th_ += delta_th_;
    }
}

void XtarkKinematic::getOdom(double* odom)
{
    odom[0] = accumulation_x_;
    odom[1] = accumulation_y_;
    odom[2] = accumulation_th_;
}

void XtarkKinematic::initKinematic()
{
    encoder_high_wrap_ = (XTARK_ENCODER_MAX-XTARK_ENCODER_MIN)*0.8 + XTARK_ENCODER_MIN;
    encoder_low_wrap_  = (XTARK_ENCODER_MAX-XTARK_ENCODER_MIN)*0.2 + XTARK_ENCODER_MIN;
    meterPerRound_     = (wheelDiameter_*M_PI);

    accumulation_x_    = 0;
    accumulation_y_    = 0;
    accumulation_th_   = 0;

    last_left_         = 0;
    last_right_        = 0;

    l_encoder_multi_   = 0;
    r_encoder_multi_   = 0;
}

void XtarkKinematic::updateVelocity(const int16_t l_wheel_rpm, const int16_t r_wheel_rpm, double* odom_vel)
{
    double left_wheel_speed_,right_wheel_speed_;

    double linear_speed,angular_speed;

    left_wheel_speed_ = (double)l_wheel_rpm/10*meterPerRound_/60;
    right_wheel_speed_ = (double)r_wheel_rpm/10*meterPerRound_/60;

    linear_speed  = (left_wheel_speed_   + right_wheel_speed_)/2;
    angular_speed = (right_wheel_speed_  - left_wheel_speed_)/wheelTrack_;

    odom_vel[0] = linear_speed;
    odom_vel[1] = angular_speed;
}


/*
int main(void)
{
    geometry_msgs::msg::Twist twist_;
    int16_t wheelspeed[2];

    int32_t left_count = 0;
    int32_t right_count = 0;

    int16_t left_rpm = 1000;
    int16_t right_rpm = 1000;

    double vel_[2];

    double odom_[3];


    PhantomKinematic kinematic(0.2,0.2,16384);
    kinematic.initKinematic();

    printf("Test Odom Running....\r\n");

    while(1)
    {
        left_count+=500;
        right_count+=500;
        printf("---Input : Left Count - [%d]  Right Cound - [%d]\r\n",left_count,right_count);
        kinematic.updateOdom(left_count,right_count);
        kinematic.getOdom(odom_);
        printf("---Output:  x: %.3f   y:%.3f  th:%.3f \r\n",odom_[0],odom_[1],odom_[2]);
        printf("\r\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    printf("Test UpdateWheelSpeed....\r\n");
    twist_.linear.x  = 1.0;
    twist_.angular.z = 0.0;
    printf("---Input : linear_X - [%.3f]  angular_Y - [%.3f]\r\n",twist_.linear.x,twist_.angular.z);
    kinematic.updateWheelSpeed(twist_ , wheelspeed);
    printf("---Output: Left wheel %d rpm   Right wheel %d rpm \r\n",wheelspeed[0],wheelspeed[1]);
    printf("\r\n");

    printf("Test updateOdom....\r\n");
    printf("---Input : Left Count - [%d]  Right Cound - [%d]\r\n",left_count,right_count);
    kinematic.updateOdom(left_count,right_count);
    kinematic.getOdom(odom_);
    printf("---Output:  x: %.3f   y:%.3f  th:%.3f \r\n",odom_[0],odom_[1],odom_[2]);
    printf("\r\n");

    printf("Test GetVelocity....\r\n");
    printf("---Input : Left RPM - [%d]  Right RPM - [%d]\r\n",left_rpm,right_rpm);
    kinematic.getVelocity(left_rpm,right_rpm,vel_);
    printf("---Output:  v_x: %.3f   v_th:%.3f \r\n",vel_[0],vel_[1]);

    printf("Test Done");

    return 0;
    //while(1)
    //{
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //}
}*/
