#ifndef XTARK_KINEMATIC_H
#define XTARK_KINEMATIC_H

#include <string>
#include <vector>
#include <math.h>
#include <chrono>
#include <thread>
#include "geometry_msgs/Twist.h"

#define XTARK_LINEAR_SPEED_LIMIT  1.5     // m/s
#define XTARK_ANGULAR_SPEED_LIMIT 3       // rad/s

#define XTARK_ENCODER_MAX     2147483647
#define XTARK_ENCODER_MIN     -2147483648

class XtarkKinematic
{
    public:
        XtarkKinematic(double wheelTrack, double wheelDiameter,double encoderResolution);
        ~XtarkKinematic();
        void    updateWheelSpeed(geometry_msgs::Twist& vel,int16_t* wheelspeed);
        void    updateOdom(const int32_t leftCounts, const int32_t rightCounts);
        void    getOdom(double* odom_);
        void    updateVelocity(const int16_t l_wheel_rpm, const int16_t r_wheel_rpm, double* odom_vel);
        void    initKinematic();
        void    setKinematicParams(double wheelTrack,double wheelDiameter, double encoderResolution);

    private:

        double wheelTrack_;
        double wheelDiameter_;
        double encoderResolution_;
        double meterPerRound_;

        double  accumulation_x_;
        double  accumulation_y_;
        double  accumulation_th_;

        int64_t recv_left_;
        int64_t recv_right_;

        int64_t last_left_;
        int64_t last_right_;

        int32_t encoder_low_wrap_;
        int32_t encoder_high_wrap_;

        int l_encoder_multi_;
        int r_encoder_multi_;

        geometry_msgs::Twist   current_twist_;

};


#endif
