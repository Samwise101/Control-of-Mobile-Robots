#include "odometry.h"

odometry::odometry()
{
    encl_new = 0.0;
    encr_new = 0.0;
    robotStarted = true;
    missionStarted = false;
}

void odometry::robot_odometry(const TKobukiData& Kobuki_data, bool useGyro, RobotCoordRotation& robotCoord)
{
    int encl_old{0};
    int encr_old{0};
    int encl_diff{0};
    int encr_diff{0};
    int gyro_old{0};

    if(robotStarted){
        encl_old = Kobuki_data.EncoderLeft;
        encr_old = Kobuki_data.EncoderRight;
        gyro_old = Kobuki_data.GyroAngle;
        robotStarted = false;
    }
    else{
        encl_old = encl_new;
        encr_old = encr_new;
    }

    encl_new = Kobuki_data.EncoderLeft;
    encr_new = Kobuki_data.EncoderRight;

    if((encl_old - encl_new) < -(UINT16_MAX/2)){
        encl_diff = (encl_new - encl_old) - UINT16_MAX;
    }
    else if((encl_old - encl_new) > (UINT16_MAX/2)){
        encl_diff = UINT16_MAX - encl_old + encl_new;
    }
    else{
        encl_diff = encl_new - encl_old;
    }

    if((encr_old - encr_new) < -(UINT16_MAX/2)){
        encr_diff = (encr_new - encr_old) - UINT16_MAX;
    }
    else if((encr_old - encr_new) > (UINT16_MAX/2)){
        encr_diff = UINT16_MAX - encr_old + encr_new;
    }
    else{
        encr_diff = encr_new - encr_old;
    }

    double lkl = tickToMeter*(encl_diff);
    double lkr = tickToMeter*(encr_diff);
    double lk = (lkr + lkl)/2;

    lr += lkr;
    ll += lkl;
    l += lk;

    double delta_alpha{0};

    if(useGyro){

        double alpha = (Kobuki_data.GyroAngle - gyro_old)/100;

        if(alpha < 0){
            alpha = 360 + alpha;
        }

        robotCoord.a = alpha;
    }
    else{
        delta_alpha = (lkr - lkl)/b;
        robotCoord.a = robotCoord.a + delta_alpha*180/PI;
    }

    if(robotCoord.a > 360.0){
        robotCoord.a = robotCoord.a - 360.0;
    }

    robotCoord.x = robotCoord.x + lk*cos(robotCoord.a*TO_RADIANS);
    robotCoord.y = robotCoord.y + lk*sin(robotCoord.a*TO_RADIANS);
}
