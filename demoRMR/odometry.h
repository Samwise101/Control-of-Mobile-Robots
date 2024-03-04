#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "CKobuki.h"
#include <iostream>

typedef struct
{
    double x;
    double y;
    double a;

}RobotCoordRotation;


class odometry
{
public:
    odometry();

    void robot_odometry(const TKobukiData& Kobuki_data, bool useGyro, RobotCoordRotation& robotCoord);

private:

    double l = 0.0;
    double lr = 0.0;
    double ll = 0.0;

    int alphak_new = 0;

    bool missionStarted;
    bool robotStarted;

    double old_speed = 0.0;

    int gyro_new = 0;

    unsigned short encl_new;
    unsigned short encr_new;
    long double tickToMeter = 0.000085292090497737556558; // [m/tick]
    long double b = 0.23;
};

#endif // ODOMETRY_H
