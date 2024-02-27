#ifndef REGULATOR_H
#define REGULATOR_H

#include "odometry.h"
#include <iostream>

typedef struct robot_motion{
    double rot_speed;
    double u_rot;
    double trans_speed;
    double u_trans;
    double u_integral;
}robot_motion;


class Regulator
{
public:
    Regulator();

    void robot_movement_reg(const double& setX, const double& setY, const RobotCoordRotation &robotCoord, robot_motion& robot_motion_param);

private:

    bool missionStarted;
    bool high_setpoint_angle;

    double integ;

    double old_speed;
};

#endif // REGULATOR_H
