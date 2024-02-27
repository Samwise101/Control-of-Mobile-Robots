#include "regulator.h"

Regulator::Regulator()
{
    missionStarted = false;
    high_setpoint_angle = false;

    old_speed = 0.0;

    integ = 0.0;
}

void Regulator::robot_movement_reg(const double& setX, const double& setY, const RobotCoordRotation &robotCoord, robot_motion& robot_motion_param)
{
    double treshHold = 0.05;
    double rot_dead_zone = 45;

    double ex = (setX - robotCoord.x)*1000.0;
    double ey = (setY - robotCoord.y)*1000.0;

    treshHold = treshHold*1000.0;

    double eRot = std::atan2(ey,ex) - robotCoord.a*TO_RADIANS;
    eRot = std::atan2(std::sin(eRot), std::cos(eRot));
    double eDist = sqrt(ex*ex + ey*ey);

    if(eDist < treshHold){
        integ = 0;
        robot_motion_param.rot_speed = 0.0;
        robot_motion_param.trans_speed = 0.0;
        return;
    }

    integ += eDist * 0.025;

    if (integ > 25.0)
        integ = 25.0;
    else if (integ < -25.0)
        integ = -25.0;

    if(eRot < rot_dead_zone*TO_RADIANS && eRot > -rot_dead_zone*TO_RADIANS){

        robot_motion_param.trans_speed = robot_motion_param.u_trans*eDist + robot_motion_param.u_integral*integ;

        if(old_speed < robot_motion_param.trans_speed){
            robot_motion_param.trans_speed = old_speed + 2.5;
        }
        if(robot_motion_param.trans_speed > 300.0){
            robot_motion_param.trans_speed = 300.0;
        }
        old_speed = robot_motion_param.trans_speed;
    }

    robot_motion_param.rot_speed = robot_motion_param.u_rot*eRot;;
}

