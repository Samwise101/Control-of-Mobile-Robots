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
    double rot_dead_zone = PI/4;

    double ex = (setX - robotCoord.x)*1000.0;
    double ey = -(setY - robotCoord.y)*1000.0;
    double eDist = sqrt(ex*ex + ey*ey);


    treshHold = treshHold*1000.0;

    double eRot = std::atan2(ey,ex) - robotCoord.a*TO_RADIANS;
    eRot = std::atan2(std::sin(eRot), std::cos(eRot));

    std::cout << "Dist=" << eDist << ", Rot=" << eRot << ", SetX=" << setX << ", SetY=" << setY << std::endl;

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

    if(eRot < rot_dead_zone && eRot > -rot_dead_zone){

        robot_motion_param.trans_speed = robot_motion_param.u_trans*eDist;

        if(old_speed < robot_motion_param.trans_speed){
            robot_motion_param.trans_speed = old_speed + 2.5;
        }
        if(robot_motion_param.trans_speed > 200.0){
            robot_motion_param.trans_speed = 200.0;
        }
        old_speed = robot_motion_param.trans_speed;
    }

    robot_motion_param.rot_speed = robot_motion_param.u_rot*eRot;;
}

