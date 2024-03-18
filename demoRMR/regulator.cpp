#include "regulator.h"

Regulator::Regulator()
{
    missionStarted = false;
    high_setpoint_angle = false;

    old_speed = 0.0;
    old_rot_speed = 0.0;
}

void Regulator::robot_movement_reg(const double& setX, const double& setY, const RobotCoordRotation &robotCoord, robot_motion& robot_motion_param, const int sign)
{
    double treshHold = 0.1;
    double rot_dead_zone = PI/4;

    double ex = (setX - robotCoord.x)*1000.0;
    double ey = sign*(setY - robotCoord.y)*1000.0;
    double eDist = sqrt(ex*ex + ey*ey);


    treshHold = treshHold*1000.0;

    double eRot = std::atan2(ey,ex) - robotCoord.a*TO_RADIANS;
    eRot = std::atan2(std::sin(eRot), std::cos(eRot));

   // std::cout << "Dist=" << eDist << ", Rot=" << eRot << ", SetX=" << setX << ", SetY=" << setY << std::endl;

    if(eDist < treshHold){
        robot_motion_param.rot_speed = 0.0;
        robot_motion_param.trans_speed = 0.0;
        return;
    }

    if(eRot < rot_dead_zone && eRot > -rot_dead_zone){

        robot_motion_param.trans_speed = robot_motion_param.u_trans*eDist;

        if(old_speed < robot_motion_param.trans_speed){
            robot_motion_param.trans_speed = old_speed + 2.5;
        }
        else if(robot_motion_param.trans_speed > 300.0){
            robot_motion_param.trans_speed = 300.0;
        }

        old_speed = robot_motion_param.trans_speed;
    }

    robot_motion_param.rot_speed = robot_motion_param.u_rot*eRot;

    if(old_rot_speed < robot_motion_param.rot_speed){
        robot_motion_param.rot_speed = old_rot_speed + 0.15;
    }
    else if(robot_motion_param.rot_speed > PI/2){
        robot_motion_param.rot_speed = PI/2;
    }

    old_rot_speed = robot_motion_param.rot_speed;
}

