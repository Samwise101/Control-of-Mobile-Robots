#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <regex>
#include <thread>
#include <chrono>
#include <cmath>
#include <utility>

#define PI  3.14159 /* pi */

void MainWindow::get_laserdata_and_write_to_map(double robotX, double robotY, double robotA, double setX, double setY)
{
    if(!tempSetPoint.xn.empty()){
        return;
    }

    double ex = (setX - robotX);
    double ey = (setY - robotY);
    double eDist = sqrt(ex*ex + ey*ey);

    double eRot = std::atan2(ey,ex) - robotA;
    eRot = std::atan2(std::sin(eRot), std::cos(eRot));
    if(eRot < 0){
        eRot  = eRot +2*PI;
    }
    if(eRot > 2*PI){
        eRot = eRot - 2*PI;
    }

    std::cout << "eRot = " << eRot << std::endl;

    double angle_threshold = std::atan2(0.20, eDist);
    double distance_threshold = 2.0;

    double leftThreshold = eRot + angle_threshold;
    double rightThreshold = eRot - angle_threshold;

    if(leftThreshold < 0){
        leftThreshold  = leftThreshold + 2*PI;
    }
    if(leftThreshold > 2*PI){
        leftThreshold = leftThreshold - 2*PI;
    }

    if(rightThreshold < 0){
        rightThreshold  = rightThreshold + 2*PI;
    }
    if(rightThreshold > 2*PI){
        rightThreshold = rightThreshold - 2*PI;
    }

    mutex mux;
    mux.lock();
    bool obstacleDetected{false};
    int obstacleLidarIndex{-1};
    double oldLidarDistLeft{-1};
    double oldLidarAngleLeft{-1};
    double oldLidarDistRight{-1};
    double oldLidarAngleRight{-1};

    for(int i = 0; i < copyOfLaserData.numberOfScans; i++){
        double lidarAngle = (360-copyOfLaserData.Data[i].scanAngle)*TO_RADIANS;
        double lidarDist = copyOfLaserData.Data[i].scanDistance/1000.0;

        if(lidarDist <= 0.25)
            continue;

        if(leftThreshold >= 0 && leftThreshold <= PI/2 && rightThreshold >= 3*PI/2 && rightThreshold <= 2*PI){
            if(((lidarAngle >= 0 && lidarAngle <= leftThreshold && lidarAngle < PI/2) || (lidarAngle >= rightThreshold && lidarAngle <= 2*PI && lidarAngle > 3*PI/2)) && lidarDist < distance_threshold && lidarDist <= eDist){
                obstacleDetected = true;
            }
        }
        else if(lidarAngle <= leftThreshold && lidarAngle >= rightThreshold && lidarDist < distance_threshold && lidarDist <= eDist){
            obstacleDetected = true;
        }

        if(obstacleDetected){
            obstacleCoord.x = robotX + lidarDist*std::cos(lidarAngle + robotA);
            obstacleCoord.y = robotY + lidarDist*std::sin(lidarAngle + robotA);
            std::cout << "Found obstacle at: xp = " << obstacleCoord.x << ", yp =" << obstacleCoord.y << std::endl;
            obstacleLidarIndex = i;
            oldLidarDistLeft = lidarDist;
            oldLidarAngleLeft = lidarAngle;
            oldLidarDistRight = lidarDist;
            oldLidarAngleRight = lidarAngle;
            break;
        }
    }

    if(!obstacleDetected){
        obstacleCoord.x = -1;
        obstacleCoord.y = -1;
        mux.unlock();
        return;
    }
    mux.unlock();

    leftThreshold = eRot + PI/2;
    rightThreshold = eRot - PI/2;

    if(leftThreshold < 0){
        leftThreshold  = leftThreshold + 2*PI;
    }
    if(leftThreshold > 2*PI){
        leftThreshold = leftThreshold - 2*PI;
    }

    if(rightThreshold < 0){
        rightThreshold  = rightThreshold + 2*PI;
    }
    if(rightThreshold > 2*PI){
        rightThreshold = rightThreshold - 2*PI;
    }

    double leftCornerDist{-1.0};
    double leftCornerAngle{-1.0};
    double rightCornerDist{-1.0};
    double rightCornerAngle{-1.0};

    bool leftCornerDetected{false};
    mux.lock();
    // trying to find the left obstacle corner
    for(int i = obstacleLidarIndex; ;i--){
        if(i <= 0){
            i = copyOfLaserData.numberOfScans + i;
        }
        double lidarAngle = (360-copyOfLaserData.Data[i].scanAngle)*TO_RADIANS;
        double lidarDist = copyOfLaserData.Data[i].scanDistance/1000.0;

        if(lidarDist <= 0.25)
            continue;

        if(leftThreshold > PI/2){
            if(lidarAngle >= leftThreshold)
                break;
            else if(std::abs(oldLidarDistLeft-lidarDist) >= 0.5){
                leftCornerDetected = true;
            }
        }
        else{
            if(lidarAngle >= leftThreshold && lidarAngle <= rightThreshold)
                break;
            else if(std::abs(oldLidarDistLeft-lidarDist) >= 0.5){
                leftCornerDetected = true;
            }
        }

        if(leftCornerDetected){
            obstacleCornerLeft.x = robotX + oldLidarDistLeft*std::cos(oldLidarAngleLeft + robotA);
            obstacleCornerLeft.y = robotY + oldLidarDistLeft*std::sin(oldLidarAngleLeft + robotA);
            leftCornerAngle = oldLidarAngleLeft;
            leftCornerDist = oldLidarDistLeft;
            std::cout << "Found LEFT obstacle corner: xp = " << obstacleCornerLeft.x << ", yp =" << obstacleCornerLeft.y << std::endl;
            break;
        }
        else{
            oldLidarAngleLeft = lidarAngle;
            oldLidarDistLeft = lidarDist;
        }
    }

    if(!leftCornerDetected){
        obstacleCornerLeft.x = -1;
        obstacleCornerLeft.y = -1;
    }
    mux.unlock();

    bool rightCornerDetected{false};
    mux.lock();
//  trying to find the left obstacle corner
    for(int i = obstacleLidarIndex; ; i++){
        if(i >= copyOfLaserData.numberOfScans){
            i = i-copyOfLaserData.numberOfScans;
        }
        double lidarAngle = (360-copyOfLaserData.Data[i].scanAngle)*TO_RADIANS;
        double lidarDist = copyOfLaserData.Data[i].scanDistance/1000.0;

        if(lidarDist <= 0.25)
            continue;

        if(rightThreshold < 3*PI/2){
            if(lidarAngle <= rightThreshold)
                break;
            else if(std::abs(oldLidarDistRight-lidarDist) >= 0.5){
                rightCornerDetected = true;
            }
        }
        else{
            if(lidarAngle >= leftThreshold && lidarAngle <= rightThreshold)
                break;
            else if(std::abs(oldLidarDistRight-lidarDist) >= 0.5){
                rightCornerDetected = true;
            }
        }

        if(rightCornerDetected){
            obstacleCornerRight.x = robotX + oldLidarDistRight*std::cos(oldLidarAngleRight + robotA);
            obstacleCornerRight.y = robotY + oldLidarDistRight*std::sin(oldLidarAngleRight + robotA);
            rightCornerAngle = oldLidarAngleRight;
            rightCornerDist = oldLidarDistRight;
            std::cout << "Found RIGHT obstacle corner: xp = " << obstacleCornerRight.x << ", yp =" << obstacleCornerRight.y << std::endl;
            break;
        }
        else{
            oldLidarAngleRight = lidarAngle;
            oldLidarDistRight = lidarDist;
        }
    }

    if(!rightCornerDetected){
        obstacleCornerRight.x = -1;
        obstacleCornerRight.y = -1;
    }
    mux.unlock();

    bool leftAccessible{false};
    bool rightAccessible{false};
    double calculatedAngle{};

    double xpRight{};
    double ypRight{};
    double xpLeft{};
    double ypLeft{};

    double robotoffset = 0.3;

    if(leftCornerDetected){
        double angle_offset = asin(robotoffset/leftCornerDist);
        calculatedAngle = leftCornerAngle + angle_offset;

        if(calculatedAngle < 0){
            calculatedAngle  = calculatedAngle + 2*PI;
        }
        if(calculatedAngle > 2*PI){
            calculatedAngle = calculatedAngle - 2*PI;
        }

        double temp = std::sqrt(robotoffset*robotoffset+rightCornerDist*rightCornerDist);
        qDebug() << temp;

        xpLeft = robotX + temp*std::cos(calculatedAngle + robotA);
        ypLeft = robotY + temp*std::sin(calculatedAngle + robotA);

        if(checkAccessibility(xpLeft, ypLeft)){
            std::cout << "Left corner is accessible!" << std::endl;
            leftAccessible = true;
        }
        else{
            std::cout << "Left corner is NOT accessible!" << std::endl;
        }
    }

    if(rightCornerDetected){
        double angle_offset = asin(robotoffset/rightCornerDist);
        calculatedAngle = rightCornerAngle - angle_offset;

        if(calculatedAngle < 0){
            calculatedAngle  = calculatedAngle + 2*PI;
        }
        if(calculatedAngle > 2*PI){
            calculatedAngle = calculatedAngle - 2*PI;
        }

        double temp = std::sqrt(robotoffset*robotoffset+rightCornerDist*rightCornerDist);

        qDebug() << temp;

        xpRight = robotX + temp*std::cos(calculatedAngle + robotA);
        ypRight = robotY + temp*std::sin(calculatedAngle + robotA);

        if(checkAccessibility(xpRight, ypRight)){
            std::cout << "Right corner is accessible!" << std::endl;
            rightAccessible = true;
        }
        else{
            std::cout << "Right corner is NOT accessible!" << std::endl;
        }
    }

    if(rightAccessible && leftAccessible){
        double leftDist = calculateDistance(xpLeft, ypLeft, setX, setY, robotX, robotY);
        double rightDist = calculateDistance(xpRight, ypRight, setX, setY, robotX, robotY);

        std::cout << "Left path distance = " << leftDist << std::endl;
        std::cout << "Right path distance = " << rightDist << std::endl;

        if(leftDist > rightDist){
            tempSetPoint.xn.push_back(xpRight);
            tempSetPoint.yn.push_back(ypRight);
        }
        else{
            tempSetPoint.xn.push_back(xpLeft);
            tempSetPoint.yn.push_back(ypLeft);
        }
    }
    else if(rightAccessible){
        tempSetPoint.xn.push_back(xpRight);
        tempSetPoint.yn.push_back(ypRight);
    }
    else if(leftAccessible){
        tempSetPoint.xn.push_back(xpLeft);
        tempSetPoint.yn.push_back(ypLeft);
    }
}

double MainWindow::calculateDistance(double& xp, double& yp, double& setX, double& setY, double& robotX, double& robotY){
    double ex = xp - robotX;
    double ey = yp - robotY;
    double eDist = std::sqrt(ex*ex+ey*ey);

    double temp = eDist;

    ex = setX - xp;
    ey = setY - yp;
    eDist = std::sqrt(ex*ex+ey*ey);

    temp += eDist;
    return temp;
}

bool MainWindow::checkAccessibility(double xp, double yp)
{
    std::cout << "xp check= " << xp << ", yp check= " << yp << std::endl;
    double search_offset = 0.16;
    mutex m;
    m.lock();
    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++){
        double lidarDist = copyOfLaserData.Data[k].scanDistance/1000.0;
        double lidarAngle = (360-copyOfLaserData.Data[k].scanAngle)*TO_RADIANS;

        double xp2 = lidarDist*std::cos(lidarAngle);
        double yp2 = lidarDist*std::sin(lidarAngle);

        double ex = xp - xp2;
        double ey = yp - yp2;

        double distToObsCorner = std::sqrt((ex*ex) + (ey*ey));

        if(distToObsCorner <= search_offset){
            std::cout << "Distance to obstacle corner point : " << distToObsCorner << std::endl;
            std::cout << "Corner is inaccessible!" << std::endl;
            std::cout << "xp = " << xp2 << ", yp = " << yp2 << std::endl;
            m.unlock();
            return false;
        }
    }
    m.unlock();
    return true;
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    set_robot_connect_data();
    datacounter=0;
    isTest = true;
    actIndex=-1;
    useCamera1=false;

    ui->xn->setText("0.0");
    ui->yn->setText("0.0");

    mission_started = false;

    robotCoord.a = 0.0;
    robotCoord.x = 0.0;
    robotCoord.y = 0.0;

    robotCoordMap.a = 0.0;
    robotCoordMap.x = 0.5;
    robotCoordMap.y = 4.8;

    datacounter = 0;
    canStart = false;
    isRotating = false;
    robotStop = false;
    obstacle_detected = false;

    controlType = 0;

    zone_corner_left.x = -1;
    zone_corner_left.y = -1;

    zone_corner_right.x = -1;
    zone_corner_right.y = -1;

    obstacleCoord.x = -1;
    obstacleCoord.y = -1;
    obstacleCornerLeft.x = -1;
    obstacleCornerLeft.y = -1;
    obstacleCornerRight.x = -1;
    obstacleCornerRight.y = -1;

    // disable 4. ulohy lebo pouzviam len kod z nej
    ui->pushButton_10->setDisabled(true);
}

MainWindow::~MainWindow()
{
    isStoped = true;
    mappingThread.join();
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero2;
    pero2.setStyle(Qt::SolidLine);
    pero2.setWidth(3);
    pero2.setColor(Qt::yellow);
    QRect rect(20,120,700,500);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1)
        {
            int offsetX = 15;
            int offsetY = 20;
            int width = 40;
            int height = 40;

            painter.setPen(pero2);

            int x = rect.width()/2+rect.topLeft().x()-offsetX;
            int y = rect.height()/2+rect.topLeft().y()-offsetY;
            int centerX = x+width/2;
            int centerY = y+height/2;

            if(rect.contains(x,y)){
                painter.drawEllipse((centerX-width/2), (centerY-height/2), width, height);
                painter.setBrush(Qt::yellow);
                QPointF *points = new QPointF[3];
                points[0] = QPointF((centerX+width/2-5),centerY);
                points[1] = QPointF(centerX,(centerY-height/2));
                points[2] = QPointF((centerX-width/2+5),centerY);;
                painter.drawPolygon(points,3);
                delete[] points;
            }

            updateLaserPicture=0;

            mutex mux;
            mux.lock();
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                double lidarDist = copyOfLaserData.Data[k].scanDistance/20;

                double lidarAngle = (360-copyOfLaserData.Data[k].scanAngle)*TO_RADIANS;

                pero2.setStyle(Qt::SolidLine);
                pero2.setWidth(3);
                pero2.setColor(Qt::yellow);
                painter.setBrush(Qt::yellow);
                painter.setPen(pero2);

                int xp=rect.width()-(rect.width()/2+lidarDist*2*sin(lidarAngle))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+lidarDist*2*cos(lidarAngle))+rect.topLeft().y();
                painter.drawEllipse(QPoint(xp, yp),2,2);
            }

            if(!set_point.xn.empty()){
                pero2.setStyle(Qt::SolidLine);
                pero2.setWidth(3);
                pero2.setColor(Qt::magenta);
                painter.setBrush(Qt::magenta);
                painter.setPen(pero2);

                for(int i = 0; i < set_point.xn.size(); i++){
                    double ex = (set_point.xn[i] - robotCoord.x);
                    double ey = (set_point.yn[i] - robotCoord.y);
                    double eDist = sqrt(ex*ex + ey*ey)/20.0*1000.0;

                    double eRot = std::atan2(ey,ex) - robotCoord.a*TO_RADIANS;
                    eRot = std::atan2(std::sin(eRot), std::cos(eRot));
                    if(eRot < 0){
                        eRot  = eRot +2*PI;
                    }
                    if(eRot > 2*PI){
                        eRot = eRot - 2*PI;
                    }

                    std::cout << "eRot draw = " << eRot << std::endl;

                    int xp=rect.width()-(rect.width()/2+eDist*2*sin(eRot))+rect.topLeft().x();
                    int yp=rect.height()-(rect.height()/2+eDist*2*cos(eRot))+rect.topLeft().y();

                    painter.drawEllipse(QPoint(xp, yp),2,2);
                }
            }

            if(!tempSetPoint.xn.empty()){
                pero2.setStyle(Qt::SolidLine);
                pero2.setWidth(3);
                pero2.setColor(Qt::cyan);
                painter.setBrush(Qt::cyan);
                painter.setPen(pero2);
                for(int i = 0; i < tempSetPoint.xn.size(); i++){
                    double ex = (tempSetPoint.xn[i] - robotCoord.x)*1000;
                    double ey = (tempSetPoint.yn[i] - robotCoord.y)*1000;
                    double eDist = sqrt(ex*ex + ey*ey)/20;

                    double eRot = std::atan2(ey,ex)-robotCoord.a*TO_RADIANS;
                    eRot = std::atan2(std::sin(eRot), std::cos(eRot));

                    int xp=rect.width()-(rect.width()/2+eDist*2*sin(eRot))+rect.topLeft().x();
                    int yp=rect.height()-(rect.height()/2+eDist*2*cos(eRot))+rect.topLeft().y();

                    painter.drawEllipse(QPoint(xp, yp),2,2);
                }
            }
            mux.unlock();
            mux.lock();
            if(obstacleCoord.x != -1 && obstacleCoord.y != -1){
                pero2.setStyle(Qt::SolidLine);
                pero2.setWidth(3);
                pero2.setColor(Qt::red);
                painter.setBrush(Qt::red);
                painter.setPen(pero2);
                double ex = (obstacleCoord.x - robotCoord.x)*1000;
                double ey = (obstacleCoord.y - robotCoord.y)*1000;
                double eDist = sqrt(ex*ex + ey*ey)/20;

                double eRot = std::atan2(ey,ex)-robotCoord.a*TO_RADIANS;
                eRot = std::atan2(std::sin(eRot), std::cos(eRot));

                int xp=rect.width()-(rect.width()/2+eDist*2*sin(eRot))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+eDist*2*cos(eRot))+rect.topLeft().y();

                painter.drawEllipse(QPoint(xp, yp),2,2);
            }
            if(obstacleCornerRight.x != -1 && obstacleCornerRight.y != -1){
                pero2.setStyle(Qt::SolidLine);
                pero2.setWidth(3);
                pero2.setColor(Qt::darkCyan);
                painter.setBrush(Qt::darkCyan);
                painter.setPen(pero2);
                double ex = (obstacleCornerRight.x - robotCoord.x)*1000;
                double ey = (obstacleCornerRight.y - robotCoord.y)*1000;
                double eDist = sqrt(ex*ex + ey*ey)/20;

                double eRot = std::atan2(ey,ex)-robotCoord.a*TO_RADIANS;
                eRot = std::atan2(std::sin(eRot), std::cos(eRot));


                int xp=rect.width()-(rect.width()/2+eDist*2*sin(eRot))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+eDist*2*cos(eRot))+rect.topLeft().y();

                painter.drawEllipse(QPoint(xp, yp),2,2);
            }
            if(obstacleCornerLeft.x != -1 && obstacleCornerLeft.y != -1){
                pero2.setStyle(Qt::SolidLine);
                pero2.setWidth(3);
                pero2.setColor(Qt::darkGray);
                painter.setBrush(Qt::darkGray);
                painter.setPen(pero2);
                double ex = (obstacleCornerLeft.x - robotCoord.x)*1000;
                double ey = (obstacleCornerLeft.y - robotCoord.y)*1000;
                double eDist = sqrt(ex*ex + ey*ey)/20;

                double eRot = std::atan2(ey,ex)-robotCoord.a*TO_RADIANS;
                eRot = std::atan2(std::sin(eRot), std::cos(eRot));


                int xp=rect.width()-(rect.width()/2+eDist*2*sin(eRot))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+eDist*2*cos(eRot))+rect.topLeft().y();

                painter.drawEllipse(QPoint(xp, yp),2,2);
            }
            mux.unlock();
        }
    }
}

void MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
    ui->lineEdit_2->setText(QString::number(robotX,10,6));
    ui->lineEdit_3->setText(QString::number(robotY,10,6));
    ui->lineEdit_4->setText(QString::number(robotFi));
}

void MainWindow::setUiValuesForMap(double setPointX, double setPointY)
{
    ui->xn->setText(QString::number(setPointX, 10, 3));
    ui->yn->setText(QString::number(setPointY, 10, 3));
}


int MainWindow::processThisRobot(TKobukiData robotdata)
{
    if(mission_started){
        if(!tempSetPoint.xn.empty()){
            robot_odometry.robot_odometry(robotdata, true, robotCoord, 1);
            robot_motion_reg.robot_movement_reg(tempSetPoint.xn[tempSetPoint.xn.size()-1], tempSetPoint.yn[tempSetPoint.yn.size()-1], robotCoord, robot_motion_param,1);
        }
        else if(controlType == 0 && !set_point.xn.empty()){
            robot_odometry.robot_odometry(robotdata, true, robotCoord, 1);
            robot_motion_reg.robot_movement_reg(set_point.xn[set_point.xn.size() - 1], set_point.yn[set_point.yn.size() - 1], robotCoord, robot_motion_param,1);
        }
        else if(controlType == 1 && !set_point_map.xn.empty() && robotCoordMap.x != -1){
            robot_odometry.robot_odometry(robotdata, true, robotCoordMap, -1);
            robot_motion_reg.robot_movement_reg(set_point_map.xn[set_point_map.xn.size() - 1], set_point_map.yn[set_point_map.yn.size() - 1], robotCoordMap, robot_motion_param,-1);
        }

        mutex mux;
        forwardspeed = robot_motion_param.trans_speed;
        rotationspeed = robot_motion_param.rot_speed;

//        forwardspeed = 0;
//        rotationspeed = 0;


        if(forwardspeed == 0.0 && rotationspeed != 0.0){
            mux.lock();
            isRotating = true;
            mux.unlock();
            robot.setRotationSpeed(rotationspeed);
        }
        else if(forwardspeed != 0.0 && rotationspeed != 0.0){
            mux.lock();
            isRotating = false;
            mux.unlock();
            robot.setArcSpeed(forwardspeed, forwardspeed/rotationspeed);
        }
        else if(forwardspeed != 0.0 && rotationspeed == 0.0){
            robot.setArcSpeed(forwardspeed, 0);
        }
        else if(forwardspeed == 0.0 && rotationspeed == 0.0){
            forwardspeed = 0;
            robot.setArcSpeed(forwardspeed, 0);

            if(!set_point.xn.empty()){
                if((robotCoord.x >= set_point.xn[set_point.xn.size()-1]-0.5) && (robotCoord.x <= set_point.xn[set_point.xn.size()-1]+0.5) &&
                    (robotCoord.y >= set_point.yn[set_point.yn.size()-1]-0.5) && (robotCoord.y <= set_point.yn[set_point.yn.size()-1]+0.5)){
                    set_point.xn.pop_back();
                    set_point.yn.pop_back();
                    zone_corner_left.x = -1;
                    zone_corner_left.y = -1;
                    zone_corner_right.x = -1;
                    zone_corner_right.y = -1;
                }
            }
            if(!tempSetPoint.xn.empty()){
                if((robotCoord.x >= tempSetPoint.xn[tempSetPoint.xn.size()-1]-0.5) && (robotCoord.x <= tempSetPoint.xn[tempSetPoint.xn.size()-1]+0.5) &&
                    (robotCoord.y >= tempSetPoint.yn[tempSetPoint.yn.size()-1]-0.5) && (robotCoord.y <= tempSetPoint.yn[tempSetPoint.yn.size()-1]+0.5)){
                    tempSetPoint.xn.pop_back();
                    tempSetPoint.yn.pop_back();
                }
            }
        }
    }
    else{
        forwardspeed = 0;
        robot.setArcSpeed(forwardspeed, 0);
    }

    if(datacounter%5)
    {
        if(controlType == 0)
            emit uiValuesChanged(robotCoord.x, robotCoord.y, robotCoord.a);
        else
            emit uiValuesChanged(robotCoordMap.x, robotCoordMap.y, robotCoordMap.a);
    }
    datacounter++;

    return 0;
}

void MainWindow::set_robot_connect_data()
{
    robot_data.robot_ip = "127.0.0.1";
    robot_data.robot_port = 53000;
    robot_data.robot_port_me = 5300;
    robot_data.lidar_port = 52999;
    robot_data.lidar_port_me = 5299;
    robot_data.camera_port = "8889";
}


int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    robotX = robotCoord.x;
    robotY = robotCoord.y;
    robotAngle = robotCoord.a*TO_RADIANS;
//    if(mission_started && !tempSetPoint.xn.empty()){
//        f = std::bind(&MainWindow::get_laserdata_and_write_to_map,this,robotX, robotY, robotAngle, tempSetPoint.xn[tempSetPoint.xn.size()-1],tempSetPoint.yn[tempSetPoint.yn.size()-1]);
//        std::async(std::launch::async, f);
//    }
    if(mission_started && !set_point.xn.empty()){
        f = std::bind(&MainWindow::get_laserdata_and_write_to_map,this,robotX, robotY, robotAngle, set_point.xn[set_point.xn.size()-1],set_point.yn[set_point.yn.size()-1]);
        std::async(std::launch::async, f);
    }
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

bool isValidIpAddress(const std::string &ipAddress) {
    // Regular expression to match valid IPv4 addresses
    std::regex ipv4_regex("^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");

    return std::regex_match(ipAddress, ipv4_regex);
}


int MainWindow::processThisCamera(cv::Mat cameraData)
{
    cameraData.copyTo(frame[(actIndex+1)%3]);
    actIndex=(actIndex+1)%3;
    updateLaserPicture=1;
    return 0;
}
void MainWindow::on_startButton_clicked() //start button
{
    forwardspeed=0;
    rotationspeed=0;

    if(isValidIpAddress(ui->comboBox->currentText().toStdString())){
        robot_data.robot_ip = ui->comboBox->currentText().toStdString();
        std::cout << "Correct ip" << std::endl;
    }
    else{
        robot_data.robot_ip = "127.0.0.1";
    }

    robot_data.camera_link = "http://" + robot_data.robot_ip + ":"+robot_data.camera_port + "/stream.mjpg";

    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));
    connect(this,SIGNAL(uiValuesChangedMap(double, double)),this,SLOT(setUiValuesForMap(double, double)));

    robot.setLaserParameters(robot_data.robot_ip,robot_data.lidar_port,robot_data.lidar_port_me,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(robot_data.robot_ip,robot_data.robot_port,robot_data.robot_port_me,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    robot.setCameraParameters(robot_data.camera_link,std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));

    robot.robotStart();
    emit uiValuesChanged(robotCoord.x, robotCoord.y, robotCoord.a);
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    forwardspeed = 0.0;
    rotationspeed = 0.0;
}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);
}

void MainWindow::on_pushButton_6_clicked() //left
{
    robot.setRotationSpeed(3.14159/2);
}

void MainWindow::on_pushButton_5_clicked()//right
{
    robot.setRotationSpeed(-3.14159/2);
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    forwardspeed = 0.0;
    rotationspeed = 0.0;
}

void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;
        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;
        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}

void MainWindow::on_startMissionButton_clicked()
{
    if(controlType == 0){
        if(!mission_started && !set_point.xn.empty()){
            mission_started = true;
            ui->startMissionButton->setText("Stop mission");
        }
        else if(mission_started){
            mission_started = false;
            ui->startMissionButton->setText("Start mission");
        }
    }
    else{
        if(!mission_started && !set_point_map.xn.empty()){
            mission_started = true;
            ui->startMissionButton->setText("Stop mission");
            int index = set_point_map.xn.size()-1;
            emit uiValuesChangedMap(set_point_map.xn[index], set_point_map.yn[index]);
        }
        else if(mission_started){
            mission_started = false;
            ui->startMissionButton->setText("Start mission");
        }
    }
}


void MainWindow::on_pushButton_9_clicked()
{
    if(controlType == 0){
        if(set_point.xn.empty() && set_point.yn.empty()){
            set_point.xn.insert(set_point.xn.begin(),ui->xn->text().toDouble());
            set_point.yn.insert(set_point.yn.begin(),ui->yn->text().toDouble());
            return;
        }
        else if(set_point.xn[0] == ui->xn->text().toDouble() && set_point.yn[0] == ui->yn->text().toDouble())
            return;

        set_point.xn.insert(set_point.xn.begin(),ui->xn->text().toDouble());
        set_point.yn.insert(set_point.yn.begin(),ui->yn->text().toDouble());
    }
}


void MainWindow::on_pushButton_10_clicked()
{

    PathFinding pathFinding(robotCoordMap.x, robotCoordMap.y);

    pathFinding.exec();

    pathFinding.setClickCounter(0);
    pathFinding.setOldClickCounter(0);

    std::vector<QPoint> points = pathFinding.getCorner_points();
    if(points.empty())
        return;

    if(!set_point_map.xn.empty()){
        set_point_map.xn.clear();
        set_point_map.yn.clear();
    }

    for(int i = 0; i < points.size(); i++){
        set_point_map.xn.insert(set_point_map.xn.begin(),points[i].x()/20.0);
        set_point_map.yn.insert(set_point_map.yn.begin(),points[i].y()/20.0);
    }

    points.clear();

    std::cout << "set_point_map size=" << set_point_map.xn.size() << std::endl;
}


void MainWindow::on_pushButton_11_clicked()
{
    if(controlType == 0){
        if(set_point.xn.empty())
            return;

        set_point.xn.clear();
        set_point.yn.clear();
        zone_corner_left.x = -1;
        zone_corner_left.y = -1;
        zone_corner_right.x = -1;
        zone_corner_right.y = -1;
    }
    else{
        if(set_point_map.xn.empty())
            return;

        set_point_map.xn.clear();
        set_point_map.yn.clear();
        zone_corner_left.x = -1;
        zone_corner_left.y = -1;
        zone_corner_right.x = -1;
        zone_corner_right.y = -1;
    }

    mission_started = false;
    ui->startMissionButton->setText("Start mission");
}


void MainWindow::on_comboBox_2_activated(int index)
{
    controlType = index;
    if(controlType == 1){
        ui->xn->setDisabled(true);
        ui->yn->setDisabled(true);

        if(set_point_map.xn.empty())
            return;

        set_point_map.xn.clear();
        set_point_map.yn.clear();
    }
    else{
        ui->xn->setDisabled(false);
        ui->yn->setDisabled(false);

        if(set_point.xn.empty())
            return;

        set_point.xn.clear();
        set_point.yn.clear();
    }
}
