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
    if(set_point.xn.empty())
        return;

    double angle_trashhold{PI/6};
    double distance_trashhold{1500};

    int obstacle_index{-1};
    double obstacle_distance{-1};

    mutex m;
    m.lock();
    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        double lidarDist = copyOfLaserData.Data[k].scanDistance;

        double lidarAngle = (360-copyOfLaserData.Data[k].scanAngle)*TO_RADIANS;

        double ex = (setX - robotX)*1000;
        double ey = (setY - robotY)*1000;

        double eDist = std::sqrt(std::pow(ey,2) + std::pow(ex,2));

        double eRot = std::atan2(ey,ex) - robotA;
        eRot = std::atan2(std::sin(eRot), std::cos(eRot));

        double leftAngleCheck = eRot + angle_trashhold;
        double rightAngleCheck = eRot - angle_trashhold;

        if(leftAngleCheck < 0){
            leftAngleCheck = 2*PI + (eRot + angle_trashhold);
        }
        else if(leftAngleCheck > 2*PI){
            leftAngleCheck = (eRot + angle_trashhold) - 2*PI;
        }

        if(rightAngleCheck < 0){
            rightAngleCheck = 2*PI + (eRot - angle_trashhold);
        }
        else if(rightAngleCheck > 2*PI){
            rightAngleCheck = (eRot - angle_trashhold) - 2*PI;
        }

        double d_crit{-1};
        if((lidarAngle <= leftAngleCheck) || (lidarAngle >= rightAngleCheck)){
            d_crit = 150/std::abs(std::sin(lidarAngle-robotAngle*TO_RADIANS));
        }

        if(lidarDist <= d_crit && d_crit < eDist && d_crit != -1)
        {
            if(lidarDist < distance_trashhold && !obstacle_detected){
                obstacle_detected = true;
                obstacle_index = k;
                obstacle_distance = lidarDist;
                std::cout << "Obstacle detected, stopping the robot!" << std::endl;
            }
        }
    }
    m.unlock();

    if(!obstacle_detected){
        return;
    }
    if(robotStop){
        return;
    }

    bool leftFound{false};
    bool rightFound{false};

    double leftDist{};
    double rightDist{};
    double oldLidarDist{obstacle_distance};
    double xp{};
    double xp2{};
    double yp{};
    double yp2{};

    m.lock();
    for(int k=obstacle_index; k>=0; k--){
        double lidarDist = copyOfLaserData.Data[k].scanDistance;
        double lidarAngle = copyOfLaserData.Data[k].scanAngle;

        if(lidarDist/10 == 0.0)
            continue;

        if(!leftFound && std::abs(lidarDist - oldLidarDist) > 300 && (lidarAngle < 90 || lidarAngle > 270)){
            leftFound = true;
            lidarDist /= 1000.0;
            xp = lidarDist*std::sin(lidarAngle*TO_RADIANS);
            yp = lidarDist*std::cos(lidarAngle*TO_RADIANS);
            double dist = std::sqrt(std::pow(xp - robotX,2) + std::pow(yp - robotY, 2));
            dist += std::sqrt(std::pow(setX - xp,2) + std::pow(setY - yp, 2));
            leftDist += dist;
            std::cout << "Found left obstacle end, left length = " << leftDist << std::endl;
            break;
        }
        else{
            oldLidarDist = lidarDist;
        }
    }
    m.unlock();


    oldLidarDist = obstacle_distance;


    m.lock();
    for(int k=obstacle_index; k<=copyOfLaserData.numberOfScans; k++){
        double lidarDist = copyOfLaserData.Data[k].scanDistance;
        double lidarAngle = copyOfLaserData.Data[k].scanAngle;

        if(lidarDist/10 == 0.0)
            continue;

        if(!rightFound && std::abs(lidarDist - oldLidarDist) > 300 && (lidarAngle < 90 || lidarAngle > 270)){
            rightFound = true;
            lidarDist /= 1000.0;
            xp2 = lidarDist*std::sin(lidarAngle*TO_RADIANS);
            yp2 = lidarDist*std::cos(lidarAngle*TO_RADIANS);
            double dist = std::sqrt(std::pow(xp2 - robotX,2) + std::pow(yp2 - robotY, 2));
            dist += std::sqrt(std::pow(setX - xp2,2) + std::pow(setY - yp2, 2));
            rightDist += dist;
            std::cout << "Found right obstacle end, right length = " << rightDist << std::endl;
            break;
        }
        else{
            oldLidarDist = lidarDist;
        }
    }
    m.unlock();

    if(leftFound && rightFound){
        if(leftDist >= rightDist){
            set_point.xn.push_back(xp2);
            set_point.yn.push_back(yp2);
        }
        else{
            set_point.xn.push_back(xp);
            set_point.yn.push_back(yp);
        }
    }
    else if(leftFound){
        set_point.xn.push_back(xp);
        set_point.yn.push_back(yp);
    }
    else if(rightFound){
        set_point.xn.push_back(xp2);
        set_point.yn.push_back(yp2);
    }

    robotStop = true;
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

    safe_zone.length = 0.0;
    safe_zone.width = 0.0;

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

                double d_crit = (((centerX/100+150)/20)/std::abs(std::sin(lidarAngle-robotAngle*TO_RADIANS)));

                if(rect.contains(xp,yp)){
                    if(lidarDist <= d_crit && (lidarAngle <= PI/2 || lidarAngle >= 3*PI/2))
                    {
                        pero2.setStyle(Qt::SolidLine);
                        pero2.setWidth(3);
                        pero2.setColor(Qt::red);
                        painter.setBrush(Qt::red);
                        painter.setPen(pero2);
                    }
                    painter.drawEllipse(QPoint(xp, yp),2,2);
                }
            }
            mux.unlock();

            if(!set_point.xn.empty()){
                pero2.setStyle(Qt::SolidLine);
                pero2.setWidth(3);
                pero2.setColor(Qt::magenta);
                painter.setBrush(Qt::magenta);
                painter.setPen(pero2);

                for(int i = 0; i < set_point.xn.size(); i++){
                    double ex = (set_point.xn[i] - robotCoord.x)*1000;
                    double ey = (set_point.yn[i] - robotCoord.y)*1000;
                    double eDist = sqrt(ex*ex + ey*ey)/20;

                    double eRot = std::atan2(ey,ex);
                    eRot = std::atan2(std::sin(eRot), std::cos(eRot)) - robotCoord.a*TO_RADIANS;

                    int xp=rect.width()-(rect.width()/2+eDist*2*sin(eRot))+rect.topLeft().x();
                    int yp=rect.height()-(rect.height()/2+eDist*2*cos(eRot))+rect.topLeft().y();

                    painter.drawEllipse(QPoint(xp, yp),2,2);
                }
            }
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
        if(controlType == 0 && !set_point.xn.empty()){
            robot_odometry.robot_odometry(robotdata, true, robotCoord, 1);
            robot_motion_reg.robot_movement_reg(set_point.xn[set_point.xn.size() - 1], set_point.yn[set_point.yn.size() - 1], robotCoord, robot_motion_param,1);
        }
        else if(controlType == 1 && !set_point_map.xn.empty() && robotCoordMap.x != -1){
            robot_odometry.robot_odometry(robotdata, true, robotCoordMap, -1);
            robot_motion_reg.robot_movement_reg(set_point_map.xn[set_point_map.xn.size() - 1], set_point_map.yn[set_point_map.yn.size() - 1], robotCoordMap, robot_motion_param,-1);
        }

        mutex mux;
        mux.lock();

        if(obstacle_detected){
            forwardspeed = 0;
            rotationspeed = 0;
        }
        else{
            forwardspeed = robot_motion_param.trans_speed;
            rotationspeed = robot_motion_param.rot_speed;
        }
        mux.unlock();

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
            safe_zone.length = std::sqrt(std::pow(set_point.xn[0],2) + std::pow(set_point.yn[0],2));
            safe_zone.width = 0.03;
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
    }
    else{
        if(set_point_map.xn.empty())
            return;

        set_point_map.xn.clear();
        set_point_map.yn.clear();
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

