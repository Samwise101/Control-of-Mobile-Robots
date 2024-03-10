#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <regex>
#include <thread>
#include <chrono>


void MainWindow::get_laserdata_and_write_to_map(double robotX, double robotY, double robotA)
{
    //std::cout << "X=" << robotX << ", Y=" << robotY << ", A=" << robotA << std::endl;
    double robotX2 = robotX;
    double robotY2 = -robotY;
    double robotAngle = robotA;

    mux2.lock();
    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k+=2)
    {
        double lidarDist=copyOfLaserData.Data[k].scanDistance/100;

        //std::cout << "Lidar dist = " << lidarDist << std::endl;

        if(lidarDist > 200 || lidarDist < 5)
            continue;

        double xp = (robotX2*10 + lidarDist*cos((-(copyOfLaserData.Data[k].scanAngle))*TO_RADIANS+robotAngle));
        double yp = (robotY2*10 - lidarDist*sin((-(copyOfLaserData.Data[k].scanAngle))*TO_RADIANS+robotAngle));

        int bl = mapDialog.getBaseLength();

        xp += bl;
        yp += bl;
        int l = mapDialog.getLength();

        if(xp < 0 || xp >= l || yp < 0 || yp >= l)
            mapDialog.resizeMapGrid(l+bl);
        else
            mapDialog.writeToGrid(std::round(xp),std::round(yp));
    }
    mux2.unlock();
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

    datacounter=0;
    canStart = false;
    isRotating = false;
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
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
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

            mux2.lock();
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int lidarDist=copyOfLaserData.Data[k].scanDistance/20;
                int xp=rect.width()-(rect.width()/2+lidarDist*2*sin((360.0+copyOfLaserData.Data[k].scanAngle)*TO_RADIANS))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+lidarDist*2*cos((360.0+copyOfLaserData.Data[k].scanAngle)*TO_RADIANS))+rect.topLeft().y();

                if(rect.contains(xp,yp))
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
            mux2.unlock();
        }
    }
}

void MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
    ui->lineEdit_2->setText(QString::number(robotX,10,6));
    ui->lineEdit_3->setText(QString::number(robotY,10,6));
    ui->lineEdit_4->setText(QString::number(robotFi));
}


int MainWindow::processThisRobot(TKobukiData robotdata)
{
    robot_odometry.robot_odometry(robotdata, true, robotCoord);

    if(mission_started && !set_point.xn.empty()){

        robot_motion_reg.robot_movement_reg(set_point.xn[set_point.xn.size() - 1], set_point.yn[set_point.yn.size() - 1], robotCoord, robot_motion_param);

        forwardspeed = robot_motion_param.trans_speed;
        rotationspeed = robot_motion_param.rot_speed;

        mutex mux;
        if(set_point.xn.empty()){
            mission_started = false;
            ui->startMissionButton->setText("Start mission");
        }

        if(forwardspeed == 0.0 && rotationspeed == 0.0)
        {
            set_point.xn.pop_back();
            set_point.yn.pop_back();
        }

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
        else if(forwardspeed != 0.0 && rotationspeed == 0.0)
            robot.setArcSpeed(forwardspeed, 0);
    }
    else{
        forwardspeed = 0;
        robot.setArcSpeed(forwardspeed, 0);
    }

    if(datacounter%5)
    {
        emit uiValuesChanged(robotCoord.x, robotCoord.y, robotCoord.a);
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
/*    mutex mux;
    mux.lock();
    robotX = robotCoord.x;
    robotY = robotCoord.y;
    robotAngle = robotCoord.a*TO_RADIANS;
    if(!isRotating){
        f = std::bind(&MainWindow::get_laserdata_and_write_to_map,this,robotX, robotY, robotAngle);
        std::async(std::launch::async, f);
    }
    mux.unlock()*/;
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
    /*std::vector<unsigned char> mess=robot.setTranslationSpeed(500);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);
  /*  std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_6_clicked() //left
{
    robot.setRotationSpeed(3.14159/2);
  /*  std::vector<unsigned char> mess=robot.setRotationSpeed(3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_5_clicked()//right
{
    robot.setRotationSpeed(-3.14159/2);
   /* std::vector<unsigned char> mess=robot.setRotationSpeed(-3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    forwardspeed = 0.0;
    rotationspeed = 0.0;
  /*  std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
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
    if(!mission_started && !set_point.xn.empty()){
        mission_started = true;
        ui->startMissionButton->setText("Stop mission");
    }
    else if(mission_started){
        mission_started = false;
        ui->startMissionButton->setText("Start mission");
    }
}


void MainWindow::on_pushButton_9_clicked()
{
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


void MainWindow::on_pushButton_10_clicked()
{
    pathFindDialog.exec();
    QFile file("map2.bmp");
    pathFindDialog.getPixmap().save(&file, "BMP");
}


void MainWindow::on_pushButton_11_clicked()
{
    if(set_point.xn.empty())
        return;

    set_point.xn.clear();
    set_point.yn.clear();

    mission_started = false;
    ui->startMissionButton->setText("Start mission");
}

