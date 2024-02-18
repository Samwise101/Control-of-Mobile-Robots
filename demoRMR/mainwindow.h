#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include<windows.h>
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "robot.h"
#include <memory.h>
#include <QMessageBox>
#include <QJoysticks.h>

typedef struct robot_connect_data{
    int robot_port;
    int robot_port_me;
    int lidar_port;
    int lidar_port_me;
    std::string robot_ip;
    std::string camera_port;
}robot_connect_data;

typedef struct SetPoint{
    std::vector<double> xn;
    std::vector<double> yn;
}SetPoint;

bool isValidIpAddress(const std::string &ipAddress);

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    cv::Mat frame[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

    void set_robot_connect_data();

int processThisCamera(cv::Mat cameraData);

private slots:
    void on_startButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();
    void getNewFrame();

    void on_robotIP_returnPressed();

    void on_startMissionButton_clicked();

    void on_pushButton_9_clicked();

private:
     JOYINFO joystickInfo;
     Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     bool isTest;
     TKobukiData robotdata;
     robot_connect_data robot_data;
     int datacounter;
     QTimer *timer;

     bool mission_started;

     SetPoint set_point;

     QJoysticks *instance;

     RobotCoordRotation robotCoord;

     double forwardspeed;//mm/s
     double rotationspeed;//omega/s
public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
};


#endif // MAINWINDOW_H
