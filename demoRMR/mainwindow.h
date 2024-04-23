#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QPixmap>
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
#include "CKobuki.h"
#include "robot.h"
#include "odometry.h"
#include "regulator.h"
#include "mapping.h"
#include "pathfinding.h"

#include <memory.h>
#include <QMessageBox>
#include <QJoysticks.h>
#include <QFile>
#include <QDebug>

#include <thread>
#include <mutex>
#include <future>

typedef struct robot_connect_data{
    int robot_port;
    int robot_port_me;
    int lidar_port;
    int lidar_port_me;
    std::string robot_ip;
    std::string camera_port;
    std::string camera_link;
}robot_connect_data;


typedef struct Point{
    double x;
    double y;
}Point;

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

    void get_laserdata_and_write_to_map(double robotX, double robotY, double robotA, double setX, double setY);

    bool checkAccessibility(double xp, double yp);

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

    void on_startMissionButton_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_11_clicked();

    void on_comboBox_2_activated(int index);

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
     double robotX;
     double robotY;
     double robotAngle;


     bool obstacle_detected;

     int controlType;

     std::function<void()> f;

     bool isRotating;
     bool robotStop;

     std::atomic_bool isStoped;
     std::atomic_bool canStart;

     bool mission_started;

     SetPoint set_point;

     SetPoint set_point_map;

     std::mutex mux;
     std::mutex mux2;

     QJoysticks *instance;

     RobotCoordRotation robotCoord;
     RobotCoordRotation robotCoordMap;
     robot_motion robot_motion_param{0.0, 0.65, 0.0, 0.4, 0.1};

     odometry robot_odometry;
     Regulator robot_motion_reg;
     mapping mapDialog;

     std::thread mappingThread;

     double forwardspeed;//mm/s
     double rotationspeed;//omega/s
public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
    void setUiValuesForMap(double setPointX, double setPointY);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
     void uiValuesChangedMap(double setPointX, double setPointY);
};


#endif // MAINWINDOW_H
