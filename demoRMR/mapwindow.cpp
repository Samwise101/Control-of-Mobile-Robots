#include "mapwindow.h"
#include "ui_mapwindow.h"
#include <QPainter>

mapWindow::mapWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::mapWindow)
{
    length = 1000.0;
    baseLength = length/2;
    map.resize(length);
    for(int i = 0; i < length; i++){
        map[i].resize(length,0);
    }
    ui->setupUi(this);
}

mapWindow::~mapWindow()
{
    delete ui;
}

void mapWindow::resizeToLeftBottom(double newLength)
{
    map.resize(newLength);
    for(int i = 0; i < newLength; i++){
        map[i].resize(newLength,0);
    }

    auto l = newLength - length;

    for(int i = length-1; i >= 0; i--){
        for(int j = 0; j < length; j++){
            map[i+l][j+l] = map[i][j];
            map[i][j] = 0;
        }
    }

    length = newLength;
    baseLength+=l;
}

void mapWindow::resizeToLeftTop(double newLength)
{
    map.resize(newLength);
    for(int i = 0; i < newLength; i++){
        map[i].resize(newLength,0);
    }

    auto l = newLength - length;

    for(int i = 0; i < length; i++){
        for(int j = length-1; j >= 0; j--){
            map[i][j+l] = map[i][j];
            map[i][j] = 0;
        }
    }

    length = newLength;
    baseLength+=l;
}

void mapWindow::resizeToRightBottom(double newLength)
{
    map.resize(newLength);
    for(int i = 0; i < newLength; i++){
        map[i].resize(newLength,0);
    }

    auto l = newLength - length;

    for(int i = length-1; i >=0; i--){
        for(int j = 0; j < length; j++){
            map[i+l][j] = map[i][j];
            map[i][j] = 0;
        }
    }

    length = newLength;
    baseLength+=l;
}

double &mapWindow::getBaseLength()
{
    return baseLength;
}

void mapWindow::writeToGrid(int& xgi, int& ygi)
{
    if(map.empty())
        return;

    map[xgi][ygi] = 1;
}

void mapWindow::resizeToRightTop(double newLength)
{
    map.resize(newLength);
    for(int i = 0; i < newLength; i++){
        map[i].resize(newLength,0);
    }
    auto l = newLength - length;
    length = newLength;
    baseLength+=l;
}

std::vector<std::vector<int>>& mapWindow::getMap()
{
    return map;
}

double mapWindow::getLength() const
{
    return length;
}

void mapWindow::setLength(double newLength)
{
    length = newLength;
}

void mapWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::white);
    QPen pen;
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(3);
    pen.setColor(Qt::black);

    QRect rect(10,10,length+10,length+10);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);

    for(int i = 0; i < map.size(); i++){
        for(int j = 0; j < map[i].size();j++){
            if(map[i][j]==1){
                int x =  i/2;
                int y =  j/2;
                painter.drawEllipse(QPoint(x,y),2,2);
            }
        }
    }
}
