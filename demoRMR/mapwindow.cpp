#include "mapwindow.h"
#include "ui_mapwindow.h"
#include <QPainter>

mapWindow::mapWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::mapWindow)
{
    length = 500.0;
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

void mapWindow::resizeMapGrid(double newLength)
{
    map.resize(newLength);
    for(int i = 0; i < newLength; i++){
        map[i].resize(newLength,0);
    }

    auto l = (newLength - length)/2;

    for(int i = length-1; i >= 0; i--){
        for(int j = 0; j < length; j++){
            map[i+l][j+l] = map[i][j];
            map[i][j] = 0;
        }
    }

    length = newLength;
    baseLength = length/2;
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
    pen.setColor(Qt::white);

    QRect rect(10,10,length+10,length+10);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);

    painter.setBrush(Qt::black);
    pen.setColor(Qt::black);

    for(int i = 0; i < map.size(); i++){
        for(int j = 0; j < map[i].size();j++){
            if(map[i][j]==1){
                int x =  100+i/2;
                int y = 100+j/2;
                painter.drawEllipse(QPoint(x,y),2,2);
            }
        }
    }
}
