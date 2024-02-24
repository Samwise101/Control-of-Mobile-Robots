#include "mapwindow.h"
#include "ui_mapwindow.h"
#include <QPainter>

mapWindow::mapWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::mapWindow)
{
    length = 20.0;
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

void mapWindow::resizeToLeft(double newLength)
{
    map.resize(newLength);
    for(int i = 0; i < newLength; i++){
        map[i].resize(newLength,0);
    }

    auto l = newLength - length;

    for(int i = length-1; i >= 0; i--){
        for(int j = 0; j < length; j++){
            map[i+l][j] = map[i][j];
            map[i][j] = 0;
        }
    }

    length = newLength;
}

void mapWindow::resizeToRight(double newLength)
{
    map.resize(newLength);
    for(int i = 0; i < newLength; i++){
        map[i].resize(newLength,0);
    }

    length = newLength;
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

}
