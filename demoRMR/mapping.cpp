#include "mapping.h"
#include "ui_mapping.h"
#include <QGraphicsView>
#include <iostream>

mapping::mapping(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::mapping)
{
    length = 140;
    scale = 2;
    baseLength = length/2;
    map.resize(length);

    h = 0;
    w = 0;

    for(int i = 0; i < length; i++){
        map[i].resize(length,0);
    }
    ui->setupUi(this);

    h = ui->label->height();
    w = ui->label->width();
    std::cout << "height=" << h << ", width=" << w << std::endl;
    pix = new QPixmap(w,h);
}

mapping::~mapping()
{
    delete pix;
    delete ui;
}


void mapping::resizeMapGrid(int newLength)
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



int &mapping::getBaseLength()
{
    return baseLength;
}

void mapping::writeToGrid(double xgi, double ygi)
{
    map[(int)xgi][(int)ygi] = true;
}


std::vector<std::vector<bool>>& mapping::getMap()
{
    return map;
}

int mapping::getLength() const
{
    return length;
}

void mapping::setLength(int newLength)
{
    length = newLength;
}

int mapping::getScale() const
{
    return scale;
}

QPixmap *mapping::getPix() const
{
    return pix;
}

void mapping::paintEvent(QPaintEvent *event)
{
    QPainter paint(pix);
    pix->fill(Qt::white);
    paint.setPen(Qt::black);
    paint.setBrush(Qt::black);

    int x{};
    int y{};

    for(int i = 0; i < map.size(); i++){
        for(int j = 0; j < map[i].size();j++){
            if(map[i][j]==1){
                x = i*scale-120;
                y = j*scale-40;
                paint.drawEllipse(QPoint(x,y),2,2);
            }
        }
    }

    paint.end();
    ui->label->setPixmap(*pix);
}
