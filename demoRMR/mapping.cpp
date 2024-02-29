#include "mapping.h"
#include "ui_mapping.h"

mapping::mapping(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::mapping)
{
    length = 1000;
    baseLength = length/2;
    map.resize(length);
    for(int i = 0; i < length; i++){
        map[i].resize(length,0);
    }
    ui->setupUi(this);
}

mapping::~mapping()
{
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

void mapping::writeToGrid(int& xgi, int& ygi)
{
    if(map.empty())
        return;

    map[xgi][ygi] = 1;
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

void mapping::paintEvent(QPaintEvent *event)
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
