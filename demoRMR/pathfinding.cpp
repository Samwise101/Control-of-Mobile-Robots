#include "pathfinding.h"
#include "ui_pathfinding.h"
#include <QRect>
#include <iostream>

PathFinding::PathFinding(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PathFinding)
{
    canDraw = false;
    image = QImage(":/images/map.bmp");
    loadMapImage(image);

    QFile file("map3.bmp");
    QPixmap pixmap = QPixmap::fromImage(image);
    pixmap.save(&file, "BMP");

    loadMapToVector(image);
    ui->setupUi(this);

    h = ui->label->height();
    w = ui->label->width();
    std::cout << "height=" << h << ", width=" << w << std::endl;
    pix = new QPixmap(w,h);
    canDraw = true;
}

PathFinding::~PathFinding()
{
    delete pix;
    delete ui;
}

QPixmap PathFinding::getPixmap() const
{
    return pixmap;
}

void PathFinding::paintEvent(QPaintEvent *event)
{
    pixmap = QPixmap::fromImage(image);
    QPainter painter(&pixmap);
    painter.setPen(Qt::black);
    painter.setBrush(Qt::black);
    int wi = pixmap.width();
    int hi = pixmap.height();

    QRect rect(0, 0, wi, hi);
    painter.end();
    ui->label->setPixmap(pixmap);

//    if(canDraw){
//    QPainter paint(pix);
//    pix->fill( Qt::white);
//    paint.setPen(Qt::black);
//    paint.setBrush(Qt::black);

//    int x{};
//    int y{};

//    for(int i = 0; i < map.size(); i++){
//        for(int j = 0; j < map[i].size();j++){
//            if(map[i][j]==1){
//                x = i*3;
//                y = j*3;
//                paint.drawEllipse(QPoint(x,y),2,2);
//            }
//        }
//    }
//    paint.end();
//    ui->label->setPixmap(*pix);
//    }
}

void PathFinding::loadMapToVector(QImage& image)
{
    map.resize(diffW);
    for(int i = 0; i < diffW; i++){
        map[i].resize(diffH,0);
    }

    QColor color;

    for(int i = startW; i < indexW; i++){
        for(int j = startH; j < indexH; j++){
            color = image.pixelColor(i,j);
            if(color != Qt::white){
                map[i-startW][j-startH] = 1;
            }
            else{
                map[i-startW][j-startH] = 0;
            }
        }
    }
}

void PathFinding::loadMapImage(QImage& image)
{
    QColor color;

    int height = image.height();
    int width = image.width();

    indexH = 0;
    indexW = 0;
    startW = 1000;
    startH = 1000;

    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            color = image.pixelColor(i,j);
            if(color == Qt::black){
                if(j > indexH){
                    indexH = j;
                }
                if(i > indexW){
                    indexW = i;
                }
                if(j < startH){
                    startH = j;
                }
                if(i < startW){
                    startW = i;
                }
                if((i-2) > 0){
                    color = image.pixelColor(i-2,j);
                    if(color == Qt::white){
                        image.setPixel(i-2, j, qRgb(128, 128, 128));
                    }
                }

                if((j-2) > 0){
                    color = image.pixelColor(i,j-2);
                    if(color == Qt::white){
                        image.setPixel(i, j-2, qRgb(128, 128, 128));
                    }
                }

                if((i+2) < width){
                    color = image.pixelColor(i+2,j);
                    if(color == Qt::white){
                        image.setPixel(i+2, j, qRgb(128, 128, 128));
                    }
                }

                if((j+2) < height){
                    color = image.pixelColor(i,j+2);
                    if(color == Qt::white){
                        image.setPixel(i, j+2, qRgb(128, 128, 128));
                    }
                }
            }
        }
    }

    diffH = indexH - startH;
    diffW = indexW - startW;
    std::cout << "Height:" << diffH << ", Width:" << diffW << std::endl;
}
