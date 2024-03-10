#include "pathfinding.h"
#include "ui_pathfinding.h"
#include <QRect>

PathFinding::PathFinding(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PathFinding)
{
    canDraw = false;
    image = QImage(":/images/map.png");
    loadMap(image);
    ui->setupUi(this);
}

PathFinding::~PathFinding()
{
    delete ui;
}

void PathFinding::paintEvent(QPaintEvent *event)
{
    pixmap = QPixmap::fromImage(image);
    QPainter painter(&pixmap);
    painter.setPen(Qt::black);
    painter.setBrush(Qt::black);
    int w = pixmap.width();
    int h = pixmap.height();

    if(canDraw){
        painter.setPen(Qt::blue);
        painter.setBrush(Qt::blue);
        painter.drawEllipse(indexW-1, indexH-1, 4,4);
    }

    QRect rect(0, 0, w, h);
    painter.end();
    ui->label->setPixmap(pixmap);
}

void PathFinding::loadMap(QImage& image)
{
    QColor color;

    int height = image.height();
    int width = image.width();

    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            color = image.pixelColor(i,j);
            if(color == Qt::black){
                if((i-4) > 0){
                    color = image.pixelColor(i-4,j);
                    if(color == Qt::white){
                        image.setPixel(i-4, j, qRgb(128, 128, 128));
                    }
                }

                if((j-4) > 0){
                    color = image.pixelColor(i,j-4);
                    if(color == Qt::white){
                        image.setPixel(i, j-4, qRgb(128, 128, 128));
                    }
                }

                if((i+4) < width){
                    color = image.pixelColor(i+4,j);
                    if(color == Qt::white){
                        image.setPixel(i+4, j, qRgb(128, 128, 128));
                    }
                }

                if((j+4) < height){
                    color = image.pixelColor(i,j+4);
                    if(color == Qt::white){
                        image.setPixel(i, j+4, qRgb(128, 128, 128));
                    }
                }
            }
        }
    }
    canDraw = true;
}
