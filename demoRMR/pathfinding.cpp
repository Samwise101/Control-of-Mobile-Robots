#include "pathfinding.h"
#include "ui_pathfinding.h"
#include <QRect>
#include <iostream>
#include <queue>
#include <stack>
#include <utility>

PathFinding::PathFinding(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PathFinding)
{
    canDraw = false;
    scale = 3;

    goal.setX(-1);
    goal.setY(-1);

    robotStart.setX(20);
    robotStart.setY(140);

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
//    pixmap = QPixmap::fromImage(image);
//    QPainter painter(&pixmap);
//    painter.setPen(Qt::black);
//    painter.setBrush(Qt::black);
//    int wi = pixmap.width();
//    int hi = pixmap.height();

//    QRect rect(0, 0, wi, hi);
//    painter.end();
//    ui->label->setPixmap(pixmap);

    if(canDraw){
        QPainter paint(pix);
        pix->fill( Qt::white);
        paint.setPen(Qt::black);
        paint.setBrush(Qt::black);

        int x{};
        int y{};

        for(int i = 0; i < map.size(); i++){
            for(int j = 0; j < map[i].size();j++){
                paint.setPen(Qt::black);
                paint.setBrush(Qt::black);
                if(map[i][j]==1){
                    x = i*scale;
                    y = j*scale;
                    paint.drawEllipse(QPoint(x,y),2,2);
                }
                else if(map[i][j]==-2){
                    paint.setPen(Qt::blue);
                    paint.setBrush(Qt::blue);
                    x = i*scale;
                    y = j*scale;
                    paint.drawEllipse(QPoint(x,y),4,4);
                }
                else if(map[i][j]==-1){
                    paint.setPen(Qt::red);
                    paint.setBrush(Qt::red);
                    x = i*scale;
                    y = j*scale;
                    paint.drawEllipse(QPoint(x,y),4,4);
                }
                else if(map[i][j] != 0){
                    paint.setPen(QColor(0,map[i][j],0));
                    paint.setBrush(QColor(0,map[i][j],0));
                    x = i*scale;
                    y = j*scale;
                    paint.drawEllipse(QPoint(x,y),1,1);
                }
            }
        }

        paint.end();
        ui->label->setPixmap(*pix);
    }
}

void PathFinding::mousePressEvent(QMouseEvent *event)
{
    //std::cout << "X=" << event->x() << ", Y=" << event->y() << std::endl;
    if (event->button() == Qt::LeftButton) {
        if(event->x() < 0 || event->x() > diffW*scale)
            return;
        if(event->y() < 0 || event->y() > diffH*scale)
            return;

        std::cout << "X=" << event->x() << ", Y=" << event->y() << std::endl;

        QImage imageTemp = pix->toImage();
        if(imageTemp.pixelColor(event->x(), event->y()) == Qt::white){
            if(goal.x() != -1 && goal.y() != -1){
                map[goal.x()][goal.y()] = 0;
            }
            goal.setX(event->x()/scale);
            goal.setY(event->y()/scale);
            map[event->x()/scale][event->y()/scale]=-2;
            update();

            floodFill(goal.x(), goal.y(), robotStart.x(), robotStart.y(),2);
        }
    }
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
            if(i == (startW + robotStart.x()) && j == (startH + robotStart.y())){
                map[i-startW][j-startH] = -1;
            }
        }
    }
}



void PathFinding::floodFill(int x, int y, int targetX, int targetY, int currValue) {
    int rows = map.size();
    int cols = map[0].size();
    const int dr[] = {-1, 0, 1, 0};
    const int dc[] = {0, 1, 0, -1};

    if (x < 0 || x >= rows || y < 0 || y >= cols || map[x][y] == map[targetX][targetY])
        return;

    std::queue<std::pair<int, int>> cells;
    cells.push({x,y});
    int k = 0;
    int newValue{};

    while (!cells.empty()) {
        int r = cells.front().first;
        int c = cells.front().second;
        cells.pop();

        if(map[r][c] == -2)
            newValue = 2;
        else
            newValue = map[r][c];

        if(r == targetX && c == targetY){
            std::cout << "Iam here" << std::endl;
            return;
        }

        for (int d = 0; d < 4; ++d) {
            int nx = r + dr[d];
            int ny = c + dc[d];
            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && map[nx][ny] == 0) {
                cells.push({nx, ny});
                k++;
                if(map[nx][ny]!=-2 && map[nx][ny]!=-1){
                    map[nx][ny] = newValue + 1;
                }
            }
        }
    }
    std::cout << "k=" << k << std::endl;
}

void PathFinding::loadMapImage(QImage& image)
{
    QColor color;

    int height{image.height()};
    int width{image.width()};

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
