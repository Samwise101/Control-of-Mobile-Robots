#include "pathfinding.h"
#include "ui_pathfinding.h"
#include <QRect>
#include <iostream>
#include <queue>
#include <array>

const int dr[] = {-1, 0, 1, 0};
const int dc[] = {0, 1, 0, -1};

PathFinding::PathFinding(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PathFinding)
{
    canDraw = false;
    scale = 2;

    goal.setX(-1);
    goal.setY(-1);

    robotStart.setX(20);
    robotStart.setY(180);

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
                    paint.drawEllipse(QPoint(x,y),2,2);
                }
                else if(map[i][j]==-1){
                    paint.setPen(Qt::red);
                    paint.setBrush(Qt::red);
                    x = i*scale;
                    y = j*scale;
                    paint.drawEllipse(QPoint(x,y),2,2);
                }
                else if(map[i][j] != 0){
                    if(map[i][j] >= 255){
                        paint.setPen(QColor(0,255,map[i][j]-255));
                        paint.setBrush(QColor(0,255,map[i][j]-255));
                    }
                    else{
                        paint.setPen(QColor(0,map[i][j],0));
                        paint.setBrush(QColor(0,map[i][j],0));
                    }

                    x = i*scale;
                    y = j*scale;
                    paint.drawEllipse(QPoint(x,y),1,1);
                }
                if(!path_corner_points.empty()){
                    paint.setPen(Qt::magenta);
                    paint.setBrush(Qt::magenta);
                    for(int i=0; i < path_corner_points.size(); i++){
                        paint.drawEllipse(path_corner_points[i].x()*scale,path_corner_points[i].y()*scale,1,1);
                    }
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

            floodFill(goal.x(), goal.y(), robotStart.x(), robotStart.y(),2);
            findPathInGrid(robotStart.x(), robotStart.y(), goal.x(), goal.y());
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

void PathFinding::findHighestNumberInMap(const int& x, const int& y)
{
    std::cout << "targetX=" << x << ", targetY=" << y << std::endl;

    if(map[x][y] == -1){
        for (int i = 0; i < 4; ++i) {
            int nx = x + dr[i];
            int ny = y + dc[i];
            std::cout << "x=" << nx << ", y=" << ny << ", val=" << map[nx][ny] << std::endl;
        }
    }
}

void PathFinding::findPathInGrid(int startX, int startY, int targetX, int targetY)
{
    int x{startX};
    int y{startY};
    int tempVal{-1};
    int tempX{};
    int tempY{};

    for (int i = 0; i < 4; ++i) {
        int nx = x + dr[i];
        int ny = y + dc[i];

        if(map[nx][ny] > tempVal){
            tempVal = map[nx][ny];
        }
    }
    map[x][y] = tempVal+1;
    tempVal = -1;

    const int dr[] = {-1, 0, 1, 0};
    const int dc[] = {0, 1, 0, -1};

    //i=0 -> x-1, y
    //i=1 -> x, y+1
    //i=2 -> x+1, y
    //i=3 -> x, y-1
    while(map[x][y] != 3){
        if(map[x][y]-1 == map[x + dr[1]][y + dc[1]]){
            x = x + dr[1];
            y = y + dc[1];
            path_corner_points.push_back(QPoint(x,y));
        }
        else if(map[x][y]-1 == map[x + dr[3]][y + dc[3]]){
            x = x + dr[3];
            y = y + dc[3];
            path_corner_points.push_back(QPoint(x,y));
        }
        else if(map[x][y]-1 == map[x + dr[2]][y + dc[2]]){
            x = x + dr[2];
            y = y + dc[2];
            path_corner_points.push_back(QPoint(x,y));
        }
        else if(map[x][y]-1 == map[x + dr[0]][y + dc[0]]){
            x = x + dr[0];
            y = y + dc[0];
            path_corner_points.push_back(QPoint(x,y));
        }
    }
    map[startX][startY] = -1;
}



void PathFinding::floodFill(int x, int y, int targetX, int targetY, int currValue)
{
    int rows = map.size();
    int cols = map[0].size();

    if (x < 0 || x >= rows || y < 0 || y >= cols || map[x][y] == map[targetX][targetY])
        return;

    std::queue<QPoint> cells;
    cells.push({x,y});
    int k{};
    int newValue{};

    while (!cells.empty()) {
        int r = cells.front().x();
        int c = cells.front().y();
        cells.pop();

        if(map[r][c] == -2)
            newValue = 2;
        else
            newValue = map[r][c];

        if(r > targetX-2 && r < targetX+2 && c > targetY-2 && c < targetY+2){
            std::queue<QPoint>().swap(cells);
            std::cout << "Iam here" << std::endl;
            break;
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
    findHighestNumberInMap(targetX, targetY);
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
                        image.setPixel(i-2, j, Qt::gray);
                    }
                }
                if((j-2) > 0){
                    color = image.pixelColor(i,j-2);
                    if(color == Qt::white){
                        image.setPixel(i, j-2, Qt::gray);
                    }
                }
                if((i+2) < width){
                    color = image.pixelColor(i+2,j);
                    if(color == Qt::white){
                        image.setPixel(i+2, j, Qt::gray);
                    }
                }
                if((j+2) < height){
                    color = image.pixelColor(i,j+2);
                    if(color == Qt::white){
                        image.setPixel(i, j+2, Qt::gray);
                    }
                }
                if((j+2) < height && (i+2) < width){
                    color = image.pixelColor(i+2,j+2);
                    if(color == Qt::white){
                        image.setPixel(i+2, j+2, Qt::gray);
                    }
                }
                if((j-2) > 0 && (i-2) > 0){
                    color = image.pixelColor(i-2,j-2);
                    if(color == Qt::white){
                        image.setPixel(i-2, j-2, Qt::gray);
                    }
                }
                if((j+2) < height && (i-2) > 0){
                    color = image.pixelColor(i-2,j+2);
                    if(color == Qt::white){
                        image.setPixel(i-2, j+2, Qt::gray);
                    }
                }
                if((j-2) > 0 && (i+2) < width){
                    color = image.pixelColor(i+2,j-2);
                    if(color == Qt::white){
                        image.setPixel(i+2, j-2, Qt::gray);
                    }
                }
            }
        }
    }
    diffH = indexH - startH;
    diffW = indexW - startW;

    std::cout << "Height:" << diffH << ", Width:" << diffW << std::endl;
}
