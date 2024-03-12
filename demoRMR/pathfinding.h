#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <QDialog>
#include <QPixmap>
#include <QPainter>
#include <QFile>
#include <QImage>
#include <vector>
#include <QMouseEvent>
#include <QEvent>
#include <QPoint>

namespace Ui {
class PathFinding;
}

class PathFinding : public QDialog
{
    Q_OBJECT

public:
    explicit PathFinding(QWidget *parent = nullptr);
    ~PathFinding();

    QPixmap getPixmap() const;

private:
    Ui::PathFinding *ui;

    QImage image;
    QPixmap pixmap;

    QPoint robotStart;
    QPoint goal;
    int indexH;
    int indexW;
    int startH;
    int startW;
    int diffH;
    int diffW;
    int scale;

    std::vector<std::vector<int>> map;
    std::vector<QPoint> path_corner_points;

    bool canDraw;

    QPixmap* pix;

    int h;
    int w;

    void floodFill(int startRow, int startCol, int targetRow, int targetCol, int currValue);

    void paintEvent(QPaintEvent *event);

    void mousePressEvent(QMouseEvent *event);

    void loadMapImage(QImage& image);

    void loadMapToVector(QImage& image);

    void findHighestNumberInMap(const int& x, const int& y);

    void findPathInGrid(int startX, int startY, int targetX, int targetY);
};

#endif // PATHFINDING_H
