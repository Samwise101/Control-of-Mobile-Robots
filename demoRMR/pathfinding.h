#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <QDialog>
#include <QPixmap>
#include <QPainter>
#include <QFile>
#include <QImage>
#include <vector>

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

    int indexH;
    int indexW;
    int startH;
    int startW;
    int diffH;
    int diffW;

    std::vector<std::vector<int>> map;

    bool canDraw;

    QPixmap* pix;

    int h;
    int w;

    void paintEvent(QPaintEvent *event);

    void loadMapImage(QImage& image);

    void loadMapToVector(QImage& image);
};

#endif // PATHFINDING_H
