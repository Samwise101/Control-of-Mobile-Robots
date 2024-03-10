#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <QDialog>
#include <QPixmap>
#include <QPainter>
#include <QFile>
#include <QImage>

namespace Ui {
class PathFinding;
}

class PathFinding : public QDialog
{
    Q_OBJECT

public:
    explicit PathFinding(QWidget *parent = nullptr);
    ~PathFinding();

private:
    Ui::PathFinding *ui;

    QImage image;
    QPixmap pixmap;

    void paintEvent(QPaintEvent *event);

    void loadMap(QImage& image);
};

#endif // PATHFINDING_H
