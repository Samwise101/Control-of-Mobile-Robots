#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <QDialog>
#include <QPixmap>
#include <QPainter>
#include <QFile>

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

    void paintEvent(QPaintEvent *event);
};

#endif // PATHFINDING_H
