#include "pathfinding.h"
#include "ui_pathfinding.h"
#include <QRect>

PathFinding::PathFinding(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PathFinding)
{
    ui->setupUi(this);
}

PathFinding::~PathFinding()
{
    delete ui;
}

void PathFinding::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    painter.setPen(QPen(Qt::white));

    QPixmap pixmap(":/images/map.png");
    int w = pixmap.width();
    int h = pixmap.height();

    QRect rect(50, 50, w, h);

    painter.drawRect(rect);

    painter.drawPixmap(rect, pixmap);
}
