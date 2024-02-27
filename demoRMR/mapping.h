#ifndef MAPPING_H
#define MAPPING_H

#include <QDialog>
#include <QPainter>

namespace Ui {
class mapping;
}

class mapping : public QDialog
{
    Q_OBJECT

public:
    explicit mapping(QWidget *parent = nullptr);
    ~mapping();

    void resizeMapGrid(double newLength);

    double &getBaseLength();

    void writeToGrid(int& xgi, int& ygi);

    std::vector<std::vector<bool>>& getMap();

    double getLength() const;
    void setLength(double newLength);


private:
    Ui::mapping *ui;

    std::vector<std::vector<bool>> map;
    double length;
    double baseLength;

    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
};

#endif // MAPPING_H
