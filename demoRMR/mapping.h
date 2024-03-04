#ifndef MAPPING_H
#define MAPPING_H

#include <QDialog>
#include <QPainter>
#include <memory>

namespace Ui {
class mapping;
}

class mapping : public QDialog
{
    Q_OBJECT

public:
    explicit mapping(QWidget *parent = nullptr);
    ~mapping();

    void resizeMapGrid(int newLength);

    int &getBaseLength();

    void writeToGrid(int xgi, int ygi);

    std::vector<std::vector<bool>>& getMap();

    int getLength() const;
    void setLength(int newLength);


    int getScale() const;

    QPixmap *getPix() const;

private:
    Ui::mapping *ui;

    std::vector<std::vector<bool>> map;
    int length;
    int baseLength;
    int scale;
    int h;
    int w;

    QPixmap* pix;

    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
};

#endif // MAPPING_H
