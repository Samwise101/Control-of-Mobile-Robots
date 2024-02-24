#ifndef MAPWINDOW_H
#define MAPWINDOW_H

#include <QDialog>
#include <vector>

namespace Ui {
class mapWindow;
}

class mapWindow : public QDialog
{
    Q_OBJECT

public:
    explicit mapWindow(QWidget *parent = nullptr);
    ~mapWindow();

    void resizeToLeftTop(double newLength);
    void resizeToRightTop(double newLength);
    void resizeToLeftBottom(double newLength);
    void resizeToRightBottom(double newLength);

    std::vector<std::vector<int>>& getMap();

    double getLength() const;
    void setLength(double newLength);

private:
    Ui::mapWindow *ui;
    std::vector<std::vector<int>> map;
    double length;

    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;

};

#endif // MAPWINDOW_H
