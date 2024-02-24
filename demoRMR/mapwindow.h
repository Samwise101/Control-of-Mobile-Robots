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

    void resizeToLeft(double newLength);
    void resizeToRight(double newLength);

    std::vector<std::vector<int>>& getMap();

    double getLength() const;
    void setLength(double newLength);

private:
    Ui::mapWindow *ui;
    std::vector<std::vector<int>> map;
    double length;

};

#endif // MAPWINDOW_H
