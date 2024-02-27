#ifndef MAPTHREAD_H
#define MAPTHREAD_H

#include <QThread>
#include <QMutex>
#include "rplidar.h"

class mapThread : public QThread
{
    Q_OBJECT
private:

public:
    explicit mapThread(QObject *parent = nullptr);

    bool Stop;

    LaserMeasurement copyOfLaserData;

    void run();

signals:
    void setLaserData(LaserMeasurement&);

public slots:

};

#endif // MAPTHREAD_H
