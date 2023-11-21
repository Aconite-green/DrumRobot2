// ChartHandler.hpp
#ifndef CHARTHANDLER_HPP
#define CHARTHANDLER_HPP

#include <QObject>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QApplication>

using namespace QtCharts;

class ChartHandler : public QObject
{
    Q_OBJECT

public:
    ChartHandler(QObject *parent = nullptr) : QObject(parent) {}

signals:
    void displayChartSignal();

public slots:
    void displayChartSlot();
};
#endif // CHARTHANDLER_HPP