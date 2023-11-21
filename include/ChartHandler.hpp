// ChartHandler.hpp
#ifndef CHARTHANDLER_HPP
#define CHARTHANDLER_HPP

#include <QObject>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QApplication>
#include <QDir>
#include <QFileInfoList>
#include <QSplitter>
#include <QtWidgets/QMainWindow>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace QtCharts;
using namespace std;

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