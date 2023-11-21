#include "../include/ChartHandler.hpp"

void ChartHandler::displayChartSlot(){
    QLineSeries *series = new QLineSeries();
        series->append(0, 6);
        series->append(2, 4);
        series->append(3, 8);
        series->append(7, 4);
        series->append(10, 5);

        QChart *chart = new QChart();
        chart->legend()->hide();
        chart->addSeries(series);
        chart->createDefaultAxes();
        chart->setTitle("Simple Line Chart Example");

        QChartView *chartView = new QChartView(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartView->resize(400, 300);
        chartView->show();
}