#include "../include/ChartHandler.hpp"

void ChartHandler::displayChartSlot(){
   QDir directory("TuningData");
    QFileInfoList files = directory.entryInfoList(QStringList() << "*.csv", QDir::Files);

    if (files.isEmpty())
    {
        std::cerr << "No CSV files found in the TuningData directory." << std::endl;
        return;
    }

    QString firstCsvFile = files.first().absoluteFilePath();
    std::cout << "Loading CSV file: " << firstCsvFile.toStdString() << std::endl;

    std::ifstream file(firstCsvFile.toStdString());

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << firstCsvFile.toStdString() << std::endl;
        return;
    }

    QLineSeries *seriesPDes = new QLineSeries();
    QLineSeries *seriesPAct = new QLineSeries();
    QLineSeries *seriesTffDes = new QLineSeries();
    QLineSeries *seriesTffAct = new QLineSeries();

    std::string line;
    getline(file, line); // 헤더 줄 건너뛰기

    int time = 0;
    int lineCount = 0;
    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<float> values;

        while (getline(ss, value, ','))
        {
            values.push_back(std::stof(value));
        }

        if (values.size() != 5) {
            std::cerr << "Invalid line format: " << line << std::endl;
            continue;
        }

        seriesPDes->append(time, values[1]);
        seriesPAct->append(time, values[2]);
        seriesTffDes->append(time, values[3]);
        seriesTffAct->append(time, values[4]);
        time += 5; // 행 하나당 5ms 증가
        lineCount++;
    }

    std::cout << "Loaded " << lineCount << " data points from the file." << std::endl;

    // P_des와 P_act 차트
    QChart *chartP = new QChart();
    chartP->legend()->hide();
    chartP->addSeries(seriesPDes);
    chartP->addSeries(seriesPAct);
    chartP->createDefaultAxes();
    std::string titleP = "Position Chart: " + firstCsvFile.toStdString();
    chartP->setTitle(QString::fromStdString(titleP));

    // Tff_des와 Tff_act 차트
    QChart *chartTff = new QChart();
    chartTff->legend()->hide();
    chartTff->addSeries(seriesTffDes);
    chartTff->addSeries(seriesTffAct);
    chartTff->createDefaultAxes();
    std::string titleTff = "Torque Chart: " + firstCsvFile.toStdString();
    chartTff->setTitle(QString::fromStdString(titleTff));

    std::cout << "Charts created." << std::endl;

    // 차트 뷰 생성 및 설정
    QChartView *chartViewP = new QChartView(chartP);
    chartViewP->setRenderHint(QPainter::Antialiasing);

    QChartView *chartViewTff = new QChartView(chartTff);
    chartViewTff->setRenderHint(QPainter::Antialiasing);

    std::cout << "Chart views created." << std::endl;

    // 메인 윈도우 설정
    QMainWindow *window = new QMainWindow();
    QSplitter *splitter = new QSplitter();
    splitter->addWidget(chartViewP);
    splitter->addWidget(chartViewTff);

    window->setCentralWidget(splitter);
    window->resize(1200, 300); // 기존의 3배 크기
    window->show();

    std::cout << "Window displayed." << std::endl;
}