#include "../include/ChartHandler.hpp"

void ChartHandler::displayChartSlot()
{
    QDir directory("TuningData");
    QFileInfoList files = directory.entryInfoList(QStringList() << "*.csv", QDir::Files);

    if (files.isEmpty())
    {
        std::cerr << "No CSV files found in the TuningData directory." << std::endl;
        return;
    }

    QString firstCsvFile = files.first().absoluteFilePath();
    std::string fileName = files.first().fileName().toStdString();
    QString qFileName = QString::fromStdString(fileName);
    std::cout << "Loading CSV file: " << fileName << std::endl;

    // Extract Kp and Kd values from the filename
    std::regex kpKdRegex("kp_([0-9.]+)_kd_([0-9.]+)");
    std::smatch kpKdMatch;
    QString chartTitle;
    if (std::regex_search(fileName, kpKdMatch, kpKdRegex) && kpKdMatch.size() > 2)
    {
        chartTitle = "Kp : " + QString::fromStdString(kpKdMatch.str(1)) +
                     " Kd : " + QString::fromStdString(kpKdMatch.str(2));
    }
    else
    {
        chartTitle = "Chart";
    }

    std::ifstream file(firstCsvFile.toStdString());

    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return;
    }

    QLineSeries *seriesPDes = new QLineSeries();
    seriesPDes->setName("P_des");
    QLineSeries *seriesPAct = new QLineSeries();
    seriesPAct->setName("P_act");
    QLineSeries *seriesTffDes = new QLineSeries();
    seriesTffDes->setName("Tff_des");
    QLineSeries *seriesTffAct = new QLineSeries();
    seriesTffAct->setName("Tff_act");

    std::string line;
    getline(file, line); // Skip header line

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

        if (values.size() != 5)
        {
            std::cerr << "Invalid line format: " << line << std::endl;
            continue;
        }

        seriesPDes->append(time, values[1]);
        seriesPAct->append(time, values[2]);
        seriesTffDes->append(time, values[3]);
        seriesTffAct->append(time, values[4]);
        time += 5; // Increment time by 5ms for each line
        lineCount++;
    }

    std::cout << "Loaded " << lineCount << " data points from the file." << std::endl;

    // Create new axes and set their properties
    QValueAxis *axisX = new QValueAxis();
    axisX->setTitleText("Time [ms]");
    QValueAxis *axisYP = new QValueAxis();
    axisYP->setTitleText("Position [rad]");
    axisYP->setLabelFormat("%0.2f");
    axisYP->setRange(-2.0, 2.0); // Adjust the range as needed
    QValueAxis *axisYT = new QValueAxis();
    axisYT->setTitleText("Torque [N/m]");
    axisYT->setLabelFormat("%0.2f");
    axisYT->setRange(-5.0, 5.0); // Adjust the range as needed

    // Configure the Position chart
    QChart *chartP = new QChart();
    chartP->setTitle(chartTitle);
    chartP->addAxis(axisX, Qt::AlignBottom);
    chartP->addAxis(axisYP, Qt::AlignLeft);
    chartP->addSeries(seriesPDes);
    chartP->addSeries(seriesPAct);
    seriesPDes->attachAxis(axisX);
    seriesPDes->attachAxis(axisYP);
    seriesPAct->attachAxis(axisX);
    seriesPAct->attachAxis(axisYP);

    // Configure the Torque chart
    QChart *chartTff = new QChart();
    chartTff->setTitle(chartTitle);
    chartTff->addAxis(axisX, Qt::AlignBottom);
    chartTff->addAxis(axisYT, Qt::AlignLeft);
    chartTff->addSeries(seriesTffDes);
    chartTff->addSeries(seriesTffAct);
    seriesTffDes->attachAxis(axisX);
    seriesTffDes->attachAxis(axisYT);
    seriesTffAct->attachAxis(axisX);
    seriesTffAct->attachAxis(axisYT);

    // Create chart views and configure them
    QChartView *chartViewP = new QChartView(chartP);
    chartViewP->setRenderHint(QPainter::Antialiasing);
    QChartView *chartViewTff = new QChartView(chartTff);
    chartViewTff->setRenderHint(QPainter::Antialiasing);

    // Setup the main window
    QMainWindow *window = new QMainWindow();
    QSplitter *splitter = new QSplitter();
    splitter->addWidget(chartViewP);
    splitter->addWidget(chartViewTff);
    window->setCentralWidget(splitter);
    window->resize(1200, 700);
    window->show();
}
