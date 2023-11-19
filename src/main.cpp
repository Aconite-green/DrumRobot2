#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

#include "../include/Motor.hpp"
#include "../include/Task.hpp"
#include "../include/TaskUtility.hpp"
#include <atomic>
#include <QApplication>
#include <QtCharts/QLineSeries>

using namespace std;
using namespace QtCharts;

int main(int argc, char *argv[])
{
    /*
sudo apt-get update
sudo apt-get install qt5-default
sudo apt-get install libqt5charts5 libqt5charts5-dev

    */

    // Motor Declariration
    std::map<std::string, std::shared_ptr<TMotor>, CustomCompare> tmotors;
    // tmotors["waist"] = make_shared<TMotor>(0x007, "AK10_9", "can0");

    // tmotors["R_arm1"] = make_shared<TMotor>(0x001, "AK70_10", "can0");
    // tmotors["L_arm1"] = make_shared<TMotor>(0x002, "AK70_10", "can0");
    // tmotors["R_arm2"] = make_shared<TMotor>(0x003, "AK70_10", "can0");

    // tmotors["R_arm3"] = make_shared<TMotor>(0x004, "AK70_10", "can0");
    // tmotors["L_arm2"] = make_shared<TMotor>(0x005, "AK70_10", "can0");
    // tmotors["L_arm3"] = make_shared<TMotor>(0x006, "AK70_10", "can0");

    map<string, shared_ptr<MaxonMotor>> maxonMotors;
    /*maxonMotors["a_maxon"] = make_shared<MaxonMotor>(0x001,
                                                          vector<uint32_t>{0x201, 0x301},
                                                          vector<uint32_t>{0x181},
                                                          "can0");
    maxonMotors["b_maxon"] = make_shared<MaxonMotor>(0x002,
                                                          vector<uint32_t>{0x202, 0x302},
                                                          vector<uint32_t>{0x182},
                                                          "can0");*/

    QApplication app(argc, argv);

    Task task(tmotors, maxonMotors);
    std::thread threadLoop(std::ref(task));
    int ret = app.exec();
    threadLoop.join();

    return ret;
}
