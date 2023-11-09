#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

#include "../include/Motor.hpp"
#include "../include/Task.hpp"
#include <atomic>

using namespace std;

int main()
{

    // Motor Declariration
    map<string, shared_ptr<TMotor>> tmotors;
    //tmotors["1_waist"] = make_shared<TMotor>(0x007, "AK10_9", "can0");
//
    tmotors["2_R_arm1"] = make_shared<TMotor>(0x001, "AK70_10", "can0");
    //tmotors["3_L_arm1"] = make_shared<TMotor>(0x002, "AKs70_10", "can0");
    //tmotors["4_R_arm2"] = make_shared<TMotor>(0x003, "AK70_10", "can0");

    //tmotors["5_R_arm3"] = make_shared<TMotor>(0x004, "AK70_10", "can0");
    //tmotors["6_L_arm2"] = make_shared<TMotor>(0x005, "AK70_10", "can0");
    //tmotors["7_L_arm3"] = make_shared<TMotor>(0x001, "AK70_10", "can0");

    map<string, shared_ptr<MaxonMotor>> maxonMotors;
    /*maxonMotors["a_maxon"] = make_shared<MaxonMotor>(0x001,
                                                          vector<uint32_t>{0x201, 0x301},
                                                          vector<uint32_t>{0x181},
                                                          "can0");
    maxonMotors["b_maxon"] = make_shared<MaxonMotor>(0x002,
                                                          vector<uint32_t>{0x202, 0x302},
                                                          vector<uint32_t>{0x182},
                                                          "can0");*/

    Task task(tmotors, maxonMotors);
    std::thread threadLoop(std::ref(task));
    threadLoop.join();

    return 0;
}
