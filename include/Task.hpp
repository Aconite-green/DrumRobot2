#include <stdio.h>
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/ErrorHandle.hpp"
#include "../include/Motor.hpp"
#include "../include/TaskUtility.hpp"
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <queue>
#include <algorithm>
#include <thread>
#include <cerrno>  // errno
#include <cstring> // strerror
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <mutex>
#include <atomic>
#include <condition_variable>

using namespace std;

class Task
{

public:
    Task(map<string, shared_ptr<TMotor>> &tmotors,
         map<string, shared_ptr<MaxonMotor>> &maxonMotors,
         atomic<int> &state,
         map<string, int> &sockets,
         queue<can_frame> &sendBuffer,
         queue<can_frame> &recieveBuffer,
         queue<int> &sensorBuffer);

    void operator()();

private:
    atomic<int> &state;
    map<string, shared_ptr<TMotor>> &tmotors;
    map<string, shared_ptr<MaxonMotor>> &maxonMotors;
    map<string, int> &sockets;
    queue<can_frame> &sendBuffer;
    queue<can_frame> &recieveBuffer;
    queue<int> &sensorBuffer;

    mutex mtx;
    condition_variable cv; 

    TMotorCommandParser TParser;
    MaxonCommandParser MParser;

    // DeactivateTask/ActivateTask
    void ActivateControlTask();
    void DeactivateControlTask();

    // Functions for Testing
    void Tuning(float kp, float kd, float sine_t);
    void TuningLoopTask();

    // Functions for SendLoop
    void SendLoopTask(queue<can_frame> &sendBuffer);

    // Functions for PathGenerating
    void PathLoopTask(queue<can_frame> &sendBuffer);

    // Functions for RecieveLoop
    void RecieveLoopTask(queue<can_frame> &recieveBuffer);

    // Funtions for SensorLoop
    void SensorLoopTask(queue<int> &seonsorBuffer);
};