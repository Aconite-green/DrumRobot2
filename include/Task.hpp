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
#include <cmath>

#define Pause 1
#define Terminate 2
#define Resume 0

queue<can_frame> sendBuffer;
queue<can_frame> recieveBuffer;
queue<int> sensorBuffer;
atomic<int> state(0);

using namespace std;

class Task
{

public:
    Task(map<string, shared_ptr<TMotor>> &tmotors,
         map<string, shared_ptr<MaxonMotor>> &maxonMotors,
         const map<string, int> &sockets);

    void operator()();

private:
    atomic<int> &state;
    map<string, shared_ptr<TMotor>> &tmotors;
    map<string, shared_ptr<MaxonMotor>> &maxonMotors;
    const map<string, int> &sockets;
    queue<can_frame> &sendBuffer;
    queue<can_frame> &recieveBuffer;
    queue<int> &sensorBuffer;

    TMotorCommandParser TParser;
    MaxonCommandParser MParser;

    std::mutex sendMutex;
    std::condition_variable sendCV;

    // DeactivateTask/ActivateTask
    void ActivateControlTask();
    void DeactivateControlTask();

    // Functions for Testing
    const int Tdegree_180 = M_PI;
    const int Tdegree_180 = M_PI / 2;
    const int Mdegree_180 = 2048 * 35;
    const int Mdegree_90 = 1024 * 35;

    void Tuning(float kp, float kd, float sine_t);
    void TuningLoopTask();
    void PeriodicMotionTester(queue<can_frame> &sendBuffer);

    // Functions for DrumRobot PathGenerating
    void PathLoopTask(queue<can_frame> &sendBuffer);

    // Functions for SendLoop
    template <typename MotorMap>
    void writeToSocket(MotorMap &motorMap, std::queue<can_frame> &sendBuffer,const std::map<std::string, int> &sockets);
    void SendLoopTask(queue<can_frame> &sendBuffer);

    // Functions for RecieveLoop
    const int NUM_FRAMES = 100;
    const int TIME_THRESHOLD_MS = 100;

    void initializeMotorCounts(std::map<std::string, int> &motor_count_per_port);
    void checkUserInput();
    void RecieveLoopTask(queue<can_frame> &recieveBuffer);
    void handleSocketRead(int socket_descriptor, int motor_count, queue<can_frame> &recieveBuffer);

    // Funtions for SensorLoop
    void SensorLoopTask(queue<int> &seonsorBuffer);
};