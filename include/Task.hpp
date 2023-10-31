#include <stdio.h>
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/ErrorHandle.hpp"
#include "../include/Motor.hpp"
#include "../include/TaskUtility.hpp"
#include "../include/Global.hpp"
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
#include <vector>
#include <limits>
#include <time.h>
#include <fstream>
#include <atomic>
#include <cmath>

#define Pause 1
#define Terminate 2
#define Resume 0

using namespace std;

class Task
{

public:
   // 생성자 매개변수 이름 변경 및 명확성 추가
    Task(map<string, shared_ptr<TMotor>> &input_tmotors,
         map<string, shared_ptr<MaxonMotor>> &input_maxonMotors,
         const map<string, int> &input_sockets);

    void operator()();

private:
    map<string, shared_ptr<TMotor>> &tmotors;
    map<string, shared_ptr<MaxonMotor>> &maxonMotors;
    const map<string, int> &sockets;
    queue<can_frame> sendBuffer;
    queue<can_frame> recieveBuffer;
    queue<int> sensorBuffer;
    atomic<int> state; // 초기값은 생성자에서 설정

    TMotorCommandParser TParser;
    MaxonCommandParser MParser;

    // DeactivateTask/ActivateTask
    void ActivateControlTask();
    void DeactivateControlTask();

    // Functions for Testing
    const int Tdegree_180 = M_PI;
    const int Tdegree_90 = M_PI / 2;
    const int Mdegree_180 = 2048 * 35;
    const int Mdegree_90 = 1024 * 35;

    void Tuning(float kp, float kd, float sine_t);
    void TuningLoopTask();
    void PeriodicMotionTester(queue<can_frame> &sendBuffer);

    // Functions for DrumRobot PathGenerating
    vector<double> c_MotorAngle;
    vector<vector<double>> right_inst;
    vector<vector<double>> left_inst;

    int bpm = 80;
    vector<double> time_arr;
    vector<vector<int>> RA, LA;
    vector<int> RF, LF;

    double theta0_standby = 0;
    double theta1_standby = M_PI / 2;
    double theta2_standby = M_PI / 2;
    double theta3_standby = M_PI / 6;
    double theta4_standby = 2 * M_PI / 3;
    double theta5_standby = M_PI / 6;
    double theta6_standby = 2 * M_PI / 3;

    vector<double> standby = {theta0_standby, theta1_standby, theta2_standby, theta3_standby, theta4_standby, theta5_standby, theta6_standby};

    struct can_frame frame;

    int end = RF.size();
    int line = 0;

    double p_R = 0; // 오른손 이전 악기 유무
	double p_L = 0; // 왼손 이전 악기 유무
	double c_R = 0; // 오른손 현재 악기 유무
	double c_L = 0; // 왼손 현재 악기 유무

    /*
	vector<double> P1 = {0.265, -0.391, -0.039837};	 // RightArm Standby
	vector<double> P2 = {-0.265, -0.391, -0.039837}; // LeftArm Standby
	int n_inst = 10;

	vector<double> R = {0.368, 0.414, 0.368, 0.414};
	double s = 0.530;
	double z0 = 0.000;
	*/
	vector<double> P1 = {0.3, -0.45, -0.0866};	 // RightArm Standby
	vector<double> P2 = {-0.3, -0.45, -0.0866}; // LeftArm Standby
	int n_inst = 10;

	vector<double> R = {0.500, 0.400, 0.500, 0.400};
	double s = 0.600;
	double z0 = 0.000;
	vector<vector<double>> q;
    
    void GetMusicSheet();
    void GetReadyArr();
    void PathLoopTask(queue<can_frame> &sendBuffer);
    void GetBackArr();

    // Functions for SendLoop
    template <typename MotorMap>
    void writeToSocket(MotorMap &motorMap, std::queue<can_frame> &sendBuffer, const std::map<std::string, int> &sockets);
    void SendLoopTask(queue<can_frame> &sendBuffer);

    // Functions for RecieveLoop
    const int NUM_FRAMES = 100;
    const int TIME_THRESHOLD_MS = 100;

    void initializeMotorCounts(std::map<std::string, int> &motor_count_per_port);
    void checkUserInput();
    void RecieveLoopTask(queue<can_frame> &recieveBuffer);
    void handleSocketRead(int socket_descriptor, int motor_count, queue<can_frame> &recieveBuffer);

    // Funtions for SensorLoop

    void SensorLoopTask();
    void SensorInitialization();
};