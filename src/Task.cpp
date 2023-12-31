#include "../include/Task.hpp"

Task::Task(map<string, shared_ptr<TMotor>, CustomCompare> &input_tmotors,
           map<string, shared_ptr<MaxonMotor>> &input_maxonMotors)
    : tmotors(input_tmotors),
      maxonMotors(input_maxonMotors),
      canUtils(extractIfnamesFromMotors(input_tmotors)) // 멤버 초기화
{
    state.store(0);
    chartHandler = nullptr;
}

vector<string> Task::extractIfnamesFromMotors(const map<string, shared_ptr<TMotor>, CustomCompare> &motors)
{
    set<string> interface_names;
    for (const auto &motor_pair : motors)
    {
        interface_names.insert(motor_pair.second->interFaceName);
    }
    return vector<string>(interface_names.begin(), interface_names.end());
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for Loops Managing
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::operator()()
{
    // Begin Operation
    ActivateControlTask();
    GetMusicSheet();
    initializeTMotors();
    std::cout << "Start Ready. \n";
    bool isHomeSet = false; // Home 설정 상태

    while (true)
    {
        // 화면 클리어
        int result = system("clear");
        if (result != 0)
        {
            std::cerr << "Error during clear screen" << std::endl;
        }

        // UI/UX 개선을 위한 화면 디자인
        std::cout << "========================================\n"
                     "DrumRobot Control Panel\n"
                     "========================================\n"
                     "[H] Home - Initialize motor positions"
                  << (isHomeSet ? " (Set)" : "") << "\n"
                                                    "[R] Ready - Prepare the arrangement\n"
                                                    "[P] Perform - Start the performance\n"
                                                    "[T] Test - Enter tuning mode\n"
                                                    "[C] Check - Check current motor positions\n"
                                                    "[E] Exit - Terminate the program\n"
                                                    "========================================\n"
                                                    "Enter your choice: ";

        char userInput;
        std::cin >> userInput;
        userInput = std::tolower(userInput);

        if (userInput == 'e')
        {
            break;
            state = Terminate;
        }
        else if (userInput == 'h')
        {
            SetHome();
            isHomeSet = true;
        }
        else if (userInput == 'r')
        {
            std::cout << "Setting ...... \n";
            GetReadyArr(sendBuffer);
        }
        else if (userInput == 'p')
        {
            canUtils.restart_all_can_ports();
            std::cout << "Performing ...... \n";
            std::thread sendThread(&Task::SendLoopTask, this, std::ref(sendBuffer));
            std::thread readThread(&Task::RecieveLoopTask, this, std::ref(recieveBuffer));
            sendThread.join();
            readThread.join();
            std::cout << "........End performance \n";
        }
        else if (userInput == 't')
        {
            TuningLoopTask();
        }
        else if (userInput == 'c') // Check current motor positions
        {
            do
            {
                CheckAllMotorsCurrentPosition();
                std::cout << "Do you want to check again? (y/n): ";
                std::cin >> userInput;
                userInput = std::tolower(userInput);
            } while (userInput == 'y');
        }
    }
    DeactivateControlTask();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for Activate / Deactivate Motors
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::ActivateControlTask()
{
    struct can_frame frame;

    // sockets 맵의 모든 항목에 대해 set_socket_timeout 설정
    for (const auto &socketPair : canUtils.sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 5, 0) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }
    if (!tmotors.empty())
    {
        // 첫 번째 for문: 모터 상태 확인 및 제어 모드 설정
        for (const auto &motorPair : tmotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<TMotor> motor = motorPair.second;

            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Motor [" << motorName << "] status check passed." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Motor [" << motorName << "] status check failed." << std::endl;
                               }
                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Control mode set for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to set control mode for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForZeroing());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Zero set for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to set zero for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
        }
    }
    else
    {
        std::cout << "No Tmotors to process." << std::endl;
    }

    // MaxonMotor
    if (!maxonMotors.empty())
    {
        for (const auto &motorPair : maxonMotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<MaxonMotor> motor = motorPair.second;

            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Motor [" << motorName << "] status check passed." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Motor [" << motorName << "] status check failed." << std::endl;
                               }
                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForPosOffset());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });
            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForTorqueOffset());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForOperational());
            sendNotRead(canUtils.sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success) {

                        });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForEnable());
            sendNotRead(canUtils.sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success)
                        {
                            if (success)
                            {
                                std::cout << "Enabled for motor [" << motorName << "]." << std::endl;
                            }
                            else
                            {
                                std::cerr << "Failed to Enable for motor [" << motorName << "]." << std::endl;
                            }
                        });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForSync());
            writeAndReadForSync(canUtils.sockets.at(motor->interFaceName), name, frame, maxonMotors.size(),
                                [](const std::string &motorName, bool success) {

                                });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
        }
    }
    else
    {
        std::cout << "No Maxon motors to process." << std::endl;
    }
}

void Task::DeactivateControlTask()
{
    struct can_frame frame;

    // sockets 맵의 모든 항목에 대해 set_socket_timeout 설정
    for (const auto &socketPair : canUtils.sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 50000) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

    // Tmotors
    if (!tmotors.empty())
    {
        for (const auto &motorPair : tmotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<TMotor> motor = motorPair.second;

            fillCanFrameFromInfo(&frame, motor->getCanFrameForCheckMotor());
            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForExit());

            sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "Exiting control mode for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to exit control mode for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
        }
    }
    else
    {
        std::cout << "No Tmotors to process." << std::endl;
    }

    // MaxonMotor
    if (!maxonMotors.empty())
    {
        for (const auto &motorPair : maxonMotors)
        {
            std::string name = motorPair.first;
            std::shared_ptr<MaxonMotor> motor = motorPair.second;

            fillCanFrameFromInfo(&frame, motor->getCanFrameForQuickStop());
            sendNotRead(canUtils.sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success)
                        {
                            if (success)
                            {
                                std::cout << "Exiting for motor [" << motorName << "]." << std::endl;
                            }
                            else
                            {
                                std::cerr << "Failed to exit for motor [" << motorName << "]." << std::endl;
                            }
                        });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForSync());
            writeAndReadForSync(canUtils.sockets.at(motor->interFaceName), name, frame, maxonMotors.size(),
                                [](const std::string &motorName, bool success) {

                                });

            // 구분자 추가
            std::cout << "=======================================" << std::endl;
        }
    }
    else
    {
        std::cout << "No Maxon motors to process." << std::endl;
    }
}

void Task::initializeTMotors()
{
    for (auto &motor_pair : tmotors)
    {
        std::shared_ptr<TMotor> &motor = motor_pair.second;

        // 각 모터 이름에 따른 멤버 변수 설정
        if (motor_pair.first == "waist")
        {
            motor->cwDir = 1.0f;
            motor->rMin = -M_PI * 0.75f;     // -120deg
            motor->rMax = M_PI / 2.0f;      // 90deg
        }
        else if (motor_pair.first == "R_arm1")
        {
            motor->cwDir = 1.0f;
            motor->sensorBit = 1;
            motor->rMin = 0.0f;        // 0deg
            motor->rMax = M_PI;         // 180deg
        }
        else if (motor_pair.first == "L_arm1")
        {
            motor->cwDir = 1.0f;
            motor->sensorBit = 1;
            motor->rMin = 0.0f;        // 0deg
            motor->rMax = M_PI;         // 180deg
        }
        else if (motor_pair.first == "R_arm2")
        {
            motor->cwDir = 1.0f;
            motor->sensorBit = 1;
            motor->rMin = -M_PI / 4.0f; // -45deg
            motor->rMax = M_PI / 2.0f;  // 90deg
        }
        else if (motor_pair.first == "R_arm3")
        {
            motor->cwDir = 1.0f;
            motor->sensorBit = 1;
            motor->rMin = 0.0f;                // 0deg
            motor->rMax = M_PI * 0.75f;   // 135deg
        }
        else if (motor_pair.first == "L_arm2")
        {
            motor->cwDir = -1.0f;
            motor->sensorBit = 0;
            motor->rMin = -M_PI / 2.0f; // -90deg
            motor->rMax = M_PI / 4.0f;  // 45deg
        }
        else if (motor_pair.first == "L_arm3")
        {
            motor->cwDir = -1.0f;
            motor->sensorBit = 2;
            motor->rMin = -M_PI * 0.75f;  // -135deg
            motor->rMax = 0.0f;                 // 0deg
        }
    }

    map<string, shared_ptr<MaxonMotor>> maxonMotors;
    /*maxonMotors["a_maxon"] = make_shared<MaxonMotor>(0x001,
                                                          vector<uint32_t>{0x201, 0x301},
                                                          vector<uint32_t>{0x181},
                                                          "can0");
    maxonMotors["b_maxon"] = make_shared<MaxonMotor>(0x002,
                                                          vector<uint32_t>{0x202, 0x302},
                                                          vector<uint32_t>{0x182},
                                                          "can0");*/
};

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for Testing
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::Tuning(float kp, float kd, float sine_t, const std::string selectedMotor, int cycles, float peakAngle, int pathType)
{
    // sockets 맵의 모든 항목에 대해 set_socket_timeout 설정
    for (const auto &socketPair : canUtils.sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 50000) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

    /*

    std::string fileName;



    // 파일 이름 자동 설정
    std::string folderName = "TuningData";
    std::string baseName = ss.str(); // ss.str()로 stringstream의 내용을 std::string으로 가져옵니다.
    fileName = folderName + "/" + baseName;

    // CSV 파일을 쓰기 모드로 열기

    std::ofstream csvFile(fileName);
    if (!csvFile.is_open())
    {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return; // 또는 다른 오류 처리
    }
    */

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2); // 소수점 둘째 자리까지만
    ss << "kp_" << kp << "_kd_" << kd << "_Hz_" << 1 / sine_t;
    std::string parameter = ss.str();

    // CSV 파일명 설정
    std::string FileName1 = "../../READ/" + parameter + "_in.txt";

    // CSV 파일 열기
    std::ofstream csvFileIn(FileName1);

    if (!csvFileIn.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileIn << "Start file"
              << "\n";

    // CSV 파일명 설정
    std::string FileName2 = "../../READ/" + parameter + "_out.txt";

    // CSV 파일 열기
    std::ofstream csvFileOut(FileName2);

    if (!csvFileOut.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFileOut << "CAN_ID,p_act,tff_des,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float peakRadian = peakAngle * M_PI / 180.0; // 피크 각도를 라디안으로 변환
    float amplitude = peakRadian;

    float sample_time = 0.005;
    int max_samples = static_cast<int>(sine_t / sample_time);
    float v_des = 0, p_des = 0;
    float tff_des = 0;
    float p_act, v_act, tff_act;
    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            for (auto &entry : tmotors)
            {
                if (entry.first != selectedMotor)
                    continue;

                std::shared_ptr<TMotor> &motor = entry.second;

                if ((int)motor->nodeId == 7)
                {
                    csvFileIn << std::dec << p_des << "0,0,0,0,0,0";
                }
                else
                {
                    for (int i = 0; i < (int)motor->nodeId; i++)
                    {
                        csvFileIn << "0,";
                    }
                    csvFileIn << std::dec << p_des << ",";
                    for (int i = 0; i < (6 - (int)motor->nodeId); i++)
                    {
                        csvFileIn << "0,";
                    }
                }

                float local_time = std::fmod(time, sine_t);
                if (pathType == 1) // 1-cos 경로
                {
                    p_des = amplitude * (1 - cosf(2 * M_PI * local_time / sine_t)) / 2;
                }
                else if (pathType == 2) // 1-cos 및 -1+cos 결합 경로
                {
                    if (local_time < sine_t / 2)
                        p_des = amplitude * (1 - cosf(4 * M_PI * local_time / sine_t)) / 2;
                    else
                        p_des = amplitude * (-1 + cosf(4 * M_PI * (local_time - sine_t / 2) / sine_t)) / 2;
                }

                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, v_des, kp, kd, tff_des);
                csvFileOut << "0x" << std::hex << std::setw(4) << std::setfill('0') << motor->nodeId;

                chrono::system_clock::time_point external = std::chrono::system_clock::now();
                while (1)
                {
                    chrono::system_clock::time_point internal = std::chrono::system_clock::now();
                    chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);
                    if (elapsed_time.count() >= 5000)
                    {

                        ssize_t bytesWritten = write(canUtils.sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));
                        if (bytesWritten == -1)
                        {
                            std::cerr << "Failed to write to socket for interface: " << motor->interFaceName << std::endl;
                            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                        }

                        ssize_t bytesRead = read(canUtils.sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));

                        if (bytesRead == -1)
                        {
                            std::cerr << "Failed to read from socket for interface: " << motor->interFaceName << std::endl;
                            return;
                        }
                        else
                        {
                            std::tuple<int, float, float, float> result = TParser.parseRecieveCommand(*motor, &frame);

                            p_act = std::get<1>(result);
                            v_act = std::get<1>(result);
                            tff_act = std::get<3>(result);
                            tff_des = kp * (p_des - p_act) + kd * (v_des - v_act);
                            csvFileOut << ',' << std::dec << p_act << ',' << tff_des << ',' << tff_act << '\n';
                            break;
                        }
                    }
                }
            }
            csvFileIn << "\n";
        }
    }
    csvFileIn.close();
    csvFileOut.close();
}

void Task::TuningLoopTask()
{
    FixMotorPosition();
    std::string userInput, selectedMotor, fileName;
    float kp, kd, peakAngle;
    float sine_t = 2.0;
    int cycles = 2, pathType;

    if (!tmotors.empty())
    {
        selectedMotor = tmotors.begin()->first;
    }

    InitializeTuningParameters(selectedMotor, kp, kd, peakAngle, pathType);
    while (true)
    {
        /*int result = system("clear");
        if (result != 0)
        {
            std::cerr << "Error during clear screen" << std::endl;
        }
        */
        std::string pathTypeDescription;
        if (pathType == 1)
        {
            pathTypeDescription = "1: 1 - cos (0 -> peak -> 0)";
        }
        else if (pathType == 2)
        {
            pathTypeDescription = "2: 1 - cos & cos - 1 (0 -> peak -> 0 -> -peak -> 0)";
        }

        std::cout << "================ Tuning Menu ================\n";
        std::cout << "Available Motors:\n";
        for (const auto &motor_pair : tmotors)
        {
            std::cout << " - " << motor_pair.first << "\n";
        }
        std::cout << "---------------------------------------------\n";
        std::cout << "Selected Motor: " << selectedMotor << "\n";
        std::cout << "Kp: " << kp << " | Kd: " << kd << "\n";
        std::cout << "Sine Period: " << sine_t << " | Cycles: " << cycles << " | Hz: " << 1 / sine_t << "\n";
        std::cout << "Peak Angle: " << peakAngle << " | Path Type: " << pathTypeDescription << "\n";
        std::cout << "\nCommands:\n";
        std::cout << "[S]elect Motor | [KP] | [KD] | [Peak] | [Type]\n";
        std::cout << "[P]eriod | [C]ycles | [R]un | [A]nalyze | [E]xit\n";
        std::cout << "=============================================\n";
        std::cout << "Enter Command: ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput[0] == 'e')
        {
            break;
        }
        else if (userInput[0] == 's')
        {
            while (true)
            {
                std::cout << "Enter the name of the motor to tune: ";
                std::cin >> selectedMotor;
                if (tmotors.find(selectedMotor) != tmotors.end())
                {
                    InitializeTuningParameters(selectedMotor, kp, kd, peakAngle, pathType);
                    break;
                }
                else
                {
                    std::cout << "Invalid motor name. Please enter a valid motor name.\n";
                }
            }
        }
        else if (userInput == "kp")
        {
            std::cout << "Enter Desired Kp: ";
            std::cin >> kp;
        }
        else if (userInput == "kd")
        {
            std::cout << "Enter Desired Kd: ";
            std::cin >> kd;
        }
        else if (userInput == "peak")
        {
            std::cout << "Enter Desired Peak Angle: ";
            std::cin >> peakAngle;
        }
        else if (userInput == "type")
        {
            std::cout << "Select Path Type:\n";
            std::cout << "1: 1 - cos (0 -> peak -> 0)\n";
            std::cout << "2: 1 - cos & cos - 1 (0 -> peak -> 0 -> -peak -> 0)\n";
            std::cout << "Enter Path Type (1 or 2): ";
            std::cin >> pathType;

            if (pathType != 1 && pathType != 2)
            {
                std::cout << "Invalid path type. Please enter 1 or 2.\n";
                pathType = 1; // 기본값으로 재설정
            }
        }
        else if (userInput[0] == 'p')
        {
            std::cout << "Enter Desired Sine Period: ";
            std::cin >> sine_t;
        }
        else if (userInput[0] == 'c')
        {
            std::cout << "Enter Desired Cycles: ";
            std::cin >> cycles;
        }
        else if (userInput[0] == 'r')
        {
            Task::Tuning(kp, kd, sine_t, selectedMotor, cycles, peakAngle, pathType);
        }
    }
}

void Task::InitializeTuningParameters(const std::string selectedMotor, float &kp, float &kd, float &peakAngle, int &pathType)
{
    if (selectedMotor == "waist")
    {
        kp = 350.0;
        kd = 3.5;
        peakAngle = 30;
        pathType = 2;
    }
    else if (selectedMotor == "R_arm1" || selectedMotor == "L_arm1")
    {
        kp = 170.0;
        kd = 2.5;
        peakAngle = 120;
        pathType = 1;
    }
    else if (selectedMotor == "R_arm2" || selectedMotor == "L_arm2")
    {
        kp = 250.0;
        kd = 2.5;
        peakAngle = -90;
        pathType = 1;
    }
    else if (selectedMotor == "R_arm3" || selectedMotor == "L_arm3")
    {
        kp = 200.0;
        kd = 3.5;
        peakAngle = -90;
        pathType = 1;
    }
    // 추가적인 모터 이름과 매개변수를 이곳에 추가할 수 있습니다.
}

void Task::setChartHandler(ChartHandler *handler)
{
    chartHandler = handler;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for DrumRobot PathGenerating
//////////////////////////////////////////////////////////////////////////////////////////////////

string Task::trimWhitespace(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

vector<double> Task::connect(vector<double> &Q1, vector<double> &Q2, int k, int n)
{
    vector<double> Qi;
    std::vector<double> A, B;

    // Compute A and B
    for (long unsigned int i = 0; i < Q1.size(); ++i)
    {
        A.push_back(0.5 * (Q1[i] - Q2[i]));
        B.push_back(0.5 * (Q1[i] + Q2[i]));
    }

    // Compute Qi using the provided formula
    for (long unsigned int i = 0; i < Q1.size(); ++i)
    {
        double val = A[i] * cos(M_PI * k / n) + B[i];
        Qi.push_back(val);
    }

    return Qi;
}

// 행렬의 determinant 계산 함수
double determinant(double mat[3][3])
{
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) -
           mat[0][1] * (mat[1][0] * mat[2][2] - mat[2][0] * mat[1][2]) +
           mat[0][2] * (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]);
}

// 역행렬 계산 함수
void inverseMatrix(double mat[3][3], double inv[3][3])
{
    double det = determinant(mat);

    if (det == 0)
    {
        std::cerr << "역행렬이 존재하지 않습니다." << std::endl;
        return;
    }

    double invDet = 1.0 / det;

    inv[0][0] = (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) * invDet;
    inv[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) * invDet;
    inv[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * invDet;

    inv[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) * invDet;
    inv[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * invDet;
    inv[1][2] = (mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]) * invDet;

    inv[2][0] = (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]) * invDet;
    inv[2][1] = (mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1]) * invDet;
    inv[2][2] = (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]) * invDet;
}

void Task::iconnect(vector<double> &P0, vector<double> &P1, vector<double> &P2, vector<double> &V0, double t1, double t2, double t)
{
    vector<double> V1;
    vector<double> p_out;
    vector<double> v_out;
    for (size_t i = 0; i < P0.size(); ++i)
    {
        if ((P1[i] - P0[i]) / (P2[i] - P1[i]) > 0)
            V1.push_back((P2[i] - P0[i]) / t2);
        else
            V1.push_back(0);

        double f = P0[i];
        double d = 0;
        double e = V0[i];

        double M[3][3] = {
            {20.0 * pow(t1, 2), 12.0 * t1, 6.0},
            {5.0 * pow(t1, 4), 4.0 * pow(t1, 3), 3.0 * pow(t1, 2)},
            {pow(t1, 5), pow(t1, 4), pow(t1, 3)}};
        double ANS[3] = {0, V1[i] - V0[i], P1[i] - P0[i] - V0[i] * t1};

        double invM[3][3];
        inverseMatrix(M, invM);
        // Multiply the inverse of T with ANS
        double tem[3];
        for (size_t j = 0; j < 3; ++j)
        {
            tem[j] = 0;
            for (size_t k = 0; k < 3; ++k)
            {
                tem[j] += invM[j][k] * ANS[k];
            }
        }

        double a = tem[0];
        double b = tem[1];
        double c = tem[2];

        p_out.push_back(a * pow(t, 5) + b * pow(t, 4) + c * pow(t, 3) + d * pow(t, 2) + e * t + f);
        v_out.push_back(5 * a * pow(t, 4) + 4 * b * pow(t, 3) + 3 * c * pow(t, 2) + 3 * d * t + e);
    }

    p.push_back(p_out);
    v.push_back(v_out);
}

vector<double> Task::IKfun(vector<double> &P1, vector<double> &P2, vector<double> &R, double s, double z0)
{
    vector<double> Qf;

    double X1 = P1[0], Y1 = P1[1], z1 = P1[2];
    double X2 = P2[0], Y2 = P2[1], z2 = P2[2];
    double r1 = R[0], r2 = R[1], r3 = R[2], r4 = R[3];

    int j = 0;
    vector<double> the3(100);
    for (int i = 0; i < 100; i++)
    {
        the3[i] = -M_PI / 2 + (M_PI * i) / 99;
    }

    double zeta = z0 - z2;

    double det_the4;
    double the34;
    double the4;
    double r;
    double det_the1;
    double the1;
    double det_the0;
    double the0;
    double L;
    double det_the2;
    double the2;
    double T;
    double det_the5;
    double sol;
    double the5;
    double alpha, beta, gamma;
    double det_the6;
    double rol;
    double the6;
    double Z;

    vector<double> Q0;
    vector<double> Q1;
    vector<double> Q2;
    vector<double> Q3;
    vector<double> Q4;
    vector<double> Q5;
    vector<double> Q6;

    for (int i = 0; i < 99; i++)
    {
        det_the4 = (z0 - z1 - r1 * cos(the3[i])) / r2;

        if (det_the4 < 1 && det_the4 > -1)
        {
            the34 = acos((z0 - z1 - r1 * cos(the3[i])) / r2);
            the4 = the34 - the3[i];

            if (the4 > 0)
            {
                r = r1 * sin(the3[i]) + r2 * sin(the34);

                det_the1 = (X1 * X1 + Y1 * Y1 - r * r - s * s / 4) / (s * r);
                if (det_the1 < 1 && det_the1 > -1)
                {
                    the1 = acos(det_the1);

                    alpha = asin(X1 / sqrt(X1 * X1 + Y1 * Y1));
                    det_the0 = (s / 4 + (X1 * X1 + Y1 * Y1 - r * r) / s) / sqrt(X1 * X1 + Y1 * Y1);
                    if (det_the0 < 1 && det_the0 > -1)
                    {
                        the0 = asin(det_the0) - alpha;

                        L = sqrt(pow(X2 - 0.5 * s * cos(the0 + M_PI), 2) +
                                 pow(Y2 - 0.5 * s * sin(the0 + M_PI), 2));
                        det_the2 = (X2 + 0.5 * s * cos(the0)) / L;

                        if (det_the2 < 1 && det_the2 > -1)
                        {
                            the2 = acos(det_the2) - the0;

                            T = (zeta * zeta + L * L + r3 * r3 - r4 * r4) / (r3 * 2);
                            det_the5 = L * L + zeta * zeta - T * T;

                            if (det_the5 > 0)
                            {
                                sol = T * L - abs(zeta) * sqrt(L * L + zeta * zeta - T * T);
                                sol /= (L * L + zeta * zeta);
                                the5 = asin(sol);

                                alpha = L - r3 * sin(the5);
                                beta = r4 * sin(the5);
                                gamma = r4 * cos(the5);

                                det_the6 = gamma * gamma + beta * beta - alpha * alpha;

                                if (det_the6 > 0)
                                {
                                    rol = alpha * beta - abs(gamma) * sqrt(det_the6);
                                    rol /= (beta * beta + gamma * gamma);
                                    the6 = acos(rol);
                                    Z = z0 - r1 * cos(the5) - r2 * cos(the5 + the6);

                                    if (Z < z2 + 0.001 && Z > z2 - 0.001)
                                    {
                                        Q0.push_back(the0);
                                        Q1.push_back(the1);
                                        Q2.push_back(the2);
                                        Q3.push_back(the3[i]);
                                        Q4.push_back(the4);
                                        Q5.push_back(the5);
                                        Q6.push_back(the6);

                                        j++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    vector<vector<double>> Q;
    Q.push_back(Q0);
    Q.push_back(Q1);
    Q.push_back(Q2);
    Q.push_back(Q3);
    Q.push_back(Q4);
    Q.push_back(Q5);
    Q.push_back(Q6);

    // Find the median index
    int num_columns = Q[0].size();
    int index_theta0_min = 0, index_theta0_max = 0;

    // Find index of minimum and maximum values in the first row of A
    for (int i = 1; i < num_columns; i++)
    {
        if (Q[0][i] > Q[0][index_theta0_max])
            index_theta0_max = i;
        if (Q[0][i] < Q[0][index_theta0_min])
            index_theta0_min = i;
    }

    // Calculate the median index of the min and max
    int index_theta0_med = round((index_theta0_min + index_theta0_max) / 2);

    Qf.resize(7);

    for (int i = 0; i < 7; i++)
    {
        if(i == 5 || i == 6){
            Qf[i] = -Q[i][index_theta0_med];
        }
        else{
            Qf[i] = Q[i][index_theta0_med];
        }
    }

    return Qf;
}

void Task::GetMusicSheet()
{
    ifstream inputFile("../include/rT.txt");

    if (!inputFile.is_open())
    {
        cerr << "Failed to open the file."
             << "\n";
    }

    // Read data into a 2D vector
    vector<vector<double>> inst_xyz(6, vector<double>(8, 0));

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 8; ++j)
        {
            inputFile >> inst_xyz[i][j];
            if(i == 0 || i == 1 || i == 3 || i == 4){
                inst_xyz[i][j] = inst_xyz[i][j] * 1.25;
            }
        }
    }

    // Extract the desired elements
    vector<double> right_B = {0, 0, 0};
    vector<double> right_S;
    vector<double> right_FT;
    vector<double> right_MT;
    vector<double> right_HT;
    vector<double> right_HH;
    vector<double> right_R;
    vector<double> right_RC;
    vector<double> right_LC;

    for (int i = 0; i < 3; ++i)
    {
        right_S.push_back(inst_xyz[i][0]);
        right_FT.push_back(inst_xyz[i][1]);
        right_MT.push_back(inst_xyz[i][2]);
        right_HT.push_back(inst_xyz[i][3]);
        right_HH.push_back(inst_xyz[i][4]);
        right_R.push_back(inst_xyz[i][5]);
        right_RC.push_back(inst_xyz[i][6]);
        right_LC.push_back(inst_xyz[i][7]);
    }

    vector<double> left_B = {0, 0, 0};
    vector<double> left_S;
    vector<double> left_FT;
    vector<double> left_MT;
    vector<double> left_HT;
    vector<double> left_HH;
    vector<double> left_R;
    vector<double> left_RC;
    vector<double> left_LC;

    for (int i = 3; i < 6; ++i)
    {
        left_S.push_back(inst_xyz[i][0]);
        left_FT.push_back(inst_xyz[i][1]);
        left_MT.push_back(inst_xyz[i][2]);
        left_HT.push_back(inst_xyz[i][3]);
        left_HH.push_back(inst_xyz[i][4]);
        left_R.push_back(inst_xyz[i][5]);
        left_RC.push_back(inst_xyz[i][6]);
        left_LC.push_back(inst_xyz[i][7]);
    }

    // Combine the elements into right_inst and left_inst
    right_inst = {right_B, right_RC, right_R, right_S, right_HH, right_HH, right_FT, right_MT, right_LC, right_HT};
    left_inst = {left_B, left_RC, left_R, left_S, left_HH, left_HH, left_FT, left_MT, left_LC, left_HT};

    /////////// 드럼로봇 악기정보 텍스트 -> 딕셔너리 변환
    map<string, int> instrument_mapping = {
        {"0", 10}, {"1", 3}, {"2", 6}, {"3", 7}, {"4", 9}, {"5", 4}, {"6", 5}, {"7", 4}, {"8", 8}, {"11", 3}, {"51", 3}, {"61", 3}, {"71", 3}, {"81", 3}, {"91", 3}};

    string score_path = "../include/codeConfession_short.txt";

    ifstream file(score_path);
    if (!file.is_open())
    {
        cerr << "Error opening file." << endl;
    }

    string line;
    int lineIndex = 0;
    while (getline(file, line))
    {
        istringstream iss(line);
        string item;
        vector<string> columns;
        while (getline(iss, item, '\t'))
        {
            item = trimWhitespace(item);
            columns.push_back(item);
        }

        vector<int> inst_arr_R(10, 0), inst_arr_L(10, 0);
        time_arr.push_back(stod(columns[1]) * 100 / bpm);

        if (columns[2] != "0")
        {
            inst_arr_R[instrument_mapping[columns[2]]] = 1;
        }
        if (columns[3] != "0")
        {
            inst_arr_L[instrument_mapping[columns[3]]] = 1;
        }

        RF.push_back(stoi(columns[6]) == 1 ? 1 : 0);
        LF.push_back(stoi(columns[7]) == 2 ? 1 : 0);

        RA.push_back(inst_arr_R);
        LA.push_back(inst_arr_L);

        lineIndex++;
    }

    file.close();

    end = RF.size();
}

void Task::GetReadyArr(queue<can_frame> &sendBuffer)
{
    struct can_frame frame;
    struct can_frame frameToProcess;
    chrono::system_clock::time_point external = std::chrono::system_clock::now();
    CheckAllMotorsCurrentPosition();

    vector<double> Qi;
    vector<vector<double>> q_ready;
    for (auto &entry : tmotors)
    {
        std::shared_ptr<TMotor> &motor = entry.second;
        c_MotorAngle[motor_mapping[entry.first]] = motor->currentPos;
    }

    //// 준비자세 배열 생성
    int n = 800;
    for (int k = 0; k < n; k++)
    {
        Qi = connect(c_MotorAngle, standby, k, n);
        q_ready.push_back(Qi);

        for (auto &entry : tmotors)
        {
            std::shared_ptr<TMotor> &motor = entry.second;
            float p_des = Qi[motor_mapping[entry.first]];
            TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 200.0, 3.0, 0.0);
            sendBuffer.push(frame);
        }
        // cout << "\n";
    }

    c_MotorAngle = Qi;

    //// 준비자세 동작
    while (sendBuffer.size() != 0)
    {
        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

        if (elapsed_time.count() >= 5000) // 5ms
        {
            external = std::chrono::system_clock::now();

            Task::writeToSocket(tmotors, sendBuffer, canUtils.sockets);

            if (!maxonMotors.empty())
            {
                Task::writeToSocket(maxonMotors, sendBuffer, canUtils.sockets);

                // sync 신호 전송
                frameToProcess = sendBuffer.front();
                sendBuffer.pop();
                auto it = canUtils.sockets.find(maxonMotors.begin()->second->interFaceName);

                if (it != canUtils.sockets.end())
                {
                    int socket_descriptor_for_sync = it->second;
                    ssize_t bytesWritten = write(socket_descriptor_for_sync, &frameToProcess, sizeof(struct can_frame));

                    handleError(bytesWritten, maxonMotors.begin()->second->interFaceName);
                }
                else
                {
                    std::cerr << "Socket not found for interface: " << maxonMotors.begin()->second->interFaceName << std::endl;
                }
            }
        }
    }
}

void Task::PathLoopTask(queue<can_frame> &sendBuffer)
{
    struct can_frame frame;

    // 처음 시작할 때 Q2, Q4 모두 계산
    if (line == 0)
    {
        c_R = 0;
        c_L = 0;

        for (int j = 0; j < n_inst; ++j)
        {
            if (RA[line][j] != 0)
            {
                P1 = right_inst[j];
                c_R = 1;
            }
            if (LA[line][j] != 0)
            {
                P2 = left_inst[j];
                c_L = 1;
            }
        }

        if (c_R == 0 && c_L == 0)
        { // 왼손 & 오른손 안침
            Q1 = c_MotorAngle;
            if (p_R == 1)
            {
                Q1[4] = Q1[4] + M_PI / 36;
            }
            if (p_L == 1)
            {
                Q1[6] = Q1[6] - M_PI / 36;
            }
            Q2 = Q1;
        }
        else
        {
            Q1 = IKfun(P1, P2, R, s, z0);
            Q2 = Q1;
            if (c_R == 0)
            { // 왼손만 침
                Q1[4] = Q1[4] + M_PI / 36;
                Q2[4] = Q2[4] + M_PI / 36;
                Q1[6] = Q1[6] - M_PI / 18;
            }
            if (c_L == 0)
            { // 오른손만 침
                Q1[4] = Q1[4] + M_PI / 18;
                Q2[6] = Q2[6] - M_PI / 36;
                Q2[6] = Q2[6] - M_PI / 36;
            }
            else
            { // 왼손 & 오른손 침
                Q1[4] = Q1[4] + M_PI / 18;
                Q1[6] = Q1[6] - M_PI / 18;
            }
        }

        p_R = c_R;
        p_L = c_L;

        line++;

        p.push_back(c_MotorAngle);
        v.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }

    c_R = 0;
    c_L = 0;

    for (int j = 0; j < n_inst; ++j)
    {
        if (RA[line][j] != 0)
        {
            P1 = right_inst[j];
            c_R = 1;
        }
        if (LA[line][j] != 0)
        {
            P2 = left_inst[j];
            c_L = 1;
        }
    }

    if (c_R == 0 && c_L == 0)
    { // 왼손 & 오른손 안침
        Q3 = c_MotorAngle;
        if (p_R == 1)
        {
            Q3[4] = Q3[4] + M_PI / 36;
        }
        if (p_L == 1)
        {
            Q3[6] = Q3[6] - M_PI / 36;
        }
        Q4 = Q3;
    }
    else
    {
        Q3 = IKfun(P1, P2, R, s, z0);
        Q4 = Q3;
        if (c_R == 0)
        { // 왼손만 침
            Q3[4] = Q3[4] + M_PI / 36;
            Q4[4] = Q4[4] + M_PI / 36;
            Q3[6] = Q3[6] - M_PI / 18;
        }
        if (c_L == 0)
        { // 오른손만 침
            Q3[4] = Q3[4] + M_PI / 18;
            Q4[6] = Q4[6] - M_PI / 36;
            Q4[6] = Q4[6] - M_PI / 36;
        }
        else
        { // 왼손 & 오른손 침
            Q3[4] = Q3[4] + M_PI / 18;
            Q3[6] = Q3[6] - M_PI / 18;
        }
    }

    p_R = c_R;
    p_L = c_L;

    double t1 = time_arr[line - 1];
    double t2 = time_arr[line];
    double t = 0.005;
    int n = round((t1 / 2) / t);
    vector<double> Pi;
    vector<double> Vi;
    vector<double> V0 = v.back();
    for (int i = 0; i < n; i++)
    {
        iconnect(c_MotorAngle, Q1, Q2, V0, t1 / 2, t1, t * i);
        Pi = p.back();
        Vi = v.back();

        for (auto &entry : tmotors)
        {
            std::shared_ptr<TMotor> &motor = entry.second;
            float p_des = Pi[motor_mapping[entry.first]];
            float v_des = Vi[motor_mapping[entry.first]];
            
            if(p_des < motor->rMin){
                cout << entry.first << " is out of range.  ( " << p_des << " => " << motor->rMin << " )\n";
                p_des = motor->rMin;
                v_des = 0.0f;
                getchar();
            }
            else if(p_des > motor->rMax){
                cout << entry.first << " is out of range.  ( " << p_des << " => " << motor->rMax << " )\n";
                p_des = motor->rMax;
                v_des = 0.0f;
                getchar();
            }
            
            TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, v_des, 200.0, 3.0, 0.0);
            sendBuffer.push(frame);
        }
    }
    V0 = v.back();
    for (int i = 0; i < n; i++)
    {
        iconnect(Q1, Q2, Q3, V0, t1 / 2, (t1 + t2) / 2, t * i);
        Pi = p.back();
        Vi = v.back();

        for (auto &entry : tmotors)
        {
            std::shared_ptr<TMotor> &motor = entry.second;
            float p_des = Pi[motor_mapping[entry.first]];
            float v_des = Vi[motor_mapping[entry.first]];
            
            if(p_des < motor->rMin){
                cout << entry.first << " is out of range.  ( " << p_des << " => " << motor->rMin << " )\n";
                p_des = motor->rMin;
                v_des = 0.0f;
                getchar();
            }
            else if(p_des > motor->rMax){
                cout << entry.first << " is out of range.  ( " << p_des << " => " << motor->rMax << " )\n";
                p_des = motor->rMax;
                v_des = 0.0f;
                getchar();
            }
            
            TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, v_des, 200.0, 3.0, 0.0);
            sendBuffer.push(frame);
        }
    }
    c_MotorAngle = p.back();
    Q1 = Q3;
    Q2 = Q4;
}

void Task::GetBackArr()
{
    struct can_frame frame;

    vector<double> Q0(7, 0);
    vector<vector<double>> q_finish;

    //// 끝나는자세 배열 생성
    vector<double> Qi;
    int n = 800;
    for (int k = 0; k < n; ++k)
    {
        Qi = connect(c_MotorAngle, Q0, k, n);
        q_finish.push_back(Qi);

        for (auto &entry : tmotors)
        {
            std::shared_ptr<TMotor> &motor = entry.second;
            float p_des = Qi[motor_mapping[entry.first]];
            TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 200.0, 3.0, 0.0);
            sendBuffer.push(frame);
        }
        // cout << "\n";
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for SendTask
//////////////////////////////////////////////////////////////////////////////////////////////////

template <typename MotorMap>
void Task::writeToSocket(MotorMap &motorMap, std::queue<can_frame> &sendBuffer, const std::map<std::string, int> &sockets)
{
    struct can_frame frameToProcess;

    for (auto &motor_pair : motorMap)
    {
        auto motor_ptr = motor_pair.second;
        auto interface_name = motor_ptr->interFaceName;

        frameToProcess = sendBuffer.front(); // sendBuffer에서 데이터 꺼내기
        sendBuffer.pop();

        if (sockets.find(interface_name) != sockets.end())
        {
            int socket_descriptor = sockets.at(interface_name);
            ssize_t bytesWritten = write(socket_descriptor, &frameToProcess, sizeof(struct can_frame));

            if (bytesWritten == -1)
            {
                writeFailCount++;
                if (writeFailCount >= 10)
                {
                    state = Terminate;
                    canUtils.restart_all_can_ports();
                    writeFailCount = 0; // 카운터 리셋
                }
            }
            else
            {
                writeFailCount = 0; // 성공 시 카운터 리셋
            }
        }
        else
        {
            std::cerr << "Socket not found for interface: " << interface_name << std::endl;
        }
    }
}

void Task::SendLoopTask(std::queue<can_frame> &sendBuffer)
{

    struct can_frame frameToProcess;
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    while (state.load() != Terminate)
    {

        if (sendBuffer.size() <= 10)
        {
            if (line < end)
            {
                cout << "line : " << line << ", total_line : " << end << "\n";
                PathLoopTask(sendBuffer);
                std::cout << sendBuffer.size() << "\n";
                line++;
            }
            else if (line == end)
            {
                cout << "Turn Back\n";
                GetBackArr();
                line++;
            }
            else if (sendBuffer.size() == 0)
            {
                state = Terminate;
                cout << "Performance is Over\n";
            }
        }

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::microseconds elapsed_time = chrono::duration_cast<chrono::microseconds>(internal - external);

        if (elapsed_time.count() >= 5000) // 5ms
        {
            external = std::chrono::system_clock::now();

            Task::writeToSocket(tmotors, sendBuffer, canUtils.sockets);

            if (!maxonMotors.empty())
            {
                Task::writeToSocket(maxonMotors, sendBuffer, canUtils.sockets);

                // sync 신호 전송
                frameToProcess = sendBuffer.front();
                sendBuffer.pop();
                auto it = canUtils.sockets.find(maxonMotors.begin()->second->interFaceName);

                if (it != canUtils.sockets.end())
                {
                    int socket_descriptor_for_sync = it->second;
                    ssize_t bytesWritten = write(socket_descriptor_for_sync, &frameToProcess, sizeof(struct can_frame));

                    handleError(bytesWritten, maxonMotors.begin()->second->interFaceName);
                }
                else
                {
                    std::cerr << "Socket not found for interface: " << maxonMotors.begin()->second->interFaceName << std::endl;
                }
            }
        }
    }

    // CSV 파일명 설정
    std::string pFileName = "p_in.csv";

    // CSV 파일 열기
    std::ofstream pFile(pFileName);

    if (!pFile.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    pFile << "0x007,0x001,0x002,0x003,0x004,0x005,0x006\n";

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : p)
    {
        for (const double cell : row)
        {
            pFile << std::fixed << std::setprecision(5) << cell;
            if (&cell != &row.back())
            {
                pFile << ","; // 쉼표로 셀 구분
            }
        }
        pFile << "\n"; // 다음 행으로 이동
    }

    // CSV 파일 닫기
    pFile.close();

    // CSV 파일명 설정
    std::string vFileName = "v_in.csv";

    // CSV 파일 열기
    std::ofstream vFile(vFileName);

    if (!vFile.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    vFile << "0x007,0x001,0x002,0x003,0x004,0x005,0x006\n";

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : v)
    {
        for (const double cell : row)
        {
            vFile << std::fixed << std::setprecision(5) << cell;
            if (&cell != &row.back())
            {
                vFile << ","; // 쉼표로 셀 구분
            }
        }
        vFile << "\n"; // 다음 행으로 이동
    }

    // CSV 파일 닫기
    vFile.close();

    /*
    // CSV 파일명 설정
    std::string csvFileName = "TuningData/DrumData_in.txt";

    // CSV 파일 열기
    std::ofstream csvFile(csvFileName);

    if (!csvFile.is_open())
    {
        std::cerr << "Error opening CSV file." << std::endl;
    }

    // 헤더 추가
    csvFile << "0x007,0x001,0x002,0x003,0x004,0x005,0x006\n";

    // 2차원 벡터의 데이터를 CSV 파일로 쓰기
    for (const auto &row : p)
    {
        for (const double cell : row)
        {
            csvFile << std::fixed << std::setprecision(5) << cell;
            if (&cell != &row.back())
            {
                csvFile << ","; // 쉼표로 셀 구분
            }
        }
        csvFile << "\n"; // 다음 행으로 이동
    }

    // CSV 파일 닫기
    csvFile.close();
    */

    std::cout << "연주 CSV 파일이 생성되었습니다: " << std::endl;

    std::cout << "SendLoop terminated\n";
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for RecieveTask
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::parse_and_save_to_csv(const std::string &csv_file_name)
{

    // CSV 파일 열기. 파일이 없으면 새로 생성됩니다.
    std::ofstream ofs(csv_file_name, std::ios::app);
    int id;
    float position, speed, torque;

    if (!ofs.is_open())
    {
        std::cerr << "Failed to open or create the CSV file: " << csv_file_name << std::endl;
        return;
    }

    // 파일이 새로 생성되었으면 CSV 헤더를 추가
    ofs.seekp(0, std::ios::end);
    if (ofs.tellp() == 0)
    {
        ofs << "CAN_ID,p_act,tff_des,tff_act\n";
    };

    while (!recieveBuffer.empty())
    {
        can_frame frame = recieveBuffer.front();
        recieveBuffer.pop();

        for (const auto &pair : tmotors)
        {
            std::shared_ptr<TMotor> motor = pair.second;
            if (motor->nodeId == frame.data[0])
            {
                std::tuple<int, float, float, float> parsedData = TParser.parseRecieveCommand(*motor, &frame);
                id = std::get<0>(parsedData);
                position = std::get<1>(parsedData);
                speed = std::get<2>(parsedData);
                torque = std::get<3>(parsedData);

                ofs << "0x" << std::hex << std::setw(4) << std::setfill('0') << id << ","
                    << std::dec
                    << position << "," << speed << "," << torque << "\n";
            }
        }
    }

    ofs.close();
}

void Task::checkUserInput()
{
    if (kbhit())
    {
        char input = getchar();
        if (input == 'q')
            state = Pause;
        else if (input == 'e')
            state = Terminate;
        else if (input == 'r')
            state = Resume;
    }
}

void Task::initializeMotorCounts(std::map<std::string, int> &motor_count_per_port)
{
    // 먼저 motor_count_per_port의 값을 0으로 초기화합니다.
    for (const auto &socket_pair : canUtils.sockets)
    {
        motor_count_per_port[socket_pair.first] = 0;
    }

    // tmotors에 대한 카운트 증가
    for (const auto &motor_pair : tmotors)
    {
        const auto &interface_name = motor_pair.second->interFaceName;
        motor_count_per_port[interface_name]++;
    }

    // maxonMotors에 대한 카운트 증가
    for (const auto &motor_pair : maxonMotors)
    {
        const auto &interface_name = motor_pair.second->interFaceName;
        motor_count_per_port[interface_name]++;
    }
}

void Task::handleSocketRead(int socket_descriptor, int motor_count, queue<can_frame> &recieveBuffer)
{

    struct can_frame readFrame[NUM_FRAMES];

    ssize_t bytesRead = read(socket_descriptor, &readFrame, sizeof(can_frame) * NUM_FRAMES);
    if (bytesRead == -1)
    {
        std::cerr << "Failed to read from socket." << std::endl;
        return;
    }

    int numFramesRead = bytesRead / sizeof(can_frame); // 중복 연산 최소화
    for (int i = 0; i < numFramesRead; ++i)
    {
        recieveBuffer.push(readFrame[i]); // Push to the buffer without locking
    }
}

void Task::RecieveLoopTask(queue<can_frame> &recieveBuffer)
{
    chrono::system_clock::time_point external = std::chrono::system_clock::now();

    std::map<std::string, int> motor_count_per_port;

    // 소켓 옵션 및 모터 카운트 초기화
    for (const auto &socket_pair : canUtils.sockets)
    {
        int socket_descriptor = socket_pair.second;
        if (set_socket_timeout(socket_descriptor, 0, 5000) < 0) // 5ms
        {
            std::cerr << "Failed to set socket options for interface: " << socket_pair.first << std::endl;
            return;
        }
        motor_count_per_port[socket_pair.first] = 0;
    }

    initializeMotorCounts(motor_count_per_port);

    while (state.load() != Terminate)
    {
        checkUserInput();

        if (state.load() == Pause)
        {
            continue;
        }

        chrono::system_clock::time_point internal = std::chrono::system_clock::now();
        chrono::milliseconds elapsed_time = chrono::duration_cast<chrono::milliseconds>(internal - external);
        if (elapsed_time.count() >= TIME_THRESHOLD_MS)
        {
            external = std::chrono::system_clock::now();
            for (const auto &socket_pair : canUtils.sockets)
            {
                int socket_descriptor = socket_pair.second;
                int motor_count = motor_count_per_port[socket_pair.first];
                for (int i = 0; i < motor_count; ++i)
                {
                    handleSocketRead(socket_descriptor, motor_count, recieveBuffer);
                }
            }
        }
    }

    parse_and_save_to_csv("TuningData/DrumData_out.txt");
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for SensorLoop
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::SensorLoopTask(queue<int> &sensorBuffer)
{

    USBIO_DI_ReadValue(DevNum, &DIValue);

    for (int i = 0; i < 8; i++)
    {
        if ((DIValue >> i) & 1)
        {
            printf("Ch%2d DI  On   \n", i);
            state = Pause;
            return;
        }
    }
}

void Task::ActivateSensor()
{
    printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
    res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

    if (res)
    {
        printf("open Device failed! Erro : 0x%x\r\n", res);
    }

    printf("Demo usbio_di DevNum = %d\n", DevNum);

    USBIO_ModuleName(DevNum, module_name);

    USBIO_GetDITotal(DevNum, &total_di);
    printf("%s DI number: %d\n\n", module_name, total_di);

    USBIO_DI_SetDigitalFilterWidth(DevNum, 50);
}

void Task::DeactivateSensor()
{
    res = USBIO_CloseDevice(DevNum);

    if (res)
    {
        printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, res);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for Homing Mode
//////////////////////////////////////////////////////////////////////////////////////////////////

bool Task::CheckCurrentPosition(std::shared_ptr<TMotor> motor)
{
    struct can_frame frame;
    fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
    canUtils.set_all_sockets_timeout(0, 5000 /*5ms*/);

    canUtils.clear_all_can_buffers();
    auto interface_name = motor->interFaceName;

    canUtils.restart_all_can_ports();
    if (canUtils.sockets.find(interface_name) != canUtils.sockets.end())
    {
        int socket_descriptor = canUtils.sockets.at(interface_name);
        ssize_t bytesWritten = write(socket_descriptor, &frame, sizeof(can_frame));

        if (bytesWritten == -1)
        {
            cerr << "Failed to write to socket for motor: " << motor->nodeId << " (" << interface_name << ")" << endl;
            cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << endl;
            return false;
        }

        ssize_t bytesRead = read(socket_descriptor, &frame, sizeof(can_frame));
        if (bytesRead == -1)
        {
            cerr << "Failed to read from socket for motor: " << motor->nodeId << " (" << interface_name << ")" << endl;
            cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << endl;
            return false;
        }

        std::tuple<int, float, float, float> parsedData = TParser.parseRecieveCommand(*motor, &frame);
        motor->currentPos = std::get<1>(parsedData);
        cout << "Current Position of [" << std::hex << motor->nodeId << std::dec << "] : " << motor->currentPos << endl;
        return true;
    }
    else
    {
        cerr << "Socket not found for interface: " << interface_name << " (" << motor->nodeId << ")" << endl;
        return false;
    }
}

bool Task::CheckAllMotorsCurrentPosition()
{
    bool allMotorsChecked = true;
    for (const auto &motor_pair : tmotors)
    {
        std::shared_ptr<TMotor> motor = motor_pair.second;
        cout << "Checking position for motor: " << motor_pair.first << endl;
        bool motorChecked = CheckCurrentPosition(motor);
        if (!motorChecked)
        {
            cerr << "Failed to check position for motor: " << motor_pair.first << endl;
            allMotorsChecked = false;
        }
    }
    return allMotorsChecked;
}

void Task::CheckCurrentPosition()
{

    struct can_frame frameToProcess;
    struct can_frame frameToRecieve;

    for (const auto &socketPair : canUtils.sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 5000) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

    for (auto &motor_pair : tmotors)
    {

        canUtils.clear_all_can_buffers();
        std::shared_ptr<TMotor> &motor = motor_pair.second;
        auto interface_name = motor->interFaceName;

        // 상태 확인
        fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForControlMode());
        if (canUtils.sockets.find(interface_name) != canUtils.sockets.end())
        {
            int socket_descriptor = canUtils.sockets.at(interface_name);
            ssize_t bytesWritten = write(socket_descriptor, &frameToProcess, sizeof(can_frame));

            if (bytesWritten == -1)
            {
                std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            }

            usleep(5000);

            ssize_t bytesRead = read(socket_descriptor, &frameToRecieve, sizeof(can_frame));

            if (bytesRead == -1)
            {
                std::cerr << "Failed to read to socket for interface: " << interface_name << std::endl;
                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            }

            std::tuple<int, float, float, float> parsedData = TParser.parseRecieveCommand(*motor, &frameToRecieve);

            c_MotorAngle[motor_mapping[motor_pair.first]] = std::get<1>(parsedData);
            motor->currentPos = std::get<1>(parsedData);

            cout << "Current Position of "
                 << "[" << motor_pair.first << "] : " << motor->currentPos << endl;
        }
        else
        {
            std::cerr << "Socket not found for interface: " << interface_name << std::endl;
        }
    }
}

void Task::SendCommandToMotor(std::shared_ptr<TMotor> &motor, struct can_frame &frame, const std::string &motorName)
{
    auto interface_name = motor->interFaceName;
    if (canUtils.sockets.find(interface_name) != canUtils.sockets.end())
    {
        int socket_descriptor = canUtils.sockets.at(interface_name);

        // 명령을 소켓으로 전송합니다.
        ssize_t bytesWritten = write(socket_descriptor, &frame, sizeof(struct can_frame));
        if (bytesWritten == -1)
        {
            std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
            return;
        }

        // 명령에 대한 응답을 기다립니다.
        ssize_t bytesRead = read(socket_descriptor, &frame, sizeof(struct can_frame));
        if (bytesRead == -1)
        {
            std::cerr << "Failed to read from socket for interface: " << interface_name << std::endl;
            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
        }
    }
    else
    {
        std::cerr << "Socket not found for interface: " << interface_name << std::endl;
    }
}

bool Task::PromptUserForHoming(const std::string &motorName)
{
    char userResponse;
    std::cout << "Would you like to start homing mode for motor [" << motorName << "]? (y/n): ";
    std::cin >> userResponse;
    return userResponse == 'y';
}

float Task::MoveMotorToSensorLocation(std::shared_ptr<TMotor> &motor, const std::string &motorName, int sensorBit)
{
    float firstPosition = 0.0f, secondPosition = 0.0f;
    bool firstSensorTriggered = false;
    bool secondSensorTriggered = false;

    cout << "Moving " << motorName << " to sensor location.\n";

    while (true)
    {
        USBIO_DI_ReadValue(DevNum, &DIValue);
        bool sensorTriggered = (DIValue >> sensorBit) & 1;

        if (!firstSensorTriggered && sensorTriggered)
        {
            // 첫 번째 센서 인식
            firstSensorTriggered = true;
            CheckCurrentPosition(motor);
            firstPosition = motor->currentPos;
            cout << motorName << " first sensor position: " << firstPosition << endl;
        }
        else if (firstSensorTriggered && !sensorTriggered)
        {
            // 센서 인식 해제
            secondSensorTriggered = true;
            CheckCurrentPosition(motor);
            secondPosition = motor->currentPos;
            cout << motorName << " second sensor position: " << secondPosition << endl;

            break; // while문 탈출
        }

        if (secondSensorTriggered)
            break; // 두 번째 센서 인식 후 반복문 탈출
    }

    // 1번과 2번 위치의 차이의 절반을 저장
    float positionDifference = abs((secondPosition - firstPosition) / 2.0f);
    cout << motorName << " midpoint position: " << positionDifference << endl;

    return positionDifference;
}

void Task::RotateMotor(std::shared_ptr<TMotor> &motor, const std::string &motorName, double direction, double degree, float midpoint)
{
    struct can_frame frameToProcess;
    chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
    TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, motor->currentPos, 0, 150, 1, 0);
    SendCommandToMotor(motor, frameToProcess, motorName);

    // 수정된 부분: 사용자가 입력한 각도를 라디안으로 변환
    const double targetRadian = (degree * M_PI / 180.0 + midpoint) * direction; // 사용자가 입력한 각도를 라디안으로 변환 + midpoint
    int totalSteps = 4000 / 5;                                                  // 4초 동안 5ms 간격으로 나누기

    
    for (int step = 1; step <= totalSteps; ++step)
    {
        while (1)
        {
            chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
            if(chrono::duration_cast<chrono::microseconds>(currentTime - startTime).count() > 5000)
                break;
        }

        startTime = std::chrono::system_clock::now();

        // 5ms마다 목표 위치 계산 및 프레임 전송
        double targetPosition = targetRadian * (static_cast<double>(step) / totalSteps) + motor->currentPos;
        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, targetPosition, 0, 250, 2.5, 0);
        SendCommandToMotor(motor, frameToProcess, motorName);

        startTime = std::chrono::system_clock::now();
    }
}

struct MotorSettings
{
    double direction;
    int sensorBit;
};

void Task::SetHome()
{
    struct can_frame frameToProcess;
    Task::ActivateSensor();

    // 각 모터의 방향 및 센서 비트 설정
    std::map<std::string, MotorSettings> motorSettings = {
        {"R_arm1", {1.0, 0}}, // 예시: R_arm1 모터의 방향 1.0, 센서 비트 0
        {"L_arm1", {1.0, 1}},
        {"R_arm2", {1.0, 0}},
        {"R_arm3", {1.0, 0}},
        {"L_arm2", {-1.0, 0}},
        {"L_arm3", {-1.0, 2}},
        // ... 다른 모터 설정 ...
    };

    for (const auto &socketPair : canUtils.sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 5, 0) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

    for (auto &motor_pair : tmotors)
    {
        // 허리는 home 안잡음
        if (motor_pair.first == "waist")
            continue;

        std::shared_ptr<TMotor> &motor = motor_pair.second;
        MotorSettings &settings = motorSettings[motor_pair.first];
        if (!PromptUserForHoming(motor_pair.first)) // 사용자에게 홈 설정을 묻는 함수
            continue;

        // arm2 모터는 -30도, 나머지 모터는 +90도에 센서 위치함.
        double initialDirection = (motor_pair.first == "L_arm2" || motor_pair.first == "R_arm2") ? (-0.2) * settings.direction : 0.2 * settings.direction;

        double additionalTorque = (motor_pair.first == "L_arm2" || motor_pair.first == "R_arm2") ? settings.direction * (-1.65) : 0;
        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, 0, initialDirection, 0, 4.5, additionalTorque);
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);

        float midpoint = MoveMotorToSensorLocation(motor, motor_pair.first, settings.sensorBit); // 모터를 센서 위치까지 이동시키는 함수, 센서 비트 전달
                                                                                                 // 모터를 센서 위치까지 이동시키는 함수

        cout << "\nPress Enter to move to Home Position\n";
        getchar();

        double degree = (motor_pair.first == "L_arm2" || motor_pair.first == "R_arm2") ? -30.0 : 90.0;
        midpoint = (motor_pair.first == "L_arm2" || motor_pair.first == "R_arm2") ? -midpoint : midpoint;
        RotateMotor(motor, motor_pair.first, -settings.direction, degree, midpoint);

        cout << "----------------------moved 90 degree (Anti clock wise) --------------------------------- \n";
        // 모터를 멈추는 신호를 보냄
        TParser.parseSendCommand(*motor, &frameToProcess, motor->nodeId, 8, 0, 0, 0, 5, 0);
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);

        //sleep(10);
        // 그 상태에서 setzero 명령을 보냄(현재 position을 0으로 인식)
        fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForZeroing());
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);
        

        // 상태 확인
        fillCanFrameFromInfo(&frameToProcess, motor->getCanFrameForControlMode());
        SendCommandToMotor(motor, frameToProcess, motor_pair.first);

        if (motor_pair.first == "L_arm1" || motor_pair.first == "R_arm1")
        {
            CheckCurrentPosition(motor);
            RotateMotor(motor, motor_pair.first, settings.direction, 90, 0);
        }
        /*
        if(motor_pair.first == "L_arm2" || motor_pair.first == "R_arm2")
        {
            CheckCurrentPosition(motor);
            RotateMotor(motor, motor_pair.first, settings.direction, -45, 0);
        }
        if (motor_pair.first == "L_arm3" || motor_pair.first == "R_arm3")
        {
            RotateMotor(motor, motor_pair.first, settings.direction, 90, 0);
        }
        */
    }

    cout << "All in Home\n";
    Task::DeactivateSensor();
}

void Task::FixMotorPosition()
{
    struct can_frame frame;
    CheckCurrentPosition();
    for (const auto &motorPair : tmotors)
    {
        std::string name = motorPair.first;
        std::shared_ptr<TMotor> motor = motorPair.second;

        TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, motor->currentPos, 0, 250, 1, 0);
        sendAndReceive(canUtils.sockets.at(motor->interFaceName), name, frame,
                       [](const std::string &motorName, bool success)
                       {
                           if (success)
                           {
                               std::cout << "Position fixed for motor [" << motorName << "]." << std::endl;
                           }
                           else
                           {
                               std::cerr << "Failed to fix position for motor [" << motorName << "]." << std::endl;
                           }
                       });
    }
}