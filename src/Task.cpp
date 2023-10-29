#include "../include/Task.hpp"

Task::Task(map<string, shared_ptr<TMotor>> &tmotors,
         map<string, shared_ptr<MaxonMotor>> &maxonMotors,
         const map<string, int> &sockets)
    : tmotors(tmotors), maxonMotors(maxonMotors), sockets(sockets), 
      state(::state), sendBuffer(::sendBuffer), recieveBuffer(::recieveBuffer), sensorBuffer(::sensorBuffer) // 이 부분을 추가
{
    // 생성자 본문
}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for Loops Managing
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::operator()()
{
    // Begin Operation
    ActivateControlTask();
    std::string userInput;
    while (true)
    {
        int result = system("clear");
        if (result != 0)
        {
            std::cout << "error during sys function";
        }

        std::cout << "Enter 'run','exit','test': ";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);

        if (userInput == "exit")
        {
            break;
        }
        else if (userInput == "run")
        {
            int result = system("clear");
            if (result != 0)
            {
                std::cout << "error during sys function";
            }

            std::cout << "Performing ...... \n";

            std::thread pathThread(&Task::PathLoopTask, this, std::ref(sendBuffer));
            std::thread sendThread(&Task::SendLoopTask, this, std::ref(sendBuffer));
            std::thread readThread(&Task::RecieveLoopTask, this, std::ref(recieveBuffer));

            pathThread.join();
            sendThread.join();
            readThread.join();

            std::cout << "........End performance \n";
        }
        else if (userInput == "test")
        {
            while (true)
            {

                std::cout << "Enter 'tuning', 'waves', 'exit' :";
                std::cin >> userInput;
                std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);
                if (userInput == "tuning")
                {
                    TuningLoopTask();
                }
                else if (userInput == "waves")
                {
                    std::thread pathThread(&Task::PeriodicMotionTester, this, std::ref(sendBuffer));
                    std::thread sendThread(&Task::SendLoopTask, this, std::ref(sendBuffer));
                    std::thread readThread(&Task::RecieveLoopTask, this, std::ref(recieveBuffer));

                    pathThread.join();
                    sendThread.join();
                    readThread.join();
                }
                else if (userInput == "exit")
                {
                    break;
                }
            }
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
    for (const auto &socketPair : sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 50000) != 0)
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
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
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
            // 상태 확인
            fillCanFrameFromInfo(&frame, motor->getCanFrameForZeroing());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success)
                           {
                               if (success)
                               {
                                   std::cout << "zero set for motor [" << motorName << "]." << std::endl;
                               }
                               else
                               {
                                   std::cerr << "Failed to set zero for motor [" << motorName << "]." << std::endl;
                               }
                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForControlMode());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
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
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
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
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForPosOffset());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });
            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForTorqueOffset());
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            // 제어 모드 설정
            fillCanFrameFromInfo(&frame, motor->getCanFrameForOperational());
            sendNotRead(sockets.at(motor->interFaceName), name, frame,
                        [](const std::string &motorName, bool success) {

                        });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForEnable());
            sendNotRead(sockets.at(motor->interFaceName), name, frame,
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
            writeAndReadForSync(sockets.at(motor->interFaceName), name, frame, maxonMotors.size(),
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
    for (const auto &socketPair : sockets)
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
            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
                           [](const std::string &motorName, bool success) {

                           });

            fillCanFrameFromInfo(&frame, motor->getCanFrameForExit());

            sendAndReceive(sockets.at(motor->interFaceName), name, frame,
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
            sendNotRead(sockets.at(motor->interFaceName), name, frame,
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
            writeAndReadForSync(sockets.at(motor->interFaceName), name, frame, maxonMotors.size(),
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

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for Testing
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::Tuning(float kp, float kd, float sine_t)
{
    // sockets 맵의 모든 항목에 대해 set_socket_timeout 설정
    for (const auto &socketPair : sockets)
    {
        int hsocket = socketPair.second;
        if (set_socket_timeout(hsocket, 0, 50000) != 0)
        {
            // 타임아웃 설정 실패 처리
            std::cerr << "Failed to set socket timeout for " << socketPair.first << std::endl;
        }
    }

    std::stringstream ss;
    std::string fileName;

    ss << std::fixed << std::setprecision(2); // 소수점 둘째 자리까지만
    ss << "kp_" << kp << "_kd_" << kd << "_period_" << sine_t << ".csv";

    // 파일 이름 자동 설정
    std::string folderName = "data";
    std::string baseName = ss.str(); // ss.str()로 stringstream의 내용을 std::string으로 가져옵니다.
    fileName = folderName + "/" + baseName;

    // CSV 파일을 쓰기 모드로 열기
    std::ofstream csvFile(fileName);
    csvFile << "CAN_ID,p_des,p_act,tff_des,tff_act\n"; // CSV 헤더

    struct can_frame frame;

    float sample_time = 0.005;
    int cycles = 2;
    int max_samples = static_cast<int>(sine_t / sample_time);
    float v_des = 0;
    float tff_des = 0;
    float p_act, v_act, tff_act;
    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            for (auto &entry : tmotors)
            {

                std::shared_ptr<TMotor> &motor = entry.second;

                float local_time = std::fmod(time, sine_t);
                float p_des = (1 - cosf(2 * M_PI * local_time / sine_t)) * M_PI / 2;

                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, v_des, kp, kd, tff_des);
                csvFile << "0x" << std::hex << std::setw(4) << std::setfill('0') << motor->nodeId << ',' << std::dec << p_des;

                clock_t external = clock();
                while (1)
                {
                    clock_t internal = clock();
                    double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;
                    if (elapsed_time >= 5)
                    {

                        ssize_t bytesWritten = write(sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));
                        if (bytesWritten == -1)
                        {
                            std::cerr << "Failed to write to socket for interface: " << motor->interFaceName << std::endl;
                            std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                        }
                        ssize_t bytesRead = read(sockets.at(motor->interFaceName), &frame, sizeof(struct can_frame));

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
                            csvFile << ',' << std::dec << p_act << ',' << tff_des << ',' << tff_act << '\n';
                            break;
                        }
                    }
                }
            }
        }
    }
    csvFile.close();
}

void Task::TuningLoopTask()
{
    std::string userInput;
    float kp = 50.0;
    float kd = 1.0;
    float sine_t = 4.0;

    std::stringstream ss;
    std::string fileName;

    ss << std::fixed << std::setprecision(2); // 소수점 둘째 자리까지만
    ss << "kp_" << kp << "_kd_" << kd << "_period_" << sine_t << ".csv";

    // 파일 이름 자동 설정
    std::string folderName = "data";
    std::string baseName = ss.str(); // ss.str()로 stringstream의 내용을 std::string으로 가져옵니다.
    fileName = folderName + "/" + baseName;

    while (true)
    {
        int result = system("clear");
        if (result != 0)
        {
            printf("error using sys function\n");
        }
        std::cout << "Current Kp : " << kp << "\n";
        std::cout << "Current Kd : " << kd << "\n";
        std::cout << "Time for Sine period : " << sine_t << "\n";
        std::cout << "\n\n";
        std::cout << "Enter run, kp, kd, period, exit : \n";
        std::cin >> userInput;
        std::transform(userInput.begin(), userInput.end(), userInput.begin(), ::tolower);
        if (userInput == "run")
        {
            Task::Tuning(kp, kd, sine_t);
        }
        else if (userInput == "kp")
        {
            std::cout << "Current Kp : " << kp << "\n";
            std::cout << "Enter Desired Kp : "
                      << "\n";
            std::cin >> kp;
        }
        else if (userInput == "kd")
        {
            std::cout << "Current Kd : " << kd << "\n";
            std::cout << "Enter Desired Kd : "
                      << "\n";
            std::cin >> kd;
        }
        else if (userInput == "period")
        {
            std::cout << "Current Time for Sine period : " << sine_t << "\n";
            std::cout << "Enter Desired Sine period : "
                      << "\n";
            std::cin >> sine_t;
        }
        else if (userInput == "exit")
        {
            break;
        }
    }
}

void Task::PeriodicMotionTester(queue<can_frame> &sendBuffer)
{
    std::map<std::string, std::pair<float, float>> motor_configurations = {
        {"1_waist", {8, Tdegree_180}},
        {"2_R_arm1", {8, Tdegree_180}},
        {"3_L_arm1", {8, Tdegree_180}},
        {"4_R_arm2", {8, Tdegree_180}},
        {"5_R_arm3", {8, Tdegree_180}},
        {"6_L_arm2", {8, Tdegree_180}},
        {"7_L_arm3", {8, Tdegree_180}},
        {"a_maxon", {8, Mdegree_180}},
        {"b_maxon", {8, Mdegree_180}}};

    struct can_frame frame;
    if ((tmotors.size() + maxonMotors.size()) != motor_configurations.size())
    {
        std::cerr << "Error: The number of motors does not match the number of total_times entries.\n";
        return;
    }

    float sample_time = 0.005;
    int cycles = 5;
    float max_time = std::max_element(motor_configurations.begin(), motor_configurations.end(),
                                      [](const auto &a, const auto &b)
                                      {
                                          return a.second.first < b.second.first;
                                      })
                         ->second.first;

    int max_samples = static_cast<int>(max_time / sample_time);

    for (int cycle = 0; cycle < cycles; cycle++)
    {
        for (int i = 0; i < max_samples; i++)
        {
            float time = i * sample_time;

            std::unique_lock<std::mutex> lock(sendMutex);
            for (auto &entry : tmotors)
            {
                const std::string &motor_name = entry.first;
                std::shared_ptr<TMotor> &motor = entry.second;

                auto config_itr = motor_configurations.find(motor_name);
                if (config_itr == motor_configurations.end())
                {
                    std::cerr << "Error: Configuration for motor " << motor_name << " not found.\n";
                    continue;
                }

                float period = config_itr->second.first;
                float amplitude = config_itr->second.second;
                float local_time = std::fmod(time, period);

                float common_term = 2 * M_PI * local_time / period;
                float p_des = (1 - cosf(common_term)) / 2 * amplitude;

                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 50, 1, 0);
                sendBuffer.push(frame);
            }

            if (!maxonMotors.empty())
            {
                for (auto &entry : maxonMotors)
                {
                    const std::string &motor_name = entry.first;
                    std::shared_ptr<MaxonMotor> &motor = entry.second;

                    auto config_itr = motor_configurations.find(motor_name);
                    if (config_itr == motor_configurations.end())
                    {
                        std::cerr << "Error: Configuration for motor " << motor_name << " not found.\n";
                        continue;
                    }

                    float period = config_itr->second.first;
                    float amplitude = config_itr->second.second;
                    float local_time = std::fmod(time, period);

                    float common_term = 2 * M_PI * local_time / period;
                    int p_des = (1 - cosf(common_term)) / 2 * amplitude;

                    MParser.parseSendCommand(*motor, &frame, p_des);
                    sendBuffer.push(frame);
                }
                MParser.makeSync(&frame);
                sendBuffer.push(frame);
            }

            lock.unlock();
            sendCV.notify_one();
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for DrumRobot PathGenerating
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::PathLoopTask(queue<can_frame> &sendBuffer)
{
    //
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
                std::cerr << "Failed to write to socket for interface: " << interface_name << std::endl;
                std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
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
    clock_t external = clock();

    while (state.load() != Terminate)
    {
        if (state.load() == Pause)
        {
            continue;
        }

        clock_t internal = clock();
        double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;

        if (elapsed_time >= 5) // 5ms
        {
            std::unique_lock<std::mutex> lock(sendMutex); // Move lock inside the loop

            external = clock();

            if (sendBuffer.empty())
            {
                sendCV.wait(lock); // 조건 변수를 이용하여 대기
            }

            Task::writeToSocket(tmotors, sendBuffer, sockets);

            if (!maxonMotors.empty())
            {
                Task::writeToSocket(maxonMotors, sendBuffer, sockets);

                // sync 신호 전송
                frameToProcess = sendBuffer.front();
                sendBuffer.pop();
                auto it = sockets.find(maxonMotors.begin()->second->interFaceName);

                if (it != sockets.end())
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

            lock.unlock(); // sendMutex 락 해제
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for RecieveTask
//////////////////////////////////////////////////////////////////////////////////////////////////

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
    for (const auto &socket_pair : sockets)
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
    // 스레드 로컬 저장소 사용 (C++11 이상)
    thread_local std::vector<can_frame> readFrame(NUM_FRAMES);

    ssize_t bytesRead = read(socket_descriptor, readFrame.data(), sizeof(can_frame) * NUM_FRAMES);
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
    clock_t external = clock();

    std::map<std::string, int> motor_count_per_port;

    // 소켓 옵션 및 모터 카운트 초기화
    for (const auto &socket_pair : sockets)
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

    while (true)
    {
        checkUserInput();

        if (state.load() == Pause)
        {
            continue;
        }

        clock_t internal = clock();
        double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;
        if (elapsed_time >= TIME_THRESHOLD_MS)
        {
            external = clock();
            for (const auto &socket_pair : sockets)
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
}
