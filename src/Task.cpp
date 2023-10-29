#include "../include/Task.hpp"

Task::Task(std::map<std::string, std::shared_ptr<TMotor>> &tmotors,
           std::map<std::string, std::shared_ptr<MaxonMotor>> &maxonMotors,
           std::atomic<int> &state,
           const std::map<std::string, int> &sockets,
           std::queue<can_frame> &sendBuffer,
           std::queue<can_frame> &recieveBuffer,
           std::queue<int> &sensorBuffer)
    : tmotors(tmotors), maxonMotors(maxonMotors), state(state), sockets(sockets),
      sendBuffer(sendBuffer), recieveBuffer(recieveBuffer), sensorBuffer(sensorBuffer)
{
    // 생성자 본문
}

//////////////////////////////////////////////////////////
// Functions for Loops Managing
//////////////////////////////////////////////////////////

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
                    std::thread pathThread(&Task::PathLoopTask, this, std::ref(sendBuffer));
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

//////////////////////////////////////////////////////////
// Functions for Activate / Deactivate Motors
//////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////
// Functions for Testing
//////////////////////////////////////////////////////////

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
    float kd= 1.0;
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

//////////////////////////////////////////////////////////
// Functions for SendTask
//////////////////////////////////////////////////////////

void Task::SendLoopTask(std::queue<can_frame> &sendBuffer){
    struct can_frame frameToProcess;
    clock_t external = clock();
    while (state.load() != 2)
    {
        
            if (state.load() == 1)
            {
                continue;
            }

        clock_t internal = clock();
        double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;
        if (elapsed_time >= 5) // 5ms
        {
            external = clock();

            for (auto &motor_pair : tmotors)
            {
                auto motor_ptr = motor_pair.second;
                auto interface_name = motor_ptr->interFaceName;

                if (buffer.try_pop(frameToProcess))
                {
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
                else
                {
                    std::cerr << "No CAN frame left in buffer[T]" << std::endl;
                    stop.store(true);
                    break;
                }
            }

            if (!maxonMotors.empty())
            {
                for (auto &motor_pair : maxonMotors)
                {
                    auto motor_ptr = motor_pair.second;
                    auto interface_name = motor_ptr->interFaceName;

                    if (buffer.try_pop(frameToProcess))
                    {
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
                    else
                    {
                        std::cerr << "No CAN frame left in buffer[M]" << std::endl;
                        stop.store(true);
                        break;
                    }
                }
                if (buffer.try_pop(frameToProcess))
                {
                    auto interface_name_for_sync = maxonMotors.begin()->second->interFaceName; // 첫 번째 MaxonMotor의 인터페이스 이름을 가져옵니다.
                    int socket_descriptor_for_sync = sockets.at(interface_name_for_sync);      // 모든 MaxonMotor가 같은 소켓을 사용한다고 했으므로, 아무거나 선택해도 됩니다.                                                                         // 이곳에 sync 신호에 대한 정보를 채워주세요.
                    ssize_t bytesWritten = write(socket_descriptor_for_sync, &frameToProcess, sizeof(struct can_frame));
                    if (bytesWritten == -1)
                    {
                        std::cerr << "Failed to write sync signal to socket for interface: " << interface_name_for_sync << std::endl;
                        std::cerr << "Error: " << strerror(errno) << " (errno: " << errno << ")" << std::endl;
                    }
                }
            }
        }
    }
}

//////////////////////////////////////////////////////////
// Functions for RecieveTask
//////////////////////////////////////////////////////////

