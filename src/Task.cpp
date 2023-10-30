#include "../include/Task.hpp"

Task::Task(map<string, shared_ptr<TMotor>> &tmotors_input,
           map<string, shared_ptr<MaxonMotor>> &maxonMotors_input,
           const map<string, int> &sockets_input)
    : tmotors(tmotors_input), 
      maxonMotors(maxonMotors_input),
      sockets(sockets_input)
{
    state = 0;
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
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for DrumRobot PathGenerating
//////////////////////////////////////////////////////////////////////////////////////////////////

string trimWhitespace(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t");
    if (std::string::npos == first)
    {
        return str;
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

vector<double> connect(vector<double> &Q1, vector<double> &Q2, int k, int n)
{
    vector<double> Qi;
    std::vector<double> A, B;
    const double PI = 3.14159265358979;

    // Compute A and B
    for (size_t i = 0; i < Q1.size(); ++i)
    {
        A.push_back(0.5 * (Q1[i] - Q2[i]));
        B.push_back(0.5 * (Q1[i] + Q2[i]));
    }

    // Compute Qi using the provided formula
    for (size_t i = 0; i < Q1.size(); ++i)
    {
        double val = A[i] * cos(PI * k / n) + B[i];
        Qi.push_back(val);
    }

    return Qi;
}

vector<double> IKfun(vector<double> &P1, vector<double> &P2, vector<double> &R, double s, double z0)
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
        Qf[i] = Q[i][index_theta0_med];
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

    string score_path = "../include/codeConfession.txt";

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
}

void Task::GetReadyArr()
{
    vector<double> Q0(7, 0);
    vector<vector<double>> q_ready;

    //// 준비자세 배열 생성

    int n = 800;
    for (int k = 0; k < n; ++k)
    {
        c_MotorAngle = connect(Q0, standby, k, n);
        q_ready.push_back(c_MotorAngle);

        int j = 0; // motor num
        for (auto &entry : tmotors)
        {
            std::shared_ptr<TMotor> &motor = entry.second;
            float p_des = c_MotorAngle[j];
            TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 13.46, 0.46, 0);
            sendBuffer.push(frame);

            j++;
        }
        // cout << "\n";
    }
}

void Task::PathLoopTask(queue<can_frame> &sendBuffer)
{
    double p_R = 0; // 오른손 이전 악기 유무
    double p_L = 0; // 왼손 이전 악기 유무
    double c_R = 0; // 오른손 현재 악기 유무
    double c_L = 0; // 왼손 현재 악기 유무재

    /*
    vector<double> P1 = {0.265, -0.391, -0.039837};	 // RightArm Standby
    vector<double> P2 = {-0.265, -0.391, -0.039837}; // LeftArm Standby
    int n_inst = 10;

    vector<double> R = {0.368, 0.414, 0.368, 0.414};
    double s = 0.530;
    double z0 = 0.000;
    */
    vector<double> P1 = {0.3, -0.45, -0.0866};  // RightArm Standby
    vector<double> P2 = {-0.3, -0.45, -0.0866}; // LeftArm Standby
    int n_inst = 10;

    vector<double> R = {0.500, 0.400, 0.500, 0.400};
    double s = 0.600;
    double z0 = 0.000;
    vector<vector<double>> Q(2, vector<double>(7, 0));
    vector<vector<double>> q;

    for (long unsigned int i = 0; i < RF.size(); i++)
    {
        c_R = 0;
        c_L = 0;

        for (int j = 0; j < n_inst; ++j)
        {
            if (RA[i][j] != 0)
            {
                P1 = right_inst[j];
                c_R = 1;
            }
            if (LA[i][j] != 0)
            {
                P2 = left_inst[j];
                c_L = 1;
            }
        }

        int Time = 0;
        clock_t start = clock();

        if (c_R == 0 && c_L == 0)
        { // 왼손 & 오른손 안침
            Q[0] = c_MotorAngle;
            if (p_R == 1)
            {
                Q[0][4] = Q[0][4] + M_PI / 18;
            }
            if (p_L == 1)
            {
                Q[0][6] = Q[0][6] + M_PI / 18;
            }
            Q[1] = Q[0];
        }
        else
        {
            Q[0] = IKfun(P1, P2, R, s, z0);
            Q[1] = Q[0];
            if (c_R == 0)
            { // 왼손만 침
                Q[0][4] = Q[0][4] + M_PI / 18;
                Q[1][4] = Q[1][4] + M_PI / 18;
                Q[0][6] = Q[0][6] + M_PI / 6;
            }
            if (c_L == 0)
            { // 오른손만 침
                Q[0][4] = Q[0][4] + M_PI / 6;
                Q[0][6] = Q[0][6] + M_PI / 18;
                Q[1][6] = Q[1][6] + M_PI / 18;
            }
            else
            { // 왼손 & 오른손 침
                Q[0][4] = Q[0][4] + M_PI / 6;
                Q[0][6] = Q[0][6] + M_PI / 6;
            }
        }

        p_R = c_R;
        p_L = c_L;

        vector<double> Qi;
        double timest = time_arr[i] / 2;
        int n = round(timest / 0.005);
        for (int k = 0; k < n; ++k)
        {
            Qi = connect(c_MotorAngle, Q[0], k, n);
            q.push_back(Qi);

            int j = 0; // motor num
            for (auto &entry : tmotors)
            {
                std::shared_ptr<TMotor> &motor = entry.second;
                float p_des = Qi[j];
                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 13.46, 0.46, 0);
                sendBuffer.push(frame);

                j++;
            }
            // cout << "\n";
        }
        for (int k = 0; k < n; ++k)
        {
            Qi = connect(Q[0], Q[1], k, n);
            q.push_back(Qi);

            int j = 0; // motor num
            for (auto &entry : tmotors)
            {
                std::shared_ptr<TMotor> &motor = entry.second;
                float p_des = Qi[j];
                TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 13.46, 0.46, 0);
                sendBuffer.push(frame);

                j++;
            }
            // cout << "\n";
        }

        c_MotorAngle = Qi;

        Time += ((int)clock() - start) / (CLOCKS_PER_SEC / 1000);
        cout << "TIME : " << Time << "ms\n";
    }

    vector<double> Q0(7, 0);
    vector<vector<double>> q_finish;

    //// 끝나는자세 배열 생성
    vector<double> Qi;
    int n = 800;
    for (int k = 0; k < n; ++k)
    {
        Qi = connect(c_MotorAngle, Q0, k, n);
        q_finish.push_back(Qi);

        int j = 0; // motor num
        for (auto &entry : tmotors)
        {
            std::shared_ptr<TMotor> &motor = entry.second;
            float p_des = Qi[j];
            TParser.parseSendCommand(*motor, &frame, motor->nodeId, 8, p_des, 0, 13.46, 0.46, 0);
            sendBuffer.push(frame);

            j++;
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

        if (sendBuffer.size() <= 10)
        {
        }

        clock_t internal = clock();
        double elapsed_time = ((double)(internal - external)) / CLOCKS_PER_SEC * 1000;

        if (elapsed_time >= 5) // 5ms
        {

            external = clock();

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

//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for SensorLoop
//////////////////////////////////////////////////////////////////////////////////////////////////

void Task::SensorLoopTask()
{
    int res, DevNum, i;
	int DeviceID = USB2051_32;
	// int DeviceID = USB2026;
	BYTE BoardID = 2;
	BYTE total_di;
	char module_name[15];
	DWORD DIValue, o_dwDICntValue[USBIO_DI_MAX_CHANNEL];

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

	while (1)
	{
		//printf("Press ESC to exit.\n\n");

		USBIO_DI_ReadValue(DevNum, &DIValue);

		/*
		if (DIValue)
				printf("Ch%2d DI  On   ", 0);
		else
			printf("Ch%2d DI Off   ", 0);
		*/
		
		for (i = 0; i < 10; i++)
		{
			if ((DIValue >> i) & 1)
				printf("Ch%2d DI  On   ", i);
			else
				printf("Ch%2d DI Off   ", i);
			
			if (i % 4 == 3)
				printf("\n");
			
		}		
		
		printf("\n");

		//printf("Each DI channel counter value:\n");
		USBIO_DI_ReadCounterValue(DevNum, o_dwDICntValue);

		/*
		for (i = 0; i < total_di; i++)
		{
			printf("CH%2d  %11u   ", i, o_dwDICntValue[i]);

			if (i % 8 == 7)
				printf("\n");
		}
		*/
	}
	res = USBIO_CloseDevice(DevNum);

	if (res)
	{
		printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, res);
	}
}