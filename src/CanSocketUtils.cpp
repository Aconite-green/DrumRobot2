#include "../include/CanSocketUtils.hpp"

CanSocketUtils::CanSocketUtils(const std::vector<std::string> &ifnames) : ifnames(ifnames)
{
    list_and_activate_available_can_ports(); // 포트를 자동으로 활성화
    for (const auto &ifname : this->ifnames)
    {
        int hsocket = create_socket(ifname);
        if (hsocket < 0)
        {
            // 에러 처리
            exit(EXIT_FAILURE);
        }
        sockets[ifname] = hsocket;
    }
}

CanSocketUtils::~CanSocketUtils()
{
    for (const auto &kv : sockets)
    {
        int hsocket = kv.second;
        if (hsocket >= 0)
        {
            close(hsocket);
        }
    }
    sockets.clear();
}

int CanSocketUtils::create_socket(const std::string &ifname)
{
    int result;
    struct sockaddr_can addr;
    struct ifreq ifr;

    int localSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 지역 변수로 소켓 생성
    if (localSocket < 0)
    {
        return ERR_SOCKET_CREATE_FAILURE;
    }

    memset(&ifr, 0, sizeof(struct ifreq));
    memset(&addr, 0, sizeof(struct sockaddr_can));

    strcpy(ifr.ifr_name, ifname.c_str());
    result = ioctl(localSocket, SIOCGIFINDEX, &ifr);
    if (result < 0)
    {
        close(localSocket);
        return ERR_SOCKET_CREATE_FAILURE;
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family = AF_CAN;

    if (bind(localSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(localSocket);
        return ERR_SOCKET_CREATE_FAILURE;
    }

    return localSocket; // 생성된 소켓 디스크립터 반환
}

bool CanSocketUtils::is_port_up(const char *port)
{
    char command[50];
    snprintf(command, sizeof(command), "ip link show %s", port);

    FILE *fp = popen(command, "r");
    if (fp == NULL)
    {
        perror("Error opening pipe");
        return false;
    }

    char output[1024];
    if (fgets(output, sizeof(output) - 1, fp) == NULL)
    {
        perror("fgets failed");
        printf("Errno: %d\n", errno); // errno 값을 출력
        pclose(fp);
        return false;
    }

    if (strstr(output, "DOWN"))
    {
        return false;
    }
    else if (strstr(output, "UP"))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void CanSocketUtils::activate_port(const char *port)
{
    char command1[100], command2[100];
    snprintf(command1, sizeof(command1), "sudo ip link set %s type can bitrate 1000000 sample-point 0.850", port);
    snprintf(command2, sizeof(command2), "sudo ip link set %s up", port);

    int ret1 = system(command1);
    int ret2 = system(command2);

    if (ret1 != 0 || ret2 != 0)
    {
        fprintf(stderr, "Failed to activate port: %s\n", port);
        exit(1); // 또는 다른 에러 처리
    }
}

void CanSocketUtils::list_and_activate_available_can_ports()
{
    int portCount = 0; // CAN 포트 수를 세기 위한 변수

    FILE *fp = popen("ip link show | grep can", "r");
    if (fp == nullptr)
    {
        perror("No available CAN port");
        exit(1);
    }

    char output[1024];
    while (fgets(output, sizeof(output) - 1, fp) != nullptr)
    {

        portCount++; // CAN 포트를 발견할 때마다 카운트 증가
        std::string line(output);
        std::istringstream iss(line);
        std::string skip, port;
        iss >> skip >> port;

        // 콜론 제거
        if (!port.empty() && port.back() == ':')
        {
            port.pop_back();
        }

        if (is_port_up(port.c_str()))
        {
            printf("%s is already UP\n", port.c_str());
        }
        else
        {
            printf("%s is DOWN, activating...\n", port.c_str());
            activate_port(port.c_str());
        }
    }

    if (feof(fp) == 0)
    {
        perror("fgets failed");
        printf("Errno: %d\n", errno); // errno 값을 출력
    }

    pclose(fp);

    // CAN 포트를 하나도 발견하지 못했으면 프로그램 종료
    if (portCount == 0)
    {
        printf("No CAN port found. Exiting...\n");
        exit(1);
    }
}

void CanSocketUtils::restart_all_can_ports()
{
    // 먼저 모든 포트를 down 시킵니다.
    for (const auto &port : ifnames)
    {
        down_port(port.c_str());
        printf("Port '%s' is down.\n", port.c_str()); // 포트가 다운됨을 알림

        int socket_fd = sockets[port];
        if (socket_fd >= 0)
        {
            close(socket_fd);   // 기존 소켓을 닫습니다.
            sockets[port] = -1; // 소켓 디스크립터 값을 초기화합니다.
        }
    }

    // 각 포트에 대해 새로운 소켓을 생성하고 디스크립터를 업데이트합니다.
    for (const auto &port : ifnames)
    {
        usleep(100000); // 100ms 대기
        activate_port(port.c_str());
        printf("Activating port '%s'.\n", port.c_str()); // 포트 활성화 중

        int new_socket_fd = create_socket(port);
        if (new_socket_fd < 0)
        {
            // 새로운 소켓 생성에 실패한 경우 처리
            fprintf(stderr, "Failed to create a new socket for port: %s\n", port.c_str());
        }
        else
        {
            sockets[port] = new_socket_fd;                               // 소켓 디스크립터 값을 업데이트합니다.
            printf("New socket created for port '%s'.\n", port.c_str()); // 새 소켓 생성 완료
        }
    }
}

void CanSocketUtils::down_port(const char *port)
{
    char command[100];
    snprintf(command, sizeof(command), "sudo ip link set %s down", port);
    int ret = system(command);
    if (ret != 0)
    {
        fprintf(stderr, "Failed to down port: %s\n", port);
    }
}
