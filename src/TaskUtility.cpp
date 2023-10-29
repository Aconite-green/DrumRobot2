#include "../include/TaskUtility.hpp"


void fillCanFrameFromInfo(struct can_frame *frame, const CanFrameInfo &info)
{
    frame->can_id = info.can_id;
    frame->can_dlc = info.can_dlc;
    std::copy(info.data.begin(), info.data.end(), frame->data);
}

int set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec)
{
    struct timeval timeout;
    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = timeout_usec;

    if (setsockopt(hsocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        perror("setsockopt failed"); // perror 함수는 실패 원인을 출력해줍니다.
        return -1;                   // 실패 시 에러 코드 반환
    }
    return 0; // 성공 시 0 반환
}

void sendAndReceive(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput)
{
    ssize_t write_status = write(socket, &frame, sizeof(can_frame));
    ssize_t read_status = read(socket, &frame, sizeof(can_frame));
    bool success = write_status > 0 && read_status > 0;

    customOutput(name, success);
}

void sendNotRead(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput)
{
    ssize_t write_status = write(socket, &frame, sizeof(can_frame));
    bool success = write_status > 0;

    customOutput(name, success);
}

void writeAndReadForSync(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    size_t numMaxonMotors,
    std::function<void(const std::string &, bool)> customOutput)
{
    ssize_t write_status = write(socket, &frame, sizeof(can_frame));
    if (write_status <= 0)
    {
        customOutput(name, false);
        return;
    }

    // 응답을 저장할 버퍼
    std::vector<struct can_frame> read_frames(numMaxonMotors);

    bool success = true;
    ssize_t read_status;

    // 모터의 개수만큼 read 실행
    for (size_t i = 0; i < numMaxonMotors; ++i)
    {
        read_status = read(socket, &read_frames[i], sizeof(can_frame));
        if (read_status <= 0)
        {
            success = false;
            break;
        }
    }

    customOutput(name, success);
}
