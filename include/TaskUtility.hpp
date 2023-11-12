#ifndef TASKUTILITY_HPP
#define TASKUTILITY_HPP

#include <stdio.h>
#include "../include/CanSocketUtils.hpp"
#include "../include/CommandParser.hpp"
#include "../include/ErrorHandle.hpp"
#include "../include/Motor.hpp"
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
#include <unordered_map>

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for Activate / Deactivate Task
/////////////////////////////////////////////////////////////////////////////////////////////////
void fillCanFrameFromInfo(struct can_frame *frame, const CanFrameInfo &info);
int set_socket_timeout(int hsocket, int timeout_sec, int timeout_usec);
void sendAndReceive(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput);
void sendNotRead(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    std::function<void(const std::string &, bool)> customOutput);
void writeAndReadForSync(
    int socket,
    const std::string &name,
    struct can_frame &frame,
    size_t numMaxonMotors,
    std::function<void(const std::string &, bool)> customOutput);

//////////////////////////////////////////////////////////////////////////////////////////
// Functions for PathGenerating
/////////////////////////////////////////////////////////////////////////////////////////

struct CustomCompare
{
    bool operator()(const std::string &lhs, const std::string &rhs) const
    {
        static std::unordered_map<std::string, int> priority = {
            {"waist", 1},
            {"R_arm1", 2},
            {"L_arm1", 3},
            {"R_arm2", 4},
            {"R_arm3", 5},
            {"L_arm2", 6},
            {"L_arm3", 7}};

        auto lhsPriority = priority.find(lhs);
        auto rhsPriority = priority.find(rhs);

        if (lhsPriority != priority.end() && rhsPriority != priority.end())
        {
            return lhsPriority->second < rhsPriority->second;
        }
        if (lhsPriority != priority.end())
        {
            return true;
        }
        if (rhsPriority != priority.end())
        {
            return false;
        }
        return lhs < rhs;
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// Functions for SendLoopTask
/////////////////////////////////////////////////////////////////////////////////////////

void handleError(ssize_t bytesWritten, const std::string &interface_name);

//////////////////////////////////////////////////////////////////////////////////////////
// Functions for RecieveLoopTask
/////////////////////////////////////////////////////////////////////////////////////////

int kbhit();
#endif // TASKUTILITY_HPP