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


//////////////////////////////////////////////////////////////////////////////////////////
// Functions for Activate / Deactivate Task
/////////////////////////////////////////////////////////////////////////////////////////
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




//////////////////////////////////////////////////////////////////////////////////////////
// Functions for SendLoopTask
/////////////////////////////////////////////////////////////////////////////////////////

void handleError(ssize_t bytesWritten, const std::string& interface_name);

//////////////////////////////////////////////////////////////////////////////////////////
// Functions for RecieveLoopTask
/////////////////////////////////////////////////////////////////////////////////////////

int kbhit();
