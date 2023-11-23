#include "CAN.h"
#include <iostream>
#include <map>
#include <thread>
#include <chrono>

struct TimedFrame {
    can_frame frame;
    std::chrono::steady_clock::time_point timestamp;
};

std::map<uint32_t, TimedFrame> latestFrames;

can_frame errorFrame = {0, 0, 0, {0x99, 0x99, 0x99, 0x99, 0, 0, 0, 0}};

ODrive::CAN::CAN() {
    struct sockaddr_can Address;
    struct ifreq IFReq;

    // Socket setup.
    this->SocketFD = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    
    if (this->SocketFD < 0) {
        // Error; throw exception.
        exit(1);
    }

    strcpy(IFReq.ifr_name, "can0"); // Set interface name to can0. Change if needed.
    ioctl(this->SocketFD, SIOCGIFINDEX, &IFReq); // Store interface index in IFReq.

    Address.can_family = AF_CAN;
    Address.can_ifindex = IFReq.ifr_ifindex;

    // Bind address to FD.
    if (bind(this->SocketFD, (struct sockaddr *)&Address, sizeof(Address)) < 0) {
        // Error; throw exception.
        exit(1);
    }

    // Start the read loop in a separate thread.
    std::thread readThread(&ODrive::CAN::ReadLoop, this);
    readThread.detach();
}

ODrive::CAN::~CAN() {
    // Close socket.
    close(this->SocketFD);
}

ssize_t ODrive::CAN::Write(can_frame *Frame) {
    return write(this->SocketFD, Frame, sizeof(can_frame));
}

void ODrive::CAN::ReadLoop() {
    can_frame Frame;
    while (true) {
        ssize_t bytesRead = read(this->SocketFD, &Frame, sizeof(can_frame));

        if (bytesRead == -1) {
            // Handle read error, e.g., print an error message or return an error code.
            continue;
        } else if (bytesRead == sizeof(can_frame)) {
            // Successfully read a frame. Update the map with the new frame and the current time.
            latestFrames[Frame.can_id] = {Frame, std::chrono::steady_clock::now()};
        }
    }
}

ssize_t ODrive::CAN::Read(can_frame *Frame) {
    uint32_t desiredCanId = Frame->can_id;

    if (latestFrames.count(desiredCanId) > 0) {
        // If we have a frame with the desired CAN ID, check its timestamp.
        auto now = std::chrono::steady_clock::now();
        auto frameTime = latestFrames[desiredCanId].timestamp;
        auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - frameTime).count();

        if (age <= 200) {
            // If the frame is not older than 200ms, copy its data to the input frame.
            *Frame = latestFrames[desiredCanId].frame;
            return sizeof(can_frame);
        }
    }

    // If we don't have a frame with the desired CAN ID, or if the frame is too old, return the error frame.
    *Frame = errorFrame;
    return sizeof(can_frame);
}