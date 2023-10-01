#include "CAN.h"
#include <iostream>

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
}

ODrive::CAN::~CAN() {
    // Close socket.
    close(this->SocketFD);
}

// ssize_t ODrive::CAN::Read(can_frame *Frame) {
//     return read(this->SocketFD, Frame, sizeof(can_frame));
// }

ssize_t ODrive::CAN::Write(can_frame *Frame) {
    return write(this->SocketFD, Frame, sizeof(can_frame));
}

ssize_t ODrive::CAN::Read(can_frame *Frame) {
    ssize_t bytesRead;
    uint32_t desiredCanId = Frame -> can_id;
    int remainingTimeout = 10000000; //microseconds
    
    while (remainingTimeout > 0) {
        bytesRead = read(this->SocketFD, Frame, sizeof(can_frame));
        
        if (bytesRead == -1) {
            // Handle read error, e.g., print an error message or return an error code.
            return -1;
        } else if (bytesRead == sizeof(can_frame) && Frame->can_id == desiredCanId) {
            // Successfully read a frame with the desired CAN ID.
            return bytesRead;
        }
        
        // Update the remaining timeout.
        remainingTimeout -= 10000; // 10,000 microseconds = 0.01 seconds
        
        // Sleep for a short time to avoid busy-waiting.
        usleep(10000); // 10,000 microseconds = 0.01 seconds
    }
    // Return an axis error if timeout is hit axis error is 0x99999999 32bit
    Frame->data[0] = 0x99;
    Frame->data[1] = 0x99;
    Frame->data[2] = 0x99;
    Frame->data[3] = 0x99;

    return 4;
}