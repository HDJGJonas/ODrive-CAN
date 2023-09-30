#include "ODrive.h"
#include <iostream>

int main (int argc, char *argv[]) { 
    ODrive::ODrive Hndl;
    HeartbeatRequest Heartbeat = Hndl.GetHeartbeat(1);
    std::cout << Heartbeat.AxisError << std::endl;
    return 0;
}
