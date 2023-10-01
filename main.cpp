#include "ODrive.h"
#include <iostream>

int main (int argc, char *argv[]) { 
    ODrive::ODrive Hndl;
    std::cout << Hndl.GetHeartbeat(1).AxisCurrentState << std::endl;
    return 0;
}
