#include "ODrive.h"
#include <iostream>

int main (int argc, char *argv[]) { 
    ODrive::ODrive Hndl;
    union {
        float f;
        uint32_t u;
    }punning;

    punning.u = Hndl.GetEncoderEstimate(1).Position;

    std::cout << punning.f << std::endl;
    return 0;
}
