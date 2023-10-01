#include "ODrive.h"
#include <iostream>

int main (int argc, char *argv[]) { 
    ODrive::ODrive Hndl;

    union {
        float f;
        uint32_t u;
    } punning;
    punning.f = 0.0f;
    
    Hndl.SetInputPos(1,punning.u, 0 ,0);
    return 0;
}
