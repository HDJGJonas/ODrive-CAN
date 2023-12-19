#include "ODrive.h"
#include <iostream>

#include "ODrive.h"
#include <iostream>
#include <cmath>

#include<unistd.h>

void moveJ(ODrive::ODrive& Hndl, float J1, float J2, float J3, float J4, float J5, float J6)
{
    union
    {
        float f;
        uint32_t u;
    } punning;

    punning.f = 6.0+J1;
    Hndl.SetInputPos(1, punning.u, 0, 0);

    punning.f = J2;
    Hndl.SetInputPos(2, punning.u, 0, 0);

    punning.f = (J3+25)*(-1);
    Hndl.SetInputPos(3, punning.u, 0, 0);

    punning.f = J4;
    Hndl.SetInputPos(4, punning.u, 0, 0);

    punning.f = J5;
    Hndl.SetInputPos(5, punning.u, 0, 0);

    punning.f = J6;
    Hndl.SetInputPos(6, punning.u, 0, 0);
}

void home(ODrive::ODrive& Hndl){
    union
    {
        float f;
        uint32_t u;
    } punning;

    punning.f = 6.0;
    Hndl.SetInputPos(1, punning.u, 0, 0);

    punning.f = 0.0;
    Hndl.SetInputPos(2, punning.u, 0, 0);

    punning.f = 0.0;
    Hndl.SetInputPos(3, punning.u, 0, 0);

    punning.f = 0.0;
    Hndl.SetInputPos(4, punning.u, 0, 0);

    punning.f = 0.0;
    Hndl.SetInputPos(5, punning.u, 0, 0);

    punning.f = 0.0;
    Hndl.SetInputPos(6, punning.u, 0, 0);
}

int main(int argc, char *argv[])
{
    ODrive::ODrive Hndl;
    // home(Hndl);
    // usleep(10 * 1000000);//sleeps for 3 second
    moveJ(Hndl,0,0,0,0,0,0);

    while (true){
    usleep(4 * 1000000);//sleeps for 3 second

    moveJ(Hndl,8.682,7.998,6.366,10.476,-16.155,-6.187);

    usleep(4 * 1000000);//sleeps for 3 second

    moveJ(Hndl,-8.472,8.261,5.897,-10.326,-15.872,6.212);

    usleep(4 * 1000000);//sleeps for 3 second

    moveJ(Hndl,-8.682,1.631,-0.896,-23.791,-8.710,23.585);

    usleep(4 * 1000000);//sleeps for 3 second

    moveJ(Hndl,8.682,1.631,-0.896,-26.208,8.710,26.414);
    }

    return 0;
}
