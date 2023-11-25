#include "ODrive.h"
#include <iostream>

#include "ODrive.h"
#include <iostream>
#include <cmath>

void generateSinusWave(float startingValue1, float startingValue2, float amplitude, float frequency, float* waveform1, float* waveform2, int numSamples) {
    for (int i = 0; i < numSamples; i++) {
        float position1 = startingValue1 + amplitude * std::sin(2 * M_PI * frequency * i / numSamples);
        float position2 = startingValue2 + amplitude * std::sin(2 * M_PI * frequency * i / numSamples);
        waveform1[i] = position1;
        waveform2[i] = position2;
    }
}

int main (int argc, char *argv[]) { 
    ODrive::ODrive Hndl;
    union {
        float f;
        uint32_t u;
    }punning;

    // Generata a sinusoidal waveform
    const int numSamples = 2000;
    float waveform1[numSamples];
    float waveform2[numSamples];
    float startingValue1 = 0.0f;
    float startingValue2 = 0.0f;
    float amplitude = 5.0f;
    float frequency = 1.0f;

    generateSinusWave(startingValue, amplitude, frequency, waveform1, waveform2, numSamples);

    // Use the generated waveforms for motor control

    while (ture) {
        for (int i = 0; i < numSamples; i++) {
            punning.f = waveform1[i];
            Hndl.SetPosition(1, punning.u, 0, 0);
            punning.f = waveform2[i];
            Hndl.SetPosition(2, punning.u, 0, 0);
            std::cout << "Position 1: " << waveform1[i] << " Position 2: " << waveform2[i] << std::endl;
        }
    }

    return 0;
}
