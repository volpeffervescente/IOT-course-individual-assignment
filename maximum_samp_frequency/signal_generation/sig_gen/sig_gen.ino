#include <Arduino.h>

#define NUM_SIGNALS 5
#define SAMPLING_FREQ 8000
#define NUM_SAMPLES 2048
#define OFFSET 128

const int dacPin = 25; // DAC output pin
float frequencies[NUM_SIGNALS] = { 50.0, 2000.0, 800.0, 600.0, 1000.0 };
float amplitudes[NUM_SIGNALS] = { 8.0, 2.0, 4.0, 1.5, 3.0 };

void setup() 
{
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Ready");
}

void loop() {
    for (int i = 0; i < NUM_SAMPLES; i++) 
    {
        float t = (float)i / SAMPLING_FREQ;

        int value = 0;
        for(int j = 0; j < NUM_SIGNALS; j++)
        {
            value += OFFSET + amplitudes[j] * sin (2.0 * PI * frequencies[j] * t);
        }

        dacWrite(dacPin, value);
        Serial.println(value);
        delayMicroseconds(1000000 / SAMPLING_FREQ);
  }
}
