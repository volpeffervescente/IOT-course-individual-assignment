#include <Arduino.h>

// Define the DAC output pin
const int dacPin = 25;  // DAC1 (GPIO 25)

const int amplitude1 = 2;
const int amplitude2 = 4;
const int offset = 128;
const float signalFrequency1 = 3.0; 
const float signalFrequency2 = 5.0; 

const int samplingFrequencyDAC = 100;
const int samples = 128;

void setup() {
  
   Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
   // Initialize DAC pin (GPIO 25)
  dacWrite(dacPin, 0);
} 


void loop() {
  for (int i = 0; i < samples; i++) {
    float t = (float)i / samplingFrequencyDAC; 

    int sineValue = (amplitude1 * sin(2.0 * PI * signalFrequency1 * t)) + (amplitude2 * sin(2.0 * PI * signalFrequency2 * t)) + offset;

    dacWrite(dacPin, sineValue);
    Serial.print("dac:");  
    Serial.println(sineValue);
    delayMicroseconds(1000000/ samplingFrequencyDAC);
  }
}
