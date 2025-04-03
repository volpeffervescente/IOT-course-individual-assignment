#include <Arduino.h>
//goal is to measure the maximum freq to which heltec lorawan v3 can execute adc readings
const int adcPin = 7;   
const uint16_t samples = 64; 

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); 
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation (default 0dB)
}

void loop() {
  unsigned long startTime = micros();  

  for (int i = 0; i < samples; i++) {
    analogRead(adcPin);
  }

  unsigned long totalTime = micros() - startTime;
  float avg_time_per_sample = (float)totalTime / samples;
  float samplingFrequency = 1000000.0 / avg_time_per_sample; // Converting in Hz

  Serial.print("Frequency: ");
  Serial.print(samplingFrequency / 1000.0);  // Converting in kHz
  Serial.println(" kHz");
  //intentional no delay 
}

//max freq is 34.10 kHz
