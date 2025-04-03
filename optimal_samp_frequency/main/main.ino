#include <Arduino.h>
#include "arduinoFFT.h"

const int adcPin = 7;
const uint16_t samples = 64;            // Must be a power of 2
const double samplingFrequency = 12.0;  // Define sampling frequency -> initially very high for oversampling

double vReal[samples];
double vImag[samples];

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

unsigned int sampling_period_us;

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  analogReadResolution(10);       // ADC bit resolution
  analogSetAttenuation(ADC_11db); // Set ADC attenuation
  while(!Serial);
  Serial.println("Ready");
}

void loop() {
  acquireSamples();
  
  double majorPeak = FFT.majorPeak();
  
  Serial.println(majorPeak, 6); //Print out what frequency is the most dominant.
  delay(1000); 
}

void acquireSamples() {
  unsigned long startTime = micros();
  for (int i = 0; i < samples; i++) {
    vReal[i] = analogRead(adcPin);
    vImag[i] = 0.0;
    while(micros() - startTime < sampling_period_us){
        //empty loop
      }
      startTime += sampling_period_us;
  }
}
