/*
 * Adaptive Sampling + MQTT Integration + Deep Sleep
 * Combines FFT-based adaptive sampling with sliding window averaging, MQTT publishing, and deep sleep for energy saving.
 */

#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <WiFiServer.h>
#include <WiFiAP.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>
#include <WiFiGeneric.h>
#include <WiFiScan.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Wire.h>
#include "arduinoFFT.h"

#define INPUT_PIN         7
#define SAMPLES           64
#define INITIAL_SAMPLING_FREQ 16000.0
#define WINDOW_SIZE       50
#define TIME_TO_SLEEP_S   10
#define S_TO_uS_FACTOR    1000000ULL
#define MIN_FFT_INTERVAL_MS 5000
#define ERROR_MARGIN      3

#define IO_USERNAME  "volpeffervescente"
#define IO_KEY       "aio_nsfH8442A02P4LOgpjeouOMDUxKZ"
const char* ssid = "FASTWEB-25S396_2.4GHz";
const char* password = "Morgana10122015";
const char* mqttServer = "io.adafruit.com";
const char* feed = "volpeffervescente/feeds/avg";
const char* acknowledgment = "volpeffervescente/feeds/echoAck";
int port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE 50
char msg[MSG_BUFFER_SIZE];

volatile double samplingFrequency = INITIAL_SAMPLING_FREQ;
unsigned long lastFFTTime = 0;
bool bRecomputeFFT = true;

// Buffers for double buffering
double bufferA_vReal[SAMPLES];
double bufferA_vImag[SAMPLES];
double bufferB_vReal[SAMPLES];
double bufferB_vImag[SAMPLES];
bool useBufferA = true;
double* vReal = bufferA_vReal;
double* vImag = bufferA_vImag;

ArduinoFFT<double> FFT(bufferA_vReal, bufferA_vImag, SAMPLES, samplingFrequency);

unsigned long sentTimestamp = 0;

// Statistical trigger variables
float mean = 0.0f;
float stdDeviation = 0.0f;

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "clientId-" + String(random(1000));
    if (client.connect(clientId.c_str(), IO_USERNAME, IO_KEY)) {
      Serial.println(" connected");
      client.subscribe(acknowledgment);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  if (strcmp(topic, acknowledgment) == 0) {
    unsigned long rtt = millis() - sentTimestamp;
    Serial.print("ACK received. RTT: ");
    Serial.print(rtt);
    Serial.println(" ms");
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
  analogSetAttenuation(ADC_11db);

  bool firstRun = true;
   
  Serial.println("Connecting to WiFi...");
  wifiConnect();
  Serial.println("Connected to WiFi.");
  client.setServer(mqttServer, port);
  client.setCallback(callback);
  mqttReconnect();

  // Sampling phase
  Serial.println("sampling phase...");
  double* local_vReal = useBufferA ? bufferA_vReal : bufferB_vReal;
  double* local_vImag = useBufferA ? bufferA_vImag : bufferB_vImag;

  float sumSamples = 0;
  for (int i = 0; i < SAMPLES; i++) {
    local_vReal[i] = analogRead(INPUT_PIN);
    local_vImag[i] = 0.0;
    sumSamples += local_vReal[i];
    double delayTime = max(1.0, 1000.0 / samplingFrequency);
    delay(delayTime);
  }

  // Check if FFT recomputation needed
  float newMean = sumSamples / SAMPLES;
  float newStdDev = 0;
  for (int i = 0; i < SAMPLES; i++) {
    newStdDev += pow(local_vReal[i] - newMean, 2);
  }
  newStdDev = sqrt(newStdDev / SAMPLES);

  if (firstRun || ((millis() - lastFFTTime > MIN_FFT_INTERVAL_MS) &&
      (newMean >= mean + stdDeviation * ERROR_MARGIN || newMean <= mean - stdDeviation * ERROR_MARGIN))) {
    Serial.println("Starting FFT calculation...");
    vReal = local_vReal;
    vImag = local_vImag;
    useBufferA = !useBufferA;
    FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, samplingFrequency);
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    double peak = FFT.majorPeak();
    if (peak > 0) {
      samplingFrequency = constrain(2 * peak, 10.0, 33000.0);
      Serial.print("[FFT] Peak: ");
      Serial.print(peak);
      Serial.print(" Hz, New Sampling Freq: ");
      Serial.println(samplingFrequency);
    }
    lastFFTTime = millis();
    firstRun = false;
    Serial.println("Finished FFT calculation.");
  }
  mean = newMean;
  stdDeviation = newStdDev;

  // Average and Publish
  double slidingWindow[WINDOW_SIZE] = {0};
  int pos = 0;
  int count = 0;

  for (int i = 0; i < WINDOW_SIZE; i++) {
    slidingWindow[pos] = analogRead(INPUT_PIN);
    pos = (pos + 1) % WINDOW_SIZE;
    if (count < WINDOW_SIZE) count++;
    delay(100);
  }

  double avg = 0;
  for (int i = 0; i < count; i++) avg += slidingWindow[i];
  avg /= count;

  Serial.print("Average value of sensor readings: ");
  Serial.println(avg, 6);

  sentTimestamp = millis();
  snprintf(msg, MSG_BUFFER_SIZE, "%0.2f, time: %lu", avg, sentTimestamp);
  Serial.print("Publish message: ");
  Serial.println(msg);
  Serial.println("Sending data...");
  client.publish(feed, msg);
  Serial.println("Data sent.");
  client.loop();
  
  // Sleep
  Serial.println("Going to sleep now...");
  delay(500); 
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_S * S_TO_uS_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  // Not used with deep sleep
}
