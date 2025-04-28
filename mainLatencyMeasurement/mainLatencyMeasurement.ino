/*
 * Adaptive Sampling + MQTT Integration
 * Combines FFT-based adaptive sampling (of sketch "optimalSamplingWithFFT.ino") 
 * with sliding window averaging (from sketch windowCode.ino)and MQTT publishing.
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

#include "arduinoFFT.h"    
#include "PubSubClient.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
//#include "secrets.h"

#define INPUT_PIN         7   
#define SAMPLES          64     
#define INITIAL_SAMPLING_FREQ 16000.0   
#define SAMPLE_INTERVAL   100         
#define WINDOW_SIZE       50  
#define MIN_FFT_INTERVAL_MS 5000    
#define ERROR_MARGIN     3
#define PUBLISH_INTERVAL_MS 5000

#define IO_USERNAME  "volpeffervescente"
#define IO_KEY       "aio_qutm04j11gD0qXPnOJksLdovIDPr"

QueueHandle_t xQueue; 

const char* feed = "volpeffervescente/feeds/avg";
const char* ssid = "FASTWEB-25S396_2.4GHz";
const char* password = "Morgana10122015";
const char* mqttServer = "io.adafruit.com";
int port = 1883;
long lastMsg = 0;
String stMac;
char mac[50];
char clientId[50];
const char* acknowledgment = "volpeffervescente/feeds/echoAck";
unsigned long sentTimestamp = 0;

WiFiClient espClient;
PubSubClient client(espClient);


#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

volatile double samplingFrequency = INITIAL_SAMPLING_FREQ;
unsigned long lastFFTTime = 0;
bool bRecomputeFFT = true;

unsigned long lastPublishTime = 0;

// Buffers for double buffering
double bufferA_vReal[SAMPLES];
double bufferA_vImag[SAMPLES];
double bufferB_vReal[SAMPLES];
double bufferB_vImag[SAMPLES];
bool useBufferA = true;
double* vReal = bufferA_vReal;
double* vImag = bufferA_vImag;
TaskHandle_t FFTTaskHandle;

ArduinoFFT<double> FFT(bufferA_vReal, bufferA_vImag, SAMPLES, samplingFrequency);

// Statistical trigger variables
float mean = 0.0f;
float stdDeviation = 0.0f;


//wifi & mqtt
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}
//ok
void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    long r = random(1000);
    sprintf(clientId, "clientId-%ld", r);
    // to connect to adafruit
    if (client.connect(clientId,IO_USERNAME, IO_KEY)) {
    //if (client.connect(clientId)) {
      Serial.print(clientId);
      Serial.println(" connected");
      //client.subscribe(feed);
      client.subscribe(acknowledgment);
    } else {
      Serial.print("failed, rc=");
      Serial.print("STATE: ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
//ok
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String stMessage;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    stMessage += (char)message[i];
  }
  //Serial.println();

  /*if (String(topic) == feed) {
    Serial.print("New Threshold is: ");
    Serial.println(stMessage);
    int threshold = stMessage.toInt();
  }*/
  Serial.print("Payload: ");
  Serial.println(stMessage);
  //latency measurement here:
  if (strcmp(topic, acknowledgment) == 0) {
    Serial.println("-- Topic matches acknowledgment --");
    unsigned long rtt = millis() - sentTimestamp; 
    Serial.print("--Measuring Latency (RTT): ");
    Serial.print(rtt);
    Serial.println("ms");
  }
}

void SensorTask(void *pvParameters) {
  while (1) {
    int rawValue = analogRead(INPUT_PIN);
    Serial.print("Raw ADC Value: ");
    Serial.println(rawValue);
    double* local_vReal = useBufferA ? bufferA_vReal : bufferB_vReal;
    double* local_vImag = useBufferA ? bufferA_vImag : bufferB_vImag;

    float sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
      local_vReal[i] = analogRead(INPUT_PIN);
      local_vImag[i] = 0.0;
      sum += local_vReal[i];
      xQueueSend(xQueue, &local_vReal[i], portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(1000.0 / samplingFrequency));
    }
    float newMean = sum / SAMPLES;
    float newStdDev = 0;
    for (int i = 0; i < SAMPLES; i++) {
      newStdDev += pow(local_vReal[i] - newMean, 2);
    }
    newStdDev = sqrt(newStdDev / SAMPLES);
    
    if ((millis() - lastFFTTime > MIN_FFT_INTERVAL_MS) &&
        (newMean >= mean + stdDeviation * ERROR_MARGIN || 
        newMean <= mean - stdDeviation * ERROR_MARGIN)) {
      vReal = local_vReal;
      vImag = local_vImag;
      useBufferA = !useBufferA;
      bRecomputeFFT = true;
      lastFFTTime = millis();
      FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, samplingFrequency);
      xTaskNotifyGive(FFTTaskHandle);
      Serial.println("[Trigger] Significant signal change detected. FFT recomputation triggered.");
    }
    mean = newMean;
    stdDeviation = newStdDev;
  }
}

void FFTTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    double peak = FFT.majorPeak();
    if (peak > 0) {
      samplingFrequency = constrain(2 * peak, 10.0, 34000.0);
      Serial.print("[FFT] Peak: ");
      Serial.print(peak);
      Serial.print(" Hz, New Sampling Freq: ");
      Serial.println(samplingFrequency);
    }
    bRecomputeFFT = false;  // Reset flag after processing
  }
}

void AverageTask(void *pvParameters) {
  double value;
  double slidingWindow[WINDOW_SIZE] = {0}; // Array to store last WINDOW_SIZE readings
  int pos = 0; // Index for circular buffer
  int count = 0; // Counter to track the number of stored values
  
  while (1) {
   
    if (xQueueReceive(xQueue, &(value), portMAX_DELAY) == pdPASS) {
      slidingWindow[pos] = value;     // insert value
      pos = (pos + 1) % WINDOW_SIZE;  // Circular buffer index
      if (count < WINDOW_SIZE) count++; // Ensure we don't exceed the array size
      // Compute the moving average
      if(pos==0 && millis() - lastPublishTime >= PUBLISH_INTERVAL_MS)    // publish every 5s
      {
        float sum = 0;
        for (int i = 0; i < count; i++) {
          sum += slidingWindow[i];
        }
        float averageValue = sum / count; 
        
        Serial.print("Average value of sensor readings: ");
        Serial.println(averageValue, 6);

        sentTimestamp = millis();
        snprintf (msg, MSG_BUFFER_SIZE, "%0.2f, time: %lu", averageValue, sentTimestamp);
        Serial.print("Publish message: ");
        Serial.println(msg);
        int result = client.publish(feed, msg);

        lastPublishTime = millis();
      }
    }
  }
}

/*
  MQTTTask is responsible for continuously running client.loop()
  to ensure that incoming MQTT messages (such as ACKs) are handled
  without delay. This avoids relying on Arduino's loop() scheduling,
  making the system more reliable and ensuring accurate RTT measurements.
  By using a dedicated FreeRTOS task, MQTT operations do not interfere
  with sensor acquisition or averaging computations.
*/
void MQTTTask(void *pvParameters) {
  while (1) {
    if (!client.connected()) {
      mqttReconnect();
    }
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS); // chiama client.loop() ogni 10ms
  }
}

/*
 +----------------+
| WiFi connected |
+-------+--------+
        |
        v
+----------------+             +----------------+
| SensorTask     |             | MQTTTask        |
| (Read sensors) |             | (Manage MQTT)   |
| Core 0         |             | Core 1          |
+-------+--------+             +--------+--------+
        |                                |
        v                                v
+----------------+             +-----------------------+
| AverageTask    |             | client.loop()          |
| (Compute avg)  |             | (Handle incoming ACKs) |
| Core 0         |             | (Reconnect if needed)  |
+----------------+             +-----------------------+

*/
void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
  analogSetAttenuation(ADC_11db);

  //wifi
  //randomSeed(analogRead(0));
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  wifiConnect();

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());
  stMac = WiFi.macAddress();
  stMac.replace(":", "_");
  Serial.println(stMac);
  
  client.setServer(mqttServer, port);
  client.setCallback(callback);

  xQueue = xQueueCreate(WINDOW_SIZE, sizeof(double));
  if (xQueue == 0) {
    printf("Failed to create queue= %p\n", xQueue);
  }
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 1, NULL, 0); //0 parameter used to assign core 0 to this task. I also augmented stack dimension
  xTaskCreatePinnedToCore(FFTTask, "FFTTask", 4096, NULL, 2, &FFTTaskHandle, 1);
  xTaskCreatePinnedToCore(AverageTask, "AverageTask", 4096, NULL, 2, NULL, 1); //1 parameter used to assign core 0 to this task
  xTaskCreatePinnedToCore(MQTTTask, "MQTTTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY); // FreeRTOS tasks handle execution
}
