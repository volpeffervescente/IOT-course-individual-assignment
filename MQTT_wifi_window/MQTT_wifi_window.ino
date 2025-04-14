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

    
#include "PubSubClient.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
//#include "secrets.h"

#define INPUT_PIN         7           
#define SAMPLE_INTERVAL   100         
#define WINDOW_SIZE       50      

#define IO_USERNAME  "volpeffervescente"
#define IO_KEY       ""

QueueHandle_t xQueue; 

const char* feed = "volpeffervescente/feeds/avg";
const char* ssid = "Martino";
const char* password = "";
const char* mqttServer = "io.adafruit.com";
int port = 1883;
long lastMsg = 0;
String stMac;
char mac[50];
char clientId[50];

WiFiClient espClient;
PubSubClient client(espClient);


#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

//ok
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
      client.subscribe(feed);
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
  Serial.println();

  if (String(topic) == feed) {
    Serial.print("New Threshold is: ");
    Serial.println(stMessage);
    int threshold = stMessage.toInt();
  }
}

void SensorTask(void *pvParameters) {
  float value;
  while (1) {
    value = analogRead(INPUT_PIN);
    printf("Value: %f\n", value);
    xQueueSend(xQueue, &value, portMAX_DELAY);  // using portMAX_DELAY to wait until the queue is available
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL)); 
  }
}

void AverageTask(void *pvParameters) {
  float sum = 0;
  float value;
  float slidingWindow[WINDOW_SIZE] = {0}; // Array to store last WINDOW_SIZE readings
  int pos = 0; // Index for circular buffer
  int count = 0; // Counter to track the number of stored values
  
  while (1) {
   
    if (xQueueReceive(xQueue, &(value), portMAX_DELAY) == pdPASS) {
      slidingWindow[pos] = value;     // insert value
      pos = (pos + 1) % WINDOW_SIZE;  // Circular buffer index
      if (count < WINDOW_SIZE) count++; // Ensure we don't exceed the array size
      // Compute the moving average
      float sum = 0;
      for (int i = 0; i < count; i++) {
        sum += slidingWindow[i];
      }
      float averageValue = sum / count; 
      
      Serial.print("Average value of sensor readings: ");
      Serial.println(averageValue, 6);
      snprintf (msg, MSG_BUFFER_SIZE, "%lf", averageValue);
      
      if(pos==0)    // publish every 5s
      {
        Serial.print("Publish message: ");
        Serial.println(msg);
        int result = client.publish(feed, msg);
      }
      
      //Serial.printlnmartinaFortuna(result);
    }
  }
 
}


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

  xQueue = xQueueCreate(WINDOW_SIZE, sizeof(float));
  if (xQueue == 0) {
    printf("Failed to create queue= %p\n", xQueue);
  }
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 1, NULL, 0); //0 parameter used to assign core 0 to this task. I also augmented stack dimension
  xTaskCreatePinnedToCore(AverageTask, "AverageTask", 4096, NULL, 2, NULL, 1); //1 parameter used to assign core 0 to this task
}

void loop() {
  delay(10);

  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();
  vTaskDelay(portMAX_DELAY); // FreeRTOS tasks handle execution
}
