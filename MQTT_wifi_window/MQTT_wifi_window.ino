


// From https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide
// https://pubsubclient.knolleary.net/api
// https://www.upesy.com/blogs/tutorials/best-tutorials-for-esp32-with-arduino-code


//add the window part 




#include <WiFi.h>
#include "PubSubClient.h"

const char* ssid = "Wokwi-GUEST";
const char* password = "";
// const char* mqttServer = "io.adafruit.com";
const char* mqttServer = "test.mosquitto.org";
int port = 1883;
long lastMsg = 0;
String stMac;
char mac[50];
char clientId[50];

WiFiClient espClient;
PubSubClient client(espClient);

const int ledPin = 2;
int potPin = 32;
int val = 0;
int result = 0;
int threshold = 2000;

#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));

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

  pinMode(potPin,INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
}

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    long r = random(1000);
    sprintf(clientId, "clientId-%ld", r);
    // to connect to adafruit
    // if (client.connect(clientId,SECRET_MQTT_USER, SECRET_MQTT_API_KEY)) {
    if (client.connect(clientId)) {
      Serial.print(clientId);
      Serial.println(" connected");
      client.subscribe("avitaletti/feeds/threshold");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

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

  if (String(topic) == "avitaletti/feeds/threshold") {
    Serial.print("New Threshold is: ");
    Serial.println(stMessage);
    threshold = stMessage.toInt();
  }
}

void loop() {
  delay(10);

  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    val = analogRead(potPin);  // read the input pin

    if (val > threshold){
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }
    
    snprintf (msg, MSG_BUFFER_SIZE, "%ld", val);
    Serial.print("Publish message: ");
    Serial.println(msg);
    result = client.publish("avitaletti/feeds/potentiometer", msg);
    // https://io.adafruit.com/avitaletti/feeds/potentiometer
    Serial.println(result);
  }
}
