#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <Arduino.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "LoRaWan_APP.h"

#define SENSOR_PIN        7           // ADC pin
#define SAMPLE_INTERVAL   100         // Sampling every 100ms
#define WINDOW_SIZE       50          // 5s window (50 samples if 100ms per sample -> 50 * 100ms = 5s)



/* FreeRTOS Queue and Task handles */
QueueHandle_t xQueue;
TaskHandle_t TransmitTaskHandle = NULL;
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t AverageTaskHandle = NULL;


/* Heltec Automation LoRaWAN communication example
 *
 * Function:
 * 1. Upload node data to the server using the standard LoRaWAN protocol.
 *  
 * Description:
 * 1. Communicate using LoRaWAN protocol.
 * */

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xFF, 0x3A };
uint8_t appEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x04, 0xB0, 0xA1 };
uint8_t appKey[] = { 0x2C, 0xCF, 0xB0, 0x32, 0x3F, 0x7C, 0xAB, 0xB7, 0xD2, 0x80, 0x1D, 0x43, 0x08, 0xEF, 0x03, 0x77 };

/* ABP para
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;
*/

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868; /*or ACTIVE_REGION*/

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_C;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 30000; 

uint32_t lastSendTime = 0;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port, float avg )
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
    appDataSize = sizeof(float);
    memcpy(appData, &avg, sizeof(float));
}

//if true, next uplink will add MOTE_MAC_DEVICE_TIME_REQ 

void SensorTask(void *pvParameters) {
  float value;
  while (1) {
    value = analogRead(SENSOR_PIN);
    printf("Value: %f\n", value);
    xQueueSend(xQueue, &value, portMAX_DELAY);  // using portMAX_DELAY to wait until the queue is available
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL)); //wait
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

      // Calculate the average
      float averageValue = sum / count; 


      //Serial.print("Window: ");
      //for(int i=0; i<5;i++)
      //{
        //Serial.print(slidingWindow[i], 3);
        //Serial.print(", ");  
      //}
      
      Serial.print("Average value of sensor readings: ");
      Serial.println(averageValue, 6);
      xTaskNotify(TransmitTaskHandle, *(uint32_t*)&averageValue, eSetValueWithOverwrite); // Correctly notify with value
    }
  }
 
}
//INIT → JOIN → SEND → CYCLE → SLEEP → (notification) → SEND → ...

void TransmitTask(void *pvParameters){
  while (1){ 
    switch( deviceState )
    {
      case DEVICE_STATE_INIT:
      {
        #if(LORAWAN_DEVEUI_AUTO)
              LoRaWAN.generateDeveuiByChipID();
        #endif
        LoRaWAN.init(loraWanClass,loraWanRegion);
        //both set join DR and DR when ADR off 
        LoRaWAN.setDefaultDR(3);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
      case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        //send rignìht now the first packet
        deviceState = DEVICE_STATE_SEND;
        break;
      }
      case DEVICE_STATE_SEND:
      {
      
        //Serial.println(".");
        prepareTxFrame( appPort, 0.f );
        LoRaWAN.send();
        Serial.println("sent data");
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
      case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
      case DEVICE_STATE_SLEEP:
      {
        //LoRaWAN.sleep(loraWanClass);
        //break;
        Serial.println("Waiting for average ");
        uint32_t avg;
        if (xTaskNotifyWait(0, 0, &avg, portMAX_DELAY))
        {
          float average = *(float*)&avg;
    
          // Check if at least appTxDutyCycle milliseconds passed
          uint32_t currentTime = millis();
          if ((currentTime - lastSendTime) >= appTxDutyCycle)
          {
            lastSendTime = currentTime;

            prepareTxFrame(appPort, average);
            deviceState = DEVICE_STATE_SEND;
            Serial.println("Sending average");
        }
        else
        {
          uint32_t remaining = appTxDutyCycle - (currentTime - lastSendTime);
          Serial.printf("Waiting %lu ms before next send...\n", remaining);
          vTaskDelay(pdMS_TO_TICKS(remaining));

          lastSendTime = millis();  // Update send time
          prepareTxFrame(appPort, average);
          deviceState = DEVICE_STATE_SEND;
          Serial.println("Sending average...");
        }
      }
      default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
    }
  }
 }
}      

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
  analogSetAttenuation(ADC_11db);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  //Heltec.begin(true, true, true);


  // Create queue for storing distance values
  xQueue = xQueueCreate(WINDOW_SIZE, sizeof(float));
  if (xQueue == 0) {
    printf("Failed to create queue= %p\n", xQueue);
  }
  xTaskCreatePinnedToCore(TransmitTask, "TransmitTask", 8192, NULL, 1, &TransmitTaskHandle, 0);
  
  // Wait until device has joined the network
  while(deviceState == DEVICE_STATE_INIT || deviceState == DEVICE_STATE_JOIN) {
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 2048, NULL, 2, &SensorTaskHandle, 1); 
  xTaskCreatePinnedToCore(AverageTask, "AverageTask", 2048, NULL, 3, &AverageTaskHandle, 1); //1 parameter used to assign core 1 to this task

}

/* Main loop function */
void loop() {
    vTaskDelay(portMAX_DELAY); // The tasks will handle execution
}
