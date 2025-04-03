#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define INPUT_PIN         7           // ADC pin
#define SAMPLE_INTERVAL   100         // Sampling every 100ms
#define WINDOW_SIZE       50          // 5s window (50 samples if 100ms per sample -> 50 * 100ms = 5s)

QueueHandle_t xQueue;                 // FreeRTOS Queue to store distance readings

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

      // Calculate the average
      float averageValue = sum / count; 


      Serial.print("Window: ");
      for(int i=0; i<5;i++)
      {
        Serial.print(slidingWindow[i], 3);
        Serial.print(", ");  
      }
      
      Serial.print("Average value of sensor readings: ");
      Serial.println(averageValue, 6);
    }
  }
 
}


void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
  analogSetAttenuation(ADC_11db);
  
  // Create queue for storing distance values
  xQueue = xQueueCreate(WINDOW_SIZE, sizeof(float));
  if (xQueue == 0) {
    printf("Failed to create queue= %p\n", xQueue);
  }
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 1, NULL, 0); //0 parameter used to assign core 0 to this task. I also augmented stack dimension
  xTaskCreatePinnedToCore(AverageTask, "AverageTask", 2048, NULL, 2, NULL, 1); //1 parameter used to assign core 0 to this task
}

void loop() {
  vTaskDelay(portMAX_DELAY); // FreeRTOS tasks handle execution
}
