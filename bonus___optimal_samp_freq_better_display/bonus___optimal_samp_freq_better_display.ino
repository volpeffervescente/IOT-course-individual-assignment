/*
 * Internet of Things - Algorithms and Services  
 *
 * Simple adatptive sampling program. 
 * The program is divided in three different tasks: 
 * - one for oversamplig
 * - one for calculating the FFT and adapting the frequency
 * - one for displaying the available resources
 * 
 * The program uses a semaphore and tasks notifications to synchronize 
 * the sampling task and the FFT analyzer task. The task notification 
 * ensures that the FFT will not be calculated before collecting a 
 * complete set of samples. The semaphore ensures that the sampling task 
 * is not modifying the samples array while being referenced by the 
 * FFT.compute() function, or that the adaptation task won't change 
 * the sampling frequency in the mid of a samples set collectioning.
 *
 * An attempt at detecting signal changes to determine when to recompute
 * the FFT and change the sampling frequency has been done by following
 * this logic: if the sampled signals changes enough, at the current 
 * sampling rate, there should be an observable change in the signal
 * average, and to tell if the signal changed too much, the standard
 * deviation is beign involved. This approach is not ideal because it
 * considers just the last samples set, meaning that with a gradually
 * changing signal, the FFT recomputation will not be triggered. Another
 * issue is given by the computation of the mean and the standard 
 * deviation themselves, due to the need to compute them exactly after
 * sampling the signal, adding delays and discrepancies between a
 * sampling set and the next one. So this approach should be overall
 * avoided. Other approaches could be based on interrupts, network
 * commands, other statistical methods, or even timers.
 * 
 * Code adjusted by Martina and Leonardo :) 
 */
#include <Arduino.h>                    
#include "arduinoFFT.h"                 // Slow library, other libraries like CMSIS-DSP or ESP-DSP could be faster due to specific ARM optimizations (basically optimized for devices like the ESP32)
#include "esp_dsp.h"                    // ESP32 optimized FFT library
#include <string.h>
// For a connection via I2C to the display using the Arduino Wire include:
#include <Wire.h>               
#include "HT_SSD1306Wire.h"

#define ERROR_MARGIN 3                  // How far from the standard deviation should the new signal average be, to trigger an FFT recomputation? (Currently 3 times the std. dev.)
#define MONITOR_RESOURCES true          // Should the monitoring task be created? 
#define OLED_ENABLE true                // Should the performance by displayed on the integrated OLED display?

const int adcPin = 7;                   // Physical ADC pin used for sampling 
const uint16_t samples = 64;            // Must be a power of 2 - frequency and accuracy dependant - choose based on desired goal

// Initial Sampling Frequency (16 kHz) - oversampling at start
double samplingFrequency = 16000.0f;  

// FFT Object
float fft_input[samples * 2];           // ESP_DSP
double vReal[samples];                  // arduinoFFT - ESP_DSP
double vImag[samples];                  // arduinoFFT
ArduinoFFT<double> FFT(vReal, vImag, samples, samplingFrequency);

// Task Handles - reference to the task functions, helpful for example to notify the FFT task from the sampling task
TaskHandle_t SamplingTaskHandle = NULL;
TaskHandle_t ProcessingTaskHandle = NULL;
TaskHandle_t MonitorTaskHandle = NULL;

// FFT flags
bool bRecomputeFFT = true;              // Flag telling the sampling task if it should also trigger an FFT recalculation, keep true to trigger the initial adaptation
bool bStatisticalTrigger = false;       // Should mean and standard deviation be used for deciding when to trigger the FFT recalculation? Change as you prefer

// Statical variables to trigger the FFT recompute 
float stdDeviation = 0.0f;
float mean = 0.0f; 

// Semaphore for critical section handling - it's recommended to initialize it as a binary semaphore
SemaphoreHandle_t xSemaphore;

// Display
static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

/*
 * Calculate the power of two - gets a value and outputs value^2, used for calculating the standard deviation - not importing a math library saves space!
 */
inline float pow(float value) { return value * value; }

/*
 * Calculate the peak frequency using the arduinoFFT library
 * Execution time: ~1.60ms with 64 samples
 * Execution time: ~27.50ms with 1024 samples
 * Less initial overhead due to accessing directly vReal and vImag, slower calculations - O(nlogn) complexity according to google
 */
double ComputePeakFrequency_Arduino()
{
    // Compute FFT - needed preprocessing -> get sliding window -> go to frequency domain -> numbers to signal power of each frequency f
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    // Return the peak frequency
    return FFT.majorPeak();
}

/*
 * Calculate the peak frequency using the ESP-DSP library - optimized using ARM hardware features and assembly code benath the called functions!
 * API: https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-apis.html#fft
 * Examples: https://github.com/espressif/esp-dsp/tree/master/examples
 * Execution time: ~3.80ms with 64 samples
 * Execution time: ~6.70ms with 1024 samples
 * More initial overhead due to copying vReal and vImag in a different array, faster computations due to ARM hardware features and binary optimization
 * Overhead could be avoided by writing directly into fft_input when sampling
 */
double ComputePeakFrequency_ESP()
{
    // Initialize the ESP-DSP library
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

    // Prepare input data: interleave real input with zeros for imaginary part as required by the esp fft library
    for (int i = 0; i < samples; i++) 
    {
        fft_input[2 * i] = vReal[i];        // Real part
        fft_input[2 * i + 1] = 0.0f;        // Imaginary part
    }

    // FFT.windowing() - Apply window function to the input data
    dsps_wind_hann_f32(fft_input, samples);

    // FFT.compute() - Perform the FFT - bit reversal method as shown in the examples
    dsps_fft2r_fc32(fft_input, samples);
    dsps_bit_rev_fc32(fft_input, samples);

    // FFT.complexToMagnitude() - Scan from high to low frequencies
    int highestIndex = -1;
    float maxMagnitude = 0.0f;
    for (int i = (samples / 2) - 1; i >= 0; i--) 
    {
        float real = fft_input[2 * i];
        float imag = fft_input[2 * i + 1];
        float magnitude = sqrtf(pow(real) + pow(imag));

        // Stop at the first frequency with a magnitude higher than zero - set threshold to 1e-6f, might need some tweaking
        if (magnitude > maxMagnitude)
        {
            maxMagnitude = magnitude;
            highestIndex = i;
        }
    }

    // If no frequency found - return 0
    if (highestIndex == -1) { return 0.0; }

    // Convert ff_input index to the actual frequency
    double peakFrequency = ((double)highestIndex * samplingFrequency) / samples;
    return peakFrequency;
}

/*
 * Initialize the serial communication, setups the ADC pin for sampling, initialize the needed semaphore and the tasks, running only once at device startup
 */
void setup() 
{
    // Init Serial Console
    Serial.begin(115200);
    Serial.println("Starting Adaptive Sampling with FreeRTOS...");

    // Init ADC
    analogReadResolution(10);
    analogSetAttenuation(ADC_11db);

    // Init display
    if (OLED_ENABLE)
    {
        pinMode(Vext,OUTPUT);
        digitalWrite(Vext, LOW);
        display.init();
        display.setFont(ArialMT_Plain_10);
    }

    // Init semaphore
    xSemaphore = xSemaphoreCreateBinary();                                                                      // No end of program - never destroy semaphore
    xSemaphoreGive(xSemaphore);                                                                                 // Give the semaphore initially so one task can take it

    // Init tasks
    xTaskCreatePinnedToCore(SamplingTask, "SamplingTask", 4096, NULL, 1, &SamplingTaskHandle, 0);               // Run on core 0, highest priority of 1
    xTaskCreatePinnedToCore(ProcessingTask, "ProcessingTask", 4096, NULL, 2, &ProcessingTaskHandle, 1);         // Run on core 1, lower priority of 2

    if(MONITOR_RESOURCES)
    {
        xTaskCreatePinnedToCore(MonitorTask, "MonitorTask", 4096, NULL, 3, &MonitorTaskHandle, 1);              // Run on core 1
    }
}

/*
 * A simple sampling task that runs continuously, interrupted only during the FFT computation and the sampling rate adaptation
 * The main issue is avoiding the read and write on the same array of values (the input signal), accessing and modifying vReal and vImag should be done in a critical section
 * Other functionalities of this function are calculating the time needed to sample, and understanding if the FFT must be recomputed with a consequent sampling adaptation
 */
void SamplingTask(void *parameter) 
{
    while (1) 
    {
        // Comment this part to avoid extra delays when performance evaluation is not needed!
        // uint32_t startTime = micros();

        if( xSemaphore == NULL )
        {
            Serial.println("[Semaphore] Semaphore error in Sampling Task");
        }
        else
        {
            if( xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100) ) == pdTRUE ) // Take semaphore - enter critical section
            {
                int sum = 0;                        // Used to compute the average later - if observing weird mean values, might be due to overflow, change value type to long
                for (int i = 0; i < samples; i++)
                {
                    vReal[i] = analogRead(adcPin);  
                    vImag[i] = 0.0;                 // No imaginary component

                    sum += vReal[i];                // Again, just for signal mean computing
                     
                    // Delay based on adaptive frequency
                    delayMicroseconds(1000000 / samplingFrequency);
                }

                xSemaphoreGive(xSemaphore); // Free semaphore - vReal will not change in any way before computing the mean and standard deviation

                // Naive statistical approach to detect signal changes - bad idea, explanation above.
                if(bStatisticalTrigger)
                {
                    float newMean = sum / samples;  // average of signal in order to calculate standard deviation
                    float newStdDeviation = 0;      // Calculate the standard deviation
                    for(int i = 0; i < samples; i++)
                    {
                        newStdDeviation += pow(vReal[i] - newMean);
                    }
                    newStdDeviation = sqrt(newStdDeviation / samples);

                    if(newMean >= mean + stdDeviation * ERROR_MARGIN || newMean <= mean - stdDeviation * ERROR_MARGIN )
                    {
                        bRecomputeFFT = true;
                    }

                    mean = newMean;
                    stdDeviation = newStdDeviation;
                    Serial.print("[Adapting] Signal mean: ");
                    Serial.println(mean);
                    Serial.print("[Adapting] Signal standard deviation: ");
                    Serial.println(stdDeviation);
                }
            }
            else
            {
                Serial.println("[Semaphore] Could not take semaphore in Sampling Task");
            }
        }

        // Comment also this part to avoid performance evaluation delays
        // uint32_t elapsedTime = micros() - startTime;
        // Serial.print("[Sampling] Execution Time: ");
        // Serial.print(elapsedTime / 1000.0);
        // Serial.println(" ms");

        // Notify FFT task that data is ready
        if(bRecomputeFFT)
        {
          bRecomputeFFT = false;
          Serial.println("[FFT recomputation] FFT recompute triggered due to possible signal changes...");
          xTaskNotifyGive(ProcessingTaskHandle);
        }

        // Prevent watchdog reset, but again, we want to avoid useless delays, other solutions?
        vTaskDelay(pdMS_TO_TICKS(1));  // Unfortunately, resetting the wathcdog forcefully will cause issues
    }
}

/*
 * Task that computes the FFT and then adapts the sampling frequency
 * Notice that it's blocking to the sampling frequency to avoid garbage samples (vReal concurrence)
 * This function uses the non optimized arduinoFFT.h library
 */
void ProcessingTask(void *parameter) 
{
    while (1) 
    {
        // Wait until data is available
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        uint32_t startTime = micros();

        if( xSemaphore == NULL )
        {
          Serial.println("[Semaphore] Semaphore error in Processing Task");
             
        }
        else
        {
            if( xSemaphoreTake( xSemaphore, pdMS_TO_TICKS(100) ) == pdTRUE ) // Take semaphore - critical section
            {        
                // Get max frequency
                double peakFrequency = ComputePeakFrequency_Arduino();
                Serial.print("[Processing] Peak Frequency: ");
                Serial.println(peakFrequency, 2);
                if (peakFrequency > 0) 
                {
                    // Nyquist theorem
                    double newSamplingFreq = 2 * peakFrequency;
        
                    // Constrain within limits (10 hz, max frequency found in other assignment)
                    samplingFrequency = constrain(newSamplingFreq, 10.0, 33000.0);
                }

                xSemaphoreGive(xSemaphore); // Free semaphore
            }
            else
            {
                Serial.println("[Semaphore] Could not take semaphore in Proessing Task");
            }
        }

        uint32_t elapsedTime = micros() - startTime;
        Serial.print("[Processing] Execution Time: ");
        Serial.print(elapsedTime / 1000.0);
        Serial.println(" ms");

        vTaskDelay(1);  // Prevent watchdog reset - happening once per second without
    }
}

/*
 * Simply display some informations related to performance and resources on the serial consoles
 * Run once every 2 seconds, display the stack mark (the higher the better), and the available heap space
 * Why this metrics? We are running a resource intensive task (the FFT computing), on a limited memory and computing power device (ESP32) 
 */
void MonitorTask(void *parameter) 
{
    while (1) 
    {
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        uint32_t freeHeap = esp_get_free_heap_size();

        // Serial console print
        Serial.println("------ Performance Metrics ------");
        Serial.print("Free Heap Memory: ");
        Serial.print(freeHeap);
        Serial.println(" bytes");

        Serial.print("Sampling Frequency: ");
        Serial.print(samplingFrequency);
        Serial.println(" Hz");

        Serial.print("Task Stack High Water Mark: ");
        Serial.println(uxHighWaterMark);

        // Display print
        if (OLED_ENABLE)
        {
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.drawString(0, 0, "===Performance Metrics===");
            display.drawString(0, 12, "Free Heap:");
            display.drawString(64, 12, String(freeHeap));
            display.drawString(0, 24, "Samp Freq:");
            display.drawString(64, 24, String(samplingFrequency));
            display.drawString(0, 36, "Stack Mark:");
            display.drawString(64, 36, String(uxHighWaterMark));
            display.drawString(0, 48, "=========================");
            display.display();
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Monitor every 2s
    }
}

/*
 * Loop function, useless due to FreeRTOS managing of the tasks
 */
void loop() 
{
    // Nothing here again
    vTaskDelete(NULL);      // Just remove the loop function from the FreeRTOS schedule
}
