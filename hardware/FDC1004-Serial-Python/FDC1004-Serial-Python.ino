/*

  This program interfaces an ESP32 with the FDC1004 capacitive sensor to perform
  high-resolution capacitance measurements. It uses a hardware timer to trigger
  data acquisition at fixed intervals (12.5 ms), collects a specified number of
  samples, and sends them over the serial port. The system supports runtime
  configuration of the CAPDAC value and sample count via serial commands.

  Key Features:
  - Low-level ESP32 timer configuration for precise timing
  - Serial command interface for dynamic configuration
  - Efficient data acquisition and transmission using interrupts
  - Modular structure for easy integration with other systems

  Dependencies:
  - Wire.h (I2C communication)
  - FDC1004.h / FDC1004.cpp (sensor driver)
  - ESP32 timer driver (driver/timer.h)

  The source code for the FDC1004.h and the FDC1004.cpp is licensed under
  the MIT license found in the LICENSE file in the same directory of this
  file.

  Author: Jose Guillermo Colli Alfaro <jcollial@uwo.ca>
  Affiliation: The Wearable Biomechatronics Laboratory
  Version: 1.0
  Date: July 31, 2025

*/

#include <Wire.h>
#include "FDC1004.h"
#include "driver/timer.h"

// -----------------------------------------------------------------------------------------------------------
// Defines
// -----------------------------------------------------------------------------------------------------------
#define TIMER_DIVIDER 80  //  Hardware timer clock divider, use 80 to divide the ESP32 80 MHz clock frequency down to 1 MHz resolution
#define TIMER_INTERVAL 12500 // Based on the TIMER_DIVIDER 1 is equivalent to 1 microsecond, so 12500 is equal to 12.5 ms (we will interrupt every 12.5 ms (80 Hz))
#define TIMEOUT_MS 13 // used during dataCollect() to estimate timeout
#define LED 2

// -----------------------------------------------------------------------------------------------------------
// Declare/Initialize Variables
// -----------------------------------------------------------------------------------------------------------
const uint8_t numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing


unsigned long startTime;
unsigned long timeout = 5000; // 5 second

// variables to hold the parsed data
int8_t cmd = -1;
uint32_t cmd_value = 0;
boolean newData = false;
char pcAck = 'F';

// Set constants
int samplesSent = 0;
volatile int samplesToGet = 100;
volatile int capdac = 0;
volatile bool timer_interruptFlag = false;

//settings to configure measurement channel
uint8_t measurement = 1;    //must be 1,2,3,or 4
uint8_t sensor = 1;         //must be 1,2,3,or 4
uint8_t rate = 1;           //1 = 100 Hz, 2 = 200 Hz, 3 = 400 Hz Lower sample rate the higher the resolution

// -----------------------------------------------------------------------------------------------------------
// Data Struct
// -----------------------------------------------------------------------------------------------------------
typedef struct {
  uint32_t cap_sens_timestamp;
  int32_t cap_sens_data;
} cap_sens_reading;

cap_sens_reading dataCAP;

// -----------------------------------------------------------------------------------------------------------
// Object to access library functions
// -----------------------------------------------------------------------------------------------------------
FDC1004 myFDC1004;

// -----------------------------------------------------------------------------------------------------------
// State Machine
// -----------------------------------------------------------------------------------------------------------
enum State {
  WAIT_FOR_SERIAL, // Wait for ESP32 to be ready to accept serial commands
  WAIT_FOR_COMMAND, // Wait for PC command
  COLLECT_DATA
};

State currentState = WAIT_FOR_SERIAL;

// -----------------------------------------------------------------------------------------------------------
// Serial functions
// -----------------------------------------------------------------------------------------------------------
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static uint8_t ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  timeout = 5000;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (millis() - startTime > timeout) {
        resetBuffer(ndx, recvInProgress, receivedChars); // Timeout
        Serial.print("F");
      }

      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        resetBuffer(ndx, recvInProgress, receivedChars);
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
      startTime = millis(); // Start timeout timer
    }
  }
}

void resetBuffer(uint8_t &ndx, bool &recvInProgress, char *buffer) {
  buffer[ndx] = '\0';
  ndx = 0;
  recvInProgress = false;
}

void readCommandFromSerial() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars); // Copy contents of receivedChars into tempChars as strtok in parseData() modifies the original array
    parseData();
    newData = false;
  }
}

void parseData() {      // split the data into its parts
  char * strtokIndx; // this is used by strtok() as an index

  //  Serial.print(tempChars); // FOR DEBUG
  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  cmd = atoi(strtokIndx); // copy it to cmd

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  if (strtokIndx != NULL) {
    cmd_value = atoi(strtokIndx);     // copy it to cmd_value
  }
}

void collectData() {
  timer_start(TIMER_GROUP_0, TIMER_0);
  timeout = samplesToGet * 2 * TIMEOUT_MS; // Wait twice the number of samples to get
  startTime = millis(); // Start timeout timer
  while (samplesSent < samplesToGet) {
    if (millis() - startTime > timeout) {
      // Send end marker maybe?
      break;
    }

    if (timer_interruptFlag) {
      timer_interruptFlag = false;
      dataCAP.cap_sens_timestamp = (uint32_t)esp_timer_get_time();
      dataCAP.cap_sens_data = myFDC1004.getRawCapacitance(measurement, rate);
      Serial.write((uint8_t*)&dataCAP, sizeof(dataCAP));
      samplesSent++;
    }
  }
  resetTimer();
  samplesSent = 0;
}

void sendESP32Rdy() {
  Serial.print("<ESP32 Ready>");
  if (Serial.available() > 0) {
    pcAck = Serial.read();
  }
  if (pcAck == 'O') {
    currentState = WAIT_FOR_COMMAND;
    pcAck = 'F';
  }
}

// -----------------------------------------------------------------------------------------------------------
// Timer functions
// -----------------------------------------------------------------------------------------------------------
void resetTimer() {
  timer_pause(TIMER_GROUP_0, TIMER_0);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
}

static bool IRAM_ATTR timer_group_isr_callback(void *args) {
  timer_interruptFlag = true;
  return false;
}

static void tg_timer_init(timer_group_t group, timer_idx_t timer, timer_autoreload_t auto_reload, int timer_interval) {
  timer_config_t timer_config;
  timer_config.divider = TIMER_DIVIDER;
  timer_config.counter_dir = TIMER_COUNT_UP;
  timer_config.counter_en = TIMER_PAUSE;
  timer_config.alarm_en = TIMER_ALARM_EN;
  timer_config.auto_reload = auto_reload;

  timer_init(group, timer, &timer_config);
  timer_set_counter_value(group, timer, 0);
  timer_set_alarm_value(group, timer, timer_interval);
  timer_enable_intr(group, timer);
  timer_isr_callback_add(group, timer, timer_group_isr_callback, NULL, 0);
}

void send_echo_command(int echo_cmd) {
  uint8_t response[3] = {0x3C, echo_cmd, 0x3E};
  Serial.write((uint8_t*) &response, sizeof(response));
  return;
}


// -----------------------------------------------------------------------------------------------------------
// Setup and Loop
// -----------------------------------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  Serial.begin(115200);
  tg_timer_init(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN, TIMER_INTERVAL);
  // Set LED pin mode
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void loop() {
  switch (currentState) {
    case WAIT_FOR_SERIAL: {
        Serial.print("<ESP32 Ready>");
        if (Serial.available() > 0) {
          pcAck = Serial.read();
        }
        if (pcAck == 'O') {
          currentState = WAIT_FOR_COMMAND;
          pcAck = 'F';
        }
        break;
      }
    case WAIT_FOR_COMMAND: {
        if (Serial.available() > 0) {
          readCommandFromSerial();
        }
        if (cmd != -1) {
          switch (cmd) {
            case 0x00:
              capdac = cmd_value;
              myFDC1004.setupSingleMeasurement(measurement, sensor, capdac);
              Serial.print("<");
              Serial.print(capdac);
              Serial.print(">");
              cmd = -1;
              break;
            case 0x01:
              samplesToGet = cmd_value;
              Serial.print("<");
              Serial.print(samplesToGet);
              Serial.print(">");
              cmd = -1;
              break;
            case 0x02:
              collectData();
              cmd = -1;
              break;
            case 0x03:
              cmd = -1;
              currentState = WAIT_FOR_SERIAL;
          }
        }
        break;
      }

    //    case WAIT_FOR_START_SIGNAL: {
    //        unsigned long startTime = millis();
    //        bool startReceived = false;
    //        while (millis() - startTime < 2000) {
    //          if (Serial.available()) {
    //            char c = Serial.read();
    //            if (c == 'S') {
    //              startReceived = true;
    //              break;
    //            }
    //          }
    //        }
    //        if (startReceived) {
    //          Serial.print("O");
    //          currentState = COLLECT_DATA;
    //        } else {
    //          Serial.print("F");
    //          currentState = WAIT_FOR_COMMAND;
    //        }
    //        break;
    //      }

    case COLLECT_DATA: {
        timer_start(TIMER_GROUP_0, TIMER_0);
        while (samplesSent < samplesToGet) {
          if (timer_interruptFlag) {
            timer_interruptFlag = false;
            dataCAP.cap_sens_timestamp = (uint32_t)esp_timer_get_time();
            dataCAP.cap_sens_data = myFDC1004.getRawCapacitance(measurement, rate);
            Serial.write((uint8_t*)&dataCAP, sizeof(dataCAP));
            samplesSent++;
          }
        }
        resetTimer();
        samplesSent = 0;
        currentState = WAIT_FOR_COMMAND;
        break;
      }
  }
}
