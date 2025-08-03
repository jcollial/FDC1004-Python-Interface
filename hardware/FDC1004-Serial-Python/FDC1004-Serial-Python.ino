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

// -----------------------------------------------------------------------------------------------------------
// Declare/Initialize Variables
// -----------------------------------------------------------------------------------------------------------
// Set constants
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
  WAIT_FOR_COMMAND,
  CONFIG_CAPDAC,
  CONFIG_SAMPLES,
  WAIT_FOR_START_SIGNAL,
  COLLECT_DATA
};

State currentState = WAIT_FOR_COMMAND;

// -----------------------------------------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------------------------------------
int readCommandFromSerial() {
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (Serial.available() >= 3) {
      char start = Serial.read();
      char cmd   = Serial.read();
      char end   = Serial.read();
      if (start == '<' && end == '>') {
        return (int)cmd;
      }
    }
  }
  return -1;
}

int readIntFromSerial() {
  String input = "";
  bool started = false;
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '<') {
        started = true;
        input = "";
      } else if (c == '>' && started) {
        return input.toInt();
      } else if (started) {
        input += c;
      }
    }
  }
  return -1;
}

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

// -----------------------------------------------------------------------------------------------------------
// Setup and Loop
// -----------------------------------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("ESP32 ready");
  tg_timer_init(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN, TIMER_INTERVAL);
}

void loop() {
  switch (currentState) {
    case WAIT_FOR_COMMAND: {
      int cmd = readCommandFromSerial();
      if (cmd != -1) {
        switch (cmd) {
          case 0x00:
            currentState = CONFIG_CAPDAC;
            break;
          case 0x01:
            currentState = CONFIG_SAMPLES;
            break;
          case 0x02:
            currentState = WAIT_FOR_START_SIGNAL;
            break;
        }
      }
      break;
    }

    case CONFIG_CAPDAC: {
      int value = readIntFromSerial();
      if (value != -1) {
        capdac = value;
        myFDC1004.setupSingleMeasurement(measurement, sensor, capdac);
        Serial.print("O");
      } else {
        Serial.print("F");
      }
      currentState = WAIT_FOR_COMMAND;
      break;
    }

    case CONFIG_SAMPLES: {
      int value = readIntFromSerial();
      if (value != -1) {
        samplesToGet = value;
        Serial.print("O");
      } else {
        Serial.print("F");
      }
      currentState = WAIT_FOR_COMMAND;
      break;
    }

    case WAIT_FOR_START_SIGNAL: {
      unsigned long startTime = millis();
      bool startReceived = false;
      while (millis() - startTime < 2000) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'S') {
            startReceived = true;
            break;
          }
        }
      }
      if (startReceived) {
        Serial.print("O");
        currentState = COLLECT_DATA;
      } else {
        Serial.print("F");
        currentState = WAIT_FOR_COMMAND;
      }
      break;
    }

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

