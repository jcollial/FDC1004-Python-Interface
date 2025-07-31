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
  the MIT license found in the LICENSE file in the root directory of this
  source tree.

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

int samplesSent = 0;
int pcCommand = -1;
const char *pcStringToSend = "O";

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
// Functions
// -----------------------------------------------------------------------------------------------------------
void set_capdac() {
  int timeout = 0;
  Serial.write(0x00); // Echo configuration command received from serial port
  while (timeout < 200) {
    if (Serial.available() > 0) {
      capdac = Serial.parseInt();
      Serial.print("O");
      break;
    }
    delay(10);
    timeout++;
  }

  if (timeout >= 200) {
    Serial.print("F");
  }

  return;
}

void setSamplesToGet() {
  int timeout = 0;
  Serial.write(0x01); // Echo configuration command received from serial port
  while (timeout < 200) {
    if (Serial.available() > 0) {
      samplesToGet = Serial.parseInt();
      Serial.print("O");
      break;
    }
    delay(10);
    timeout++;
  }

  if (timeout >= 200) {
    Serial.print("F");
  }

  return;

}

void collectData() {
  int timeout = 0;
  uint8_t commandReceived;
  Serial.write(0x02); // Echo configuration command received from serial port
  while (timeout < 200) {
    if (Serial.available() > 0) {
      commandReceived = Serial.read();
      if (commandReceived == 'S') {
        break;
      }
    }
    delay(10);
    timeout++;
  }

  if (timeout < 200) {
    Serial.print("O");
  } else {
    Serial.print("F");
    return;
  }

  timer_start(TIMER_GROUP_0, TIMER_0);                 // Enable Timer with interrupt (Alarm Enable)
  while (samplesSent < samplesToGet) {
    if (timer_interruptFlag == true) {
      timer_interruptFlag = false;

      dataCAP.cap_sens_timestamp = (uint32_t)esp_timer_get_time();
      dataCAP.cap_sens_data = myFDC1004.getRawCapacitance(measurement, rate);
      Serial.write((uint8_t*) &dataCAP, sizeof(dataCAP));
      samplesSent++;

    }
  }
  timer_pause(TIMER_GROUP_0, TIMER_0);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

  samplesSent = 0;

}

// Interrupt timer callback function
static bool IRAM_ATTR timer_group_isr_callback(void *args) {
  timer_interruptFlag = true;
  return false; // return false as we do not need to yield control to other tasks in other cores
}

/*This is a low level implementation of the ESP32 timer. This avoids unnecessary time spent calling wrapping functions from the Arduino core for ESP32*/
static void tg_timer_init(timer_group_t group, timer_idx_t timer, timer_autoreload_t auto_reload, int timer_interval)
{
  /* Select and initialize basic parameters of the timer */
  // the timer_config_t type is a structure used to configure the settings and behavior of a timer of the ESP32
  timer_config_t timer_config;
  timer_config.divider = TIMER_DIVIDER;
  timer_config.counter_dir = TIMER_COUNT_UP;
  timer_config.counter_en = TIMER_PAUSE;
  timer_config.alarm_en = TIMER_ALARM_EN;
  timer_config.auto_reload = auto_reload;

  timer_init(group, timer, &timer_config);

  /* Timer's counter will initially start from value below.
     Also, if auto_reload is set, this value will be automatically reload on alarm */
  timer_set_counter_value(group, timer, 0);

  /* Configure the alarm value and the interrupt on alarm. */
  timer_set_alarm_value(group, timer, timer_interval);
  timer_enable_intr(group, timer);

  timer_isr_callback_add(group, timer, timer_group_isr_callback, NULL, 0);
}

// -----------------------------------------------------------------------------------------------------------

void setup() {
  //start i2c bus on your arduino
  Wire.begin(); 
  
  // Initialize Serial Port 0 (used to send data through serial port)
  Serial.begin(115200);

  // Initialize the ESP32 Timer
  tg_timer_init(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN, TIMER_INTERVAL);

  //set up one measurement channel to measure channel A (CHA)
  //measurement channels hold the settings for what you want to measure
  //channels are the physical pins for measurement on the chip/board
  //read more about it here: 
  myFDC1004.setupSingleMeasurement(measurement, sensor, capdac);
}

// Void loop is attached to core 1 by default
void loop() {
  if (Serial.available() > 0) {
    pcCommand = Serial.read();
  }

  if (pcCommand == 0x00) {
    set_capdac();
    pcCommand = -1;
  }
  else if (pcCommand == 0x01) {
    setSamplesToGet();
    pcCommand = -1;
  }
  else if (pcCommand == 0x02) {
    collectData();
    pcCommand = -1;
  }
}
