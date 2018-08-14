/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  v3HP/v3HP_I2C

  This example shows a method for running the LIDAR-Lite v3 HP while monitoring
  busy status via the MODE pin on the device. Using the MODE pin and the
  selected configuration, this example illustrates a higher speed implementation
  that can be used for shorter distances.

  Open a serial terminal program
  (or the Serial Monitor in the Arduino IDE) to view distance measurements.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND
  LIDAR-Lite MODE (yellow) to Arduino pin 3

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3HP_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/

#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v3HP.h"

LIDARLite_v3HP myLidarLite;

#define FAST_I2C
//#define EXTERNAL_PULLUP

#define MonitorPin    3
#define TriggerPin    2

void setup()
{
    // Initialize Arduino serial port (for display of ASCII output to PC)
    Serial.begin(115200);

    // Initialize Arduino I2C (for communication to LidarLite)
    Wire.begin();
    #ifdef FAST_I2C
        #if ARDUINO >= 157
            Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
        #else
            TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
        #endif
    #endif

    // Configuration '6' sets the mode select bits in the acquisition config
    // register to 01 and configures other settings to enable high rep rates.
    myLidarLite.configure(6);

    #ifdef EXTERNAL_PULLUP
        // Standard configuration of the monitor and trigger pins as
        // shown in the operation manual
        pinMode(MonitorPin, INPUT);
        pinMode(TriggerPin, OUTPUT);
        digitalWrite(TriggerPin, HIGH);
    #else
        // When no external pullup to the trigger pin is installed the
        // monitor pin can be used by itself using an internal pullup in the micro
        pinMode(MonitorPin, INPUT_PULLUP);
    #endif
}

void loop()
{
    uint16_t distance    = 0;
    uint8_t  newDistance = 0;

    // Continuous loop
    while (1)
    {
        newDistance = distanceContinuous(&distance);

        // When there is new distance data, print it to the serial port
        if (newDistance)
        {
            Serial.println(distance);
        }
    }
}

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//
// To use the function, the MODE pin must first be configured for
// status output mode via the acquisition configuration register.
//
// The most recent distance measurement can always be read from
// device registers. Monitoring the BUSY flag via the MODE pin
// can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
uint8_t distanceContinuous(uint16_t * distance)
{
    uint8_t newDistance = 0;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (digitalRead(MonitorPin))
    {
        // Trigger the next range measurement
        myLidarLite.takeRange();

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance();

        // Report to calling function that we have new data
        newDistance = 1;
    }

    return newDistance;
}

