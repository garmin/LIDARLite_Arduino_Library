/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  v4LED/v4LED_I2C

  This example shows methods for running the LIDAR-Lite v4 LED in various
  modes of operation. To exercise the examples open a serial terminal
  program (or the Serial Monitor in the Arduino IDE) and send ASCII
  characters to trigger the commands. See "loop" function for details.

  Connections:
  LIDAR-Lite 5 VDC   (pin 1) to Arduino 5V
  LIDAR-Lite Ground  (pin 2) to Arduino GND
  LIDAR-Lite I2C SDA (pin 3) to Arduino SDA
  LIDAR-Lite I2C SCL (pin 4) to Arduino SCL

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5V
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information

------------------------------------------------------------------------------*/

#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

LIDARLite_v4LED myLidarLite;

#define FAST_I2C

enum rangeType_T
{
    RANGE_NONE,
    RANGE_SINGLE,
    RANGE_CONTINUOUS
};

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

    // ----------------------------------------------------------------------
    // The LIDAR-Lite v4 LED is strictly a 3.3V system.
    // Care MUST be taken if connecting to a 5V system.
    // Wire.begin() turns on AVR internal pullups on SCL and SDA.
    // In a 5V system such as the Arduino Uno, they are pulled up to 5V
    //     risking damage to the LLv4.
    // To avoid damage, call digitalWrite() to turn off the pullups.
    // External pullups to 3.3V must then be added to the I2C signals
    //     if using an Arduino Uno.
    // On an Arduino Due, there are external 1.5k pullups to 3.3V
    // ----------------------------------------------------------------------
    //digitalWrite(SCL, LOW);
    //digitalWrite(SDA, LOW);
    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    // Optionally configure the LidarLite parameters to lend itself to
    // various modes of operation by altering 'configure' input integer to
    // anything in the range of 0 to 5. See LIDARLite_v4LED.cpp for details.
    // ----------------------------------------------------------------------
    //myLidarLite.configure(0);
    // ----------------------------------------------------------------------
}


void loop()
{
    uint8_t  firstTime = 1;  // print menu at startup

    uint16_t distance;
    uint8_t  newDistance = 0;
    uint8_t  c = '?';
    rangeType_T rangeMode = RANGE_NONE;

    // Continuous loop
    while (1)
    {
        //===================================================================
        // 1) Each time through the loop, look for a serial input character
        //===================================================================
        if ((Serial.available() > 0) || (firstTime))
        {
            firstTime = 0;

            //  read input character ...
            c = (uint8_t) Serial.read();

            // ... and parse
            switch (c)
            {
                case '1':
                    rangeMode = RANGE_SINGLE;
                    break;

                case '2':
                    rangeMode = RANGE_CONTINUOUS;
                    break;

                case '3':
                    rangeMode = RANGE_NONE;
                    dumpCorrelationRecord();
                    break;

                case '.':
                    rangeMode = RANGE_NONE;
                    break;

                case 0x0D:
                case 0x0A:
                    rangeMode = RANGE_NONE;
                    break;

                default:
                    Serial.println("============================================");
                    Serial.println("== LLv4 - Type a single character command ==");
                    Serial.println("============================================");
                    Serial.println(" 1 - Single Measurement");
                    Serial.println(" 2 - Continuous Measurement");
                    Serial.println(" 3 - Dump Correlation Record");
                    Serial.println(" . - Stop Measurement");

                    rangeMode = RANGE_NONE;
                    break;
            }
        }

        //===================================================================
        // 2) Check on mode and operate accordingly
        //===================================================================
        switch (rangeMode)
        {
            case RANGE_NONE:
                newDistance = 0;
                break;

            case RANGE_SINGLE:
                newDistance = distanceSingle(&distance);
                rangeMode   = RANGE_NONE;
                break;

            case RANGE_CONTINUOUS:
                newDistance = distanceContinuous(&distance);
                break;

            default:
                newDistance = 0;
                break;
        }

        //===================================================================
        // 3) When there is new distance data, print it to the serial port
        //===================================================================
        if (newDistance)
        {
            Serial.println(distance);
        }
    }
}

//---------------------------------------------------------------------
// Read Single Distance Measurement
//
// This is the simplest form of taking a measurement. This is a
// blocking function as it will not return until a range has been
// taken and a new distance measurement can be read.
//---------------------------------------------------------------------
uint8_t distanceSingle(uint16_t * distance)
{
    // 1. Wait for busyFlag to indicate device is idle. This must be
    //    done before triggering a range measurement.
    myLidarLite.waitForBusy();

    // 2. Trigger range measurement.
    myLidarLite.takeRange();

    // 3. Wait for busyFlag to indicate device is idle. This should be
    //    done before reading the distance data that was triggered above.
    myLidarLite.waitForBusy();

    // 4. Read new distance data from device registers
    *distance = myLidarLite.readDistance();

    return 1;
}

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag in the STATUS
// register can alert the user that the distance measurement is new
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
    if (myLidarLite.getBusyFlag() == 0)
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

//---------------------------------------------------------------------
// Print the correlation record for analysis
//---------------------------------------------------------------------
void dumpCorrelationRecord()
{
    int16_t corrValues[192];
    uint8_t i;

    myLidarLite.correlationRecordRead(corrValues);

    for (i=0 ; i<192 ; i++)
    {
        Serial.print(corrValues[i], DEC);
        Serial.print(",");
    }
    Serial.println(" ");
}

