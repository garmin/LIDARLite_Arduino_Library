/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  v3HP/v3HP_I2C

  This example shows methods for running the LIDAR-Lite v3 HP in various
  modes of operation. To exercise the examples open a serial terminal
  program (or the Serial Monitor in the Arduino IDE) and send ASCII
  characters to trigger the commands. See "loop" function for details.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3HP_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/

#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>

LIDARLite_v3HP myLidarLite;

#define FAST_I2C

enum rangeType_T
{
    RANGE_NONE,
    RANGE_SINGLE,
    RANGE_CONTINUOUS,
    RANGE_TIMER
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

    // Configure the LidarLite internal parameters so as to lend itself to
    // various modes of operation by altering 'configure' input integer to
    // anything in the range of 0 to 5. See LIDARLite_v3HP.cpp for details.
    myLidarLite.configure(0);
}


void loop()
{
    uint16_t distance;
    uint8_t  newDistance = 0;
    uint8_t  c;
    rangeType_T rangeMode = RANGE_NONE;

    // Continuous loop
    while (1)
    {
        // Each time through the loop, look for a serial input character
        if (Serial.available() > 0)
        {
            //  read input character ...
            c = (uint8_t) Serial.read();

            // ... and parse
            switch (c)
            {
                case 'S':
                case 's':
                    rangeMode = RANGE_SINGLE;
                    break;

                case 'C':
                case 'c':
                    rangeMode = RANGE_CONTINUOUS;
                    break;

                case 'T':
                case 't':
                    rangeMode = RANGE_TIMER;
                    break;

                case '.':
                    rangeMode = RANGE_NONE;
                    break;

                case 'D':
                case 'd':
                    rangeMode = RANGE_NONE;
                    dumpCorrelationRecord();
                    break;

                case 0x0D:
                case 0x0A:
                    break;

                default:
                    Serial.println("=====================================");
                    Serial.println("== Type a single character command ==");
                    Serial.println("=====================================");
                    Serial.println(" S - Single Measurement");
                    Serial.println(" C - Continuous Measurement");
                    Serial.println(" T - Timed Measurement");
                    Serial.println(" . - Stop Measurement");
                    Serial.println(" D - Dump Correlation Record");
                    break;
            }
        }

        switch (rangeMode)
        {
            case RANGE_NONE:
                newDistance = 0;
                break;

            case RANGE_SINGLE:
                newDistance = distanceSingle(&distance);
                break;

            case RANGE_CONTINUOUS:
                newDistance = distanceContinuous(&distance);
                break;

            case RANGE_TIMER:
                delay(250); // 4 Hz
                newDistance = distanceFast(&distance);
                break;

            default:
                newDistance = 0;
                break;
        }

        // When there is new distance data, print it to the serial port
        if (newDistance)
        {
            Serial.println(distance);
        }

        // Single measurements print once and then stop
        if (rangeMode == RANGE_SINGLE)
        {
            rangeMode = RANGE_NONE;
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
// Read Distance Measurement, Quickly
//
// Read distance. The approach is to poll the status register until the device goes
// idle after finishing a measurement, send a new measurement command, then read the
// previous distance data while it is performing the new command.
//---------------------------------------------------------------------
uint8_t distanceFast(uint16_t * distance)
{
    // 1. Wait for busyFlag to indicate device is idle. This must be
    //    done before triggering a range measurement.
    myLidarLite.waitForBusy();

    // 2. Trigger range measurement.
    myLidarLite.takeRange();

    // 3. Read previous distance data from device registers.
    //    After starting a measurement we can immediately read previous
    //    distance measurement while the current range acquisition is
    //    ongoing. This distance data is valid until the next
    //    measurement finishes. The I2C transaction finishes before new
    //    distance measurement data is acquired.
    *distance = myLidarLite.readDistance();

    return 1;
}

//---------------------------------------------------------------------
// Print the correlation record for analysis
//---------------------------------------------------------------------
void dumpCorrelationRecord()
{
    myLidarLite.correlationRecordToSerial(256);
}

