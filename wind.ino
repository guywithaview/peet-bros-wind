/*
  Wind - NMEA Wind Instrument
  Copyright (c) 2018 Tom K
  Originally written for Arduino Pro Mini 328
  v3c

  MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#define VERSION "Wind v3 18-Sep-2015"

#include "PString.h"

#define windSpeedPin 2
#define windDirPin 3
#define windSpeedINT 0 // INT0
#define windDirINT 1   // INT1

// Pin 13 has an LED connected on most Arduino boards.
int LED = 13;

const unsigned long DEBOUNCE = 10000ul;      // Minimum switch time in microseconds
const unsigned long DIRECTION_OFFSET = 0ul;  // Manual direction offset in degrees, if required
const unsigned long TIMEOUT = 1500000ul;       // Maximum time allowed between speed pulses in microseconds
const unsigned long UPDATE_RATE = 500ul;     // How often to send out NMEA data in milliseconds
const float filterGain = 0.25;               // Filter gain on direction output filter. Range: 0.0 to 1.0
                                             // 1.0 means no filtering. A smaller number increases the filtering

// Knots is actually stored as (Knots * 100). Deviations below should match these units.
const int BAND_0 =  10 * 100;
const int BAND_1 =  80 * 100;

const int SPEED_DEV_LIMIT_0 =  5 * 100;     // Deviation from last measurement to be valid. Band_0: 0 to 10 knots
const int SPEED_DEV_LIMIT_1 = 10 * 100;     // Deviation from last measurement to be valid. Band_1: 10 to 80 knots
const int SPEED_DEV_LIMIT_2 = 30 * 100;     // Deviation from last measurement to be valid. Band_2: 80+ knots

// Should be larger limits as lower speed, as the direction can change more per speed update
const int DIR_DEV_LIMIT_0 = 25;     // Deviation from last measurement to be valid. Band_0: 0 to 10 knots
const int DIR_DEV_LIMIT_1 = 18;     // Deviation from last measurement to be valid. Band_1: 10 to 80 knots
const int DIR_DEV_LIMIT_2 = 10;     // Deviation from last measurement to be valid. Band_2: 80+ knots

volatile unsigned long speedPulse = 0ul;    // Time capture of speed pulse
volatile unsigned long dirPulse = 0ul;      // Time capture of direction pulse
volatile unsigned long speedTime = 0ul;     // Time between speed pulses (microseconds)
volatile unsigned long directionTime = 0ul; // Time between direction pulses (microseconds)
volatile boolean newData = false;           // New speed pulse received
volatile unsigned long lastUpdate = 0ul;    // Time of last serial output

volatile int knotsOut = 0;    // Wind speed output in knots * 100
volatile int dirOut = 0;      // Direction output in degrees
volatile boolean ignoreNextReading = false;

boolean debug = false;

void setup()
{
    pinMode(LED, OUTPUT);

    Serial.begin(38400, SERIAL_8N1);
    Serial.println(VERSION);
    Serial.print("Direction Filter: ");
    Serial.println(filterGain);

    pinMode(windSpeedPin, INPUT);
    attachInterrupt(windSpeedINT, readWindSpeed, FALLING);

    pinMode(windDirPin, INPUT);
    attachInterrupt(windDirINT, readWindDir, FALLING);

    interrupts();
}


void readWindSpeed()
{
    // Despite the interrupt being set to FALLING edge, double check the pin is now LOW
    if (((micros() - speedPulse) > DEBOUNCE) && (digitalRead(windSpeedPin) == LOW))
    {
        // Work out time difference between last pulse and now
        speedTime = micros() - speedPulse;

        // Direction pulse should have occured after the last speed pulse
        if (dirPulse - speedPulse >= 0) directionTime = dirPulse - speedPulse;

        newData = true;
        speedPulse = micros();    // Capture time of the new speed pulse
    }
}

void readWindDir()
{
    if (((micros() - dirPulse) > DEBOUNCE) && (digitalRead(windDirPin) == LOW))
    {
      dirPulse = micros();        // Capture time of direction pulse
    }
}

boolean checkDirDev(long knots, int dev)
{
    if (knots < BAND_0)
    {
        if ((abs(dev) < DIR_DEV_LIMIT_0) || (abs(dev) > 360 - DIR_DEV_LIMIT_0)) return true;
    }
    else if (knots < BAND_1)
    {
        if ((abs(dev) < DIR_DEV_LIMIT_1) || (abs(dev) > 360 - DIR_DEV_LIMIT_1)) return true;
    }
    else
    {
        if ((abs(dev) < DIR_DEV_LIMIT_2) || (abs(dev) > 360 - DIR_DEV_LIMIT_2)) return true;
    }
    return false;
}

boolean checkSpeedDev(long knots, int dev)
{
    if (knots < BAND_0)
    {
        if (abs(dev) < SPEED_DEV_LIMIT_0) return true;
    }
    else if (knots < BAND_1)
    {
        if (abs(dev) < SPEED_DEV_LIMIT_1) return true;
    }
    else
    {
        if (abs(dev) < SPEED_DEV_LIMIT_2) return true;
    }
    return false;
}

void calcWindSpeedAndDir()
{
    unsigned long dirPulse_, speedPulse_;
    unsigned long speedTime_;
    unsigned long directionTime_;
    long windDirection = 0l, rps = 0l, knots = 0l;

    static int prevKnots = 0;
    static int prevDir = 0;
    int dev = 0;

    // Get snapshot of data into local variables. Note: an interrupt could trigger here
    noInterrupts();
    dirPulse_ = dirPulse;
    speedPulse_ = speedPulse;
    speedTime_ = speedTime;
    directionTime_ = directionTime;
    interrupts();

    // Make speed zero, if the pulse delay is too long
    if (micros() - speedPulse_ > TIMEOUT) speedTime_ = 0ul;

    // The following converts revolutions per 100 seconds (rps) to knots x 100
    // This calculation follows the Peet Bros. piecemeal calibration data
    if (speedTime_ > 0)
    {
        rps = 100000000/speedTime_;                  //revolutions per 100s

        if (rps < 323)
        {
          knots = (rps * rps * -11)/11507 + (293 * rps)/115 - 12;
        }
        else if (rps < 5436)
        {
          knots = (rps * rps / 2)/11507 + (220 * rps)/115 + 96;
        }
        else
        {
          knots = (rps * rps * 11)/11507 - (957 * rps)/115 + 28664;
        }
        //knots = mph * 0.86897

        if (knots < 0l) knots = 0l;  // Remove the possibility of negative speed
        // Find deviation from previous value
        dev = (int)knots - prevKnots;

        // Only update output if in deviation limit
        if (checkSpeedDev(knots, dev))
        {
          knotsOut = knots;

          // If speed data is ok, then continue with direction data
          if (directionTime_ > speedTime_)
          {
              windDirection = 999;    // For debugging only (not output to knots)
          }
          else
          {
            // Calculate direction from captured pulse times
            windDirection = (((directionTime_ * 360) / speedTime_) + DIRECTION_OFFSET) % 360;

            // Find deviation from previous value
            dev = (int)windDirection - prevDir;

            // Check deviation is in range
            if (checkDirDev(knots, dev))
            {
              int delta = ((int)windDirection - dirOut);
              if (delta < -180)
              {
                delta = delta + 360;    // Take the shortest path when filtering
              }
              else if (delta > +180)
              {
                delta = delta - 360;
              }
              // Perform filtering to smooth the direction output
              dirOut = (dirOut + (int)(round(filterGain * delta))) % 360;
              if (dirOut < 0) dirOut = dirOut + 360;
            }
            prevDir = windDirection;
          }
        }
        else
        {
          ignoreNextReading = true;
        }

        prevKnots = knots;    // Update, even if outside deviation limit, cause it might be valid!?
    }
    else
    {
        knotsOut = 0;
        prevKnots = 0;
    }

    if (debug)
    {
        Serial.print(millis());
        Serial.print(",");
        Serial.print(dirOut);
        Serial.print(",");
        Serial.print(windDirection);
        Serial.print(",");
        Serial.println(knotsOut/100);
        //Serial.print(",");
        //Serial.print(knots/100);
        //Serial.print(",");
        //Serial.println(rps);
    }
    else
    {
      if (millis() - lastUpdate > UPDATE_RATE)
      {
        printWindNmea();
        lastUpdate = millis();
      }
    }
}

byte getChecksum(char* str)
{
    byte cs = 0;
    for (unsigned int n = 1; n < strlen(str) - 1; n++)
    {
        cs ^= str[n];
    }
    return cs;
}

/*=== MWV - Wind Speed and Angle ===
 *
 * ------------------------------------------------------------------------------
 *         1   2 3   4 5
 *         |   | |   | |
 *  $--MWV,x.x,a,x.x,a*hh<CR><LF>
 * ------------------------------------------------------------------------------
 *
 * Field Number:
 *
 * 1. Wind Angle, 0 to 360 degrees
 * 2. Reference, R = Relative, T = True
 * 3. Wind Speed
 * 4. Wind Speed Units, K/M/N
 * 5. Status, A = Data Valid
 * 6. Checksum
 *
 */
void printWindNmea()
{
    char windSentence [30];
    float spd = knotsOut / 100.0;
    byte cs;
    //Assemble a sentence of the various parts so that we can calculate the proper checksum

    PString str(windSentence, sizeof(windSentence));
    str.print("$WIMWV,");
    str.print(dirOut);
    str.print(".0,R,");
    str.print(spd);
    str.print(",N,A*");
    //calculate the checksum

    cs = getChecksum(windSentence);
    //bug - arduino prints 0x007 as 7, 0x02B as 2B, so we add it now
    if (cs < 0x10) str.print('0');
    str.print(cs, HEX); // Assemble the final message and send it out the serial port
    Serial.println(windSentence);
}


void loop()
{
  int i;
  const unsigned int LOOP_DELAY = 50;
  const unsigned int LOOP_TIME = TIMEOUT / LOOP_DELAY;

  digitalWrite(LED, !digitalRead(LED));    // Toggle LED

  i = 0;
  // If there is new data, process it, otherwise wait for LOOP_TIME to pass
  while ((newData != true) && (i < LOOP_TIME))
  {
    i++;
    delayMicroseconds(LOOP_DELAY);
  }

  calcWindSpeedAndDir();    // Process new data
  newData = false;
}
