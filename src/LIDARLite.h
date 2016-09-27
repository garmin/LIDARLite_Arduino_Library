/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  LIDARLite.h

  This library provides quick access to all the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2016 Garmin Ltd. or its subsidiaries.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

------------------------------------------------------------------------------*/
#ifndef LIDARLite_h
#define LIDARLite_h

#define LIDARLITE_ADDR_DEFAULT 0x62

#include <Arduino.h>

class LIDARLite
{
  public:
      LIDARLite();
      void begin(int = 0, bool = false, char = LIDARLITE_ADDR_DEFAULT);
      void configure(int = 0, char = LIDARLITE_ADDR_DEFAULT);
      void reset(char = LIDARLITE_ADDR_DEFAULT);
      int distance(bool = true, char = LIDARLITE_ADDR_DEFAULT);
      void write(char, char, char = LIDARLITE_ADDR_DEFAULT);
      void read(char, int, byte*, bool, char);
      void correlationRecordToSerial(char = '\n', int = 256, char = LIDARLITE_ADDR_DEFAULT);
};

#endif
