# LIDAR-Lite Arduino Library

* [Product Page: LIDAR-Lite v3](https://buy.garmin.com/en-US/US/p/557294) - See product page for operating manual
* [Product Page: LIDAR-Lite v3HP](https://buy.garmin.com/en-US/US/p/578152) - See product page for operating manual
* [Product Page: LIDAR-Lite v4 LED](https://buy.garmin.com/en-US/US/p/610275) - See product page for operating manual

This library provides quick access to basic functions of LIDAR-Lite
via the Arduino interface. Additionally, it can provide a user of any
platform with a template for their own application code.

For detailed specifications, pinout, and connection diagrams, see the manuals linked at the above product pages.

***A Note on Compatibility:*** *Minor interface changes have occurred between LIDAR-Lite v3, v3HP, and previous versions. Backwards-compatibility of this library is largely preserved, though support is not directly provided for v1 and v2.*

## Installation instructions
To install, download this repository and place in your Arduino libraries folder or use the Arduino Library Manager. If you need help, follow the instructions here: [http://arduino.cc/en/Guide/Libraries](http://arduino.cc/en/Guide/Libraries).

## Example Sketches
### v3/GetDistancePWM
This is the simplest demonstration of LIDAR-Lite. It shows how to read a distance using the PWM interface.

### v3/GetDistanceI2c
This demonstration shows how to read distance using the I2C interface and choose preset configurations.

### v3/ShortRangeHighSpeed
This example shows a method to run LIDAR-Lite at high speed for short range applications. It combines a variety of settings to trade off range and accuracy for very fast measurements.

### v3HP/v3HP_I2C
This example shows various methods to run LIDAR-Lite v3HP.

### v3HP/v3HP_MONITOR
Use the MODE pin to monitor STATUS of the LIDAR-Lite v3HP.

### v4LED/v4LED_I2C/
Operation examples for LIDAR-Lite v4 LED.

## Version History
* [3.0.0](https://github.com/garmin/LIDARLite_Arduino_Library/tree/3.0.0) - Support for LIDAR-Lite v4 LED
* [2.0.6](https://github.com/garmin/LIDARLite_Arduino_Library/tree/2.0.6) - Add v3HP library function to reset the reference filter
* [2.0.5](https://github.com/garmin/LIDARLite_Arduino_Library/tree/2.0.5) - Add v3 library function to set alternate I2C addr
* [2.0.4](https://github.com/garmin/LIDARLite_Arduino_Library/tree/2.0.4) - Update short-range, high-error configuration
* [2.0.3](https://github.com/garmin/LIDARLite_Arduino_Library/tree/2.0.3) - New example using MODE pin for STATUS output of v3HP
* [2.0.2](https://github.com/garmin/LIDARLite_Arduino_Library/tree/2.0.2) - Updates to v3HP library
* [2.0.0](https://github.com/garmin/LIDARLite_Arduino_Library/tree/2.0.0) - Support for LIDAR-Lite v3HP
* [1.0.3](https://github.com/garmin/LIDARLite_Arduino_Library/tree/1.0.3) - Fix version convention
* [1.0.2](https://github.com/garmin/LIDARLite_Arduino_Library/tree/v1.0.2) - Library Manager Update
* [1.0.1](https://github.com/garmin/LIDARLite_Arduino_Library/tree/v1.0.1) - Release to Library Manager
* [1.0.0](https://github.com/garmin/LIDARLite_Arduino_Library/tree/1.0.0) - Initial release

## Raspberry Pi Library for LIDAR-Lite v3
* [LIDAR-Lite v3 on Pi](https://github.com/garmin/LIDARLite_RaspberryPi_Library/) - Basic support available for Raspberry Pi 3B+

## License
Copyright (c) 2018 Garmin Ltd. or its subsidiaries. Distributed under the Apache 2.0 License.
See [LICENSE](LICENSE) for further details.
