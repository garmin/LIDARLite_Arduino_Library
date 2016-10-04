# LIDAR-Lite v3 Arduino Library

* [Product Page](https://buy.garmin.com/en-US/US/oem/sensors-and-boards/lidar-lite-v3/prod557294.html)
* [Operating Manual and Technical Specifications](http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf)

This library provides quick access to all the basic functions of LIDAR-Lite
via the Arduino interface. Additionally, it can provide a user of any
platform with a template for their own application code.

For detailed specifications, pinout, and connection diagrams, see the manual linked above.

***A Note on Compatibility:*** *Minor interface changes have occurred between LIDAR-Lite v3 and previous versions. Backwards-compatibility of this library is largely preserved, but support is not directly provided for v1 and v2.*

## Installation instructions
To install, download this repository and place in your Arduino libraries folder or use the Arduino Library Manager. If you need help, follow the instructions here: [http://arduino.cc/en/Guide/Libraries](http://arduino.cc/en/Guide/Libraries).

## Example Sketches
### GetDistancePWM
This is the simplest demonstration of LIDAR-Lite. It shows how to read a distance using the PWM interface.

### GetDistanceI2c
This demonstration shows how to read distance using the I2C interface and choose preset configurations.

### ShortRangeHighSpeed
This example shows a method to run LIDAR-Lite at high speed for short range applications. It combines a variety of settings to trade off range and accuracy for very fast measurements.

## Version History
* [v1.0.2](https://github.com/garmin/LIDARLite_v3_Arduino_Library/tree/v1.0.2) - Library Manager Update
* [v1.0.1](https://github.com/garmin/LIDARLite_v3_Arduino_Library/tree/v1.0.1) - Release to Library Manager
* [1.0.0](https://github.com/garmin/LIDARLite_v3_Arduino_Library/tree/1.0.0) - Initial release

## License
Copyright (c) 2016 Garmin Ltd. or its subsidiaries. Distributed under the Apache 2.0 License.
See [LICENSE](LICENSE) for further details.
