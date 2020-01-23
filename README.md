# X-NUCLEO-6180XA1

Arduino library to support the X-NUCLEO-6180XA1 based on VL6180X proximity sensor, gesture 
and ambient light sensing (ALS) module. This sensor uses I2C to communicate. An I2C instance 
is required to access to the sensor. The APIs provide simple distance measure, simple luminosity measure, 
single swipe gesture detection, directional (left/right) swipe gesture detection and single tap gesture detection.

## Examples

There are 4 examples with the  X-NUCLEO-53L0A1 library.
* X_NUCLEO_6180XA1_HelloWorld: This example code is to show how to get proximity and luminosity
  values of the onboard VL6180X sensor

* X_NUCLEO_6180XA1_Gesture_DirSwipe: This example code is to show how to combine the
  proximity values of the two VL6180X sensor satellites together with the gesture library
  in order to detect a directional swipe gesture.

* X_NUCLEO_6180XA1_Gesture_Swipe1: This example code is to show how to combine the
  proximity value of the onboard VL6180X sensor together with the gesture library
  in order to detect a simple swipe gesture without considering the direction.

* X_NUCLEO_6180XA1_Gesture_Tap1: This example code is to show how to combine the
  proximity value of the onboard VL6180X sensor together with the gesture
  library in order to detect a simple tap gesture.

## Dependencies

This package requires the following Arduino libraries:
* STM32duino VL6180X: https://github.com/stm32duino/VL6180X
* STM32duino Proximity Gesture: https://github.com/stm32duino/Proximity_Gesture

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-6180XA1

The VL6180X datasheet is available at  
http://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl6180x.html
