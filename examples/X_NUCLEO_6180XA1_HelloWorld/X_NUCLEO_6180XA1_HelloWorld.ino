/**
 ******************************************************************************
 * @file    X_NUCLEO_6180XA1_HelloWorld.ino
 * @author  AST
 * @version V1.0.0
 * @date    14-November-2017
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-6180XA1
 *          proximity sensor expansion board based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl6180x_x_nucleo_6180xa1_class.h>
#include <stmpe1600_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

// Components.
STMPE1600DigiOut gpio0_top(&DEV_I2C, GPIO_12);
STMPE1600DigiOut gpio0_left(&DEV_I2C, GPIO_14);
STMPE1600DigiOut gpio0_right(&DEV_I2C, GPIO_15);
VL6180X_X_NUCLEO_6180XA1 sensor_vl6180x_top(&DEV_I2C, &gpio0_top);
VL6180X_X_NUCLEO_6180XA1 sensor_vl6180x_left(&DEV_I2C, &gpio0_left);
VL6180X_X_NUCLEO_6180XA1 sensor_vl6180x_right(&DEV_I2C, &gpio0_right);

/* Setup ---------------------------------------------------------------------*/

void setup() {
  int status;
  // Led.
  pinMode(13, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

//NOTE: workaround in order to unblock the I2C bus on the Arduino Due
#ifdef ARDUINO_SAM_DUE
   pinMode(71, OUTPUT);
   pinMode(70, OUTPUT);

   for (int i = 0; i<10; i++){
     digitalWrite(70, LOW);
     delay(3);
     digitalWrite(71, HIGH);
     delay(3);
     digitalWrite(70, HIGH);
     delay(3);
     digitalWrite(71, LOW);
     delay(3);
   }
   pinMode(70, INPUT);
   pinMode(71, INPUT);
#endif
//End of workaround

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL6180X top component.
  sensor_vl6180x_top.begin();
  
  // Switch off VL6180X top component.
  sensor_vl6180x_top.VL6180x_Off();
  
  // Configure (if present) VL6180X left component.
  sensor_vl6180x_left.begin();
  
  // Switch off (if present) VL6180X left component.
  sensor_vl6180x_left.VL6180x_Off();
  
  // Configure (if present) VL6180X right component.
  sensor_vl6180x_right.begin();
  
  // Switch off (if present) VL6180X right component.
  sensor_vl6180x_right.VL6180x_Off();
  
  // Initialize VL6180X top component.
  status = sensor_vl6180x_top.InitSensor(0x10);
  if(status)
  {
    SerialPort.println("Init sensor_vl6180x_top failed...");
  }
  sensor_vl6180x_top.StartInterleavedMode();
}


/* Loop ----------------------------------------------------------------------*/

void loop() {
  // Led blinking.
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);

  // Read Lux.
  uint32_t lux;
  sensor_vl6180x_top.GetLight(&lux);
  
  // Read Range.
  uint32_t range;
  sensor_vl6180x_top.GetDistance(&range);

  // Output data.
  char report[64];
  snprintf(report, sizeof(report), "| Lux[lux]: %ld | Range[mm]: %ld |", lux, range);
  SerialPort.println(report);
  
  delay(200);
}
