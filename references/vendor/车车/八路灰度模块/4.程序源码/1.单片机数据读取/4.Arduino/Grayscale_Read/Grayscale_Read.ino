//Grayscale Sensor / Arduino UNO
//        5V		          5V
//        GND		          GND
//        AD0		          2
//        AD1		          3
//        AD2		          4
//        OUT		          5
#include "grayscale_sensor.hpp"
#include <stdio.h>

uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
int i;
const char* sensor_labels[GRAYSCALE_SENSOR_CHANNELS] = {
        "X1", "X2", "X3", "X4", "X5", "X6", "X7", "X8"
    };

void setup() {
  Serial.begin(115200);
  Grayscale_Sensor_Init();
}

void loop() {
  Grayscale_Sensor_Read_All(g_sensor_data);
  for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
  {
    Serial.print("[ ");
      Serial.print(sensor_labels[i]);
      Serial.print(":");
      Serial.print(g_sensor_data[i]);
      Serial.print(" ]");
  }
  Serial.print("\n");
  //当X1的灯亮起时，值为1 / When the X1 light is on, the value is 1
  delay(60);
}

