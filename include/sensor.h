#ifndef SENSOR_H_
#define SENSOR_H_

#include "vex.h"

const int kLedPower = 100;      // 内置白光 LED 亮度 (0–100)
const int kLoopDelay = 50;    // 主循环间隔 ms
enum class ColorName { red, green, blue, yellow, unknown };
ColorName classifyHue(double hue); 
void sensorInit();
void runsensor();

extern int Accelerate;

#endif
