#include "sensor.h"
#include "robot_confing.h"

int Accelerate = 0;

// -------------------- 初始化 --------------------
void sensorInit() {
     //colorSensor.setLight(ledState::on);
     //colorSensor.setLightPower(kLedPower);
}

// -------------------- 颜色分类（若后续使用） --------------------
ColorName classifyHue(double hue) {
    if ((hue >= 330) || (hue < 30))      return ColorName::red;
    else if (hue >= 30  && hue < 90)     return ColorName::yellow;
    else if (hue >= 90  && hue < 180)    return ColorName::green;
    else if (hue >= 180 && hue < 270)    return ColorName::blue;
    else                                 return ColorName::unknown;
}

// -------------------- 主循环 --------------------
void runsensor() {

    // 只初始化一次串口
    static bool inited = false;
    if (!inited) {
        vexGenericSerialEnable(sensor.index(), 9);          // 启用通用串口
        vexGenericSerialBaudrate(sensor.index(), 115200);   // 115200 bps
        inited = true;
    }

    char  now_color = 'N';
    char  pre_color = 'N';

    while (true) {

        /* ---------- 方案 1：一次性读完缓冲区 ---------- */
        int bytesAvail = vexGenericSerialReceiveAvail(sensor.index());
        while (bytesAvail-- > 0) {
            int c = vexGenericSerialReadChar(sensor.index());
            if (c != -1) {
                now_color = static_cast<char>(c);   // 每次覆盖，循环结束后即最新字节
            }
        }
        /* -------------------------------------------- */

        // ========== 输出到 Brain 屏幕 ==========
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("%c", now_color);
        Brain.Screen.newLine();
        switch (now_color) {
            case 'R':  Brain.Screen.print(">> RED <<");   Accelerate = is_BLUE?1:0; break;
            case 'G':  Brain.Screen.print(">> GREEN <<"); Accelerate = 0; break;
            case 'B':  Brain.Screen.print(">> BLUE <<");  Accelerate = is_BLUE?0:1; break;
            default:   Brain.Screen.print("?? UNKNOWN ??"); Accelerate = 0; break;
        }
        if(is_BLUE)
        {
          // 颜色从 R 切换到非 R 的边沿检测
          if (pre_color == 'R' && now_color != 'R') {
              // this_thread::sleep_for(140);
              Accelerate = -1;
              vex::this_thread::sleep_for(500);
          }
        }else
        {
          // 颜色从 R 切换到非 R 的边沿检测
          if (pre_color == 'B' && now_color != 'B') {
              // this_thread::sleep_for(140);
              Accelerate = -1;
              vex::this_thread::sleep_for(500);
          }
        }

        pre_color = now_color;

        vex::this_thread::sleep_for(80);   // 主循环节拍
    }
}
