#include "position.h"
#include "basic_functions.h"
#include "parameters.h"


void Position::updateInertialHeading() { 
    curIMUHeading = deg_to_rad(IMUHeading()); 
}

void Position::updateLMileage() {
    lastLMileage = curLMileage;
    curLMileage = -deg_to_rad(-motor_bl1.position(vex::degrees)) * WHEEL_TRANSITION_COEFFICIENT;
}

void Position::updateRMileage() {
    lastRMileage = curRMileage;
    curRMileage = -deg_to_rad(-motor_br1.position(vex::degrees)) * WHEEL_TRANSITION_COEFFICIENT;
}

void Position::updateLSpeed() {
    lastLSpeed = curLSpeed;
    double v = (curLMileage - lastLMileage) * 1000 / sampleTime;
    if (std::abs(v) > 1000 || std::abs(v) < 0.001) {
        v = 0;
    }
    curLSpeed = v;
}

void Position::updateRSpeed() {
    lastRSpeed = curRSpeed;
    double v = (curRMileage - lastRMileage) * 1000 / sampleTime;
    if (std::abs(v) > 1000 || std::abs(v) < 0.001) {
        v = 0;
    }
    curRSpeed = v;
}

void Position::updateSelfSpeed() { 
    selfSpeed = (curLSpeed + curRSpeed) / 2; 
}

void Position::updateGlobalYSpeed() {
    lastGlobalYSpeed = globalYSpeed;
    globalYSpeed = selfSpeed * cos(curIMUHeading);
    if (std::abs(globalYSpeed) < 0.01) {
        globalYSpeed = 0;
    }
    /*if (abs(globalYSpeed) > 250) {
        globalYSpeed = lastGlobalYSpeed;
    }*/
}

void Position::updateGlobalXSpeed() {
    lastGlobalXSpeed = globalXSpeed;
    globalXSpeed = selfSpeed * sin(curIMUHeading);
    if (std::abs(globalXSpeed) < 0.01) {
        globalXSpeed = 0;
    }
    /*if (abs(globalXSpeed) > 250) {
        globalXSpeed = lastGlobalXSpeed;
    }*/
}

void Position::updateGlobalY() {
    double d = (globalYSpeed + lastGlobalYSpeed) * sampleTime / 1000 / 2;
    if (std::abs(d) < 0.001) {
        return;
    } else {
        globalY = globalY + d;
    }
}

void Position::updateGlobalX() {
    double d = (globalXSpeed + lastGlobalXSpeed) * sampleTime / 1000 / 2;
    if (std::abs(d) < 0.001) {
        return;
    } else {
        globalX = globalX + d;
    }
}

void Position::updatePos() {
    double time_cur = timer.getTime_ms();
    sampleTime = time_cur - lastTime;
    lastTime = time_cur;

    if (sampleTime < 0.001) {
        // dealing with the situation that sampleTime is too small
        sampleTime = REFRESH_TIME;
    }

    updateInertialHeading();
    updateLMileage();
    updateRMileage();
    updateLSpeed();
    updateRSpeed();
    updateSelfSpeed();
    updateGlobalYSpeed();
    updateGlobalXSpeed();
    updateGlobalY();
    updateGlobalX();
}

Point Position::getPos() const { return Point(globalX, globalY); }

double Position::getXSpeed() const { return globalXSpeed; }

double Position::getYSpeed() const { return globalYSpeed; }

double Position::getLMileage() const { return curLMileage; }

double Position::getRMileage() const { return curRMileage; }

void Position::resetXPosition() { globalX = 0; }

void Position::resetYPosition() { globalY = 0; }

void Position::setGlobalPosition(double _x, double _y) {
    globalX = _x;
    globalY = _y;
}

void updatePosition() {
    while (true) {
        Position::getInstance()->updatePos();
        vex::this_thread::sleep_for(REFRESH_TIME);
    }
}

void Position::reset() {
    globalX = 0;
    globalY = 0;
}
