#ifndef POSITION_H_
#define POSITION_H_

#include"point.h"
#include"vector.h"
#include"timer.h"

class Position{
private:
    Position():curIMUHeading(0),curLMileage(0),curRMileage(0),lastLMileage(0),lastRMileage(0),
               curLSpeed(0),curRSpeed(0),lastLSpeed(0), lastRSpeed(0),selfSpeed(0),
               globalXSpeed(0), globalYSpeed(0),lastGlobalXSpeed(0), lastGlobalYSpeed(0),
               globalX(0), globalY(0),lastTime(0),sampleTime(0){}

    double curIMUHeading;
    double curLMileage, curRMileage;
    double lastLMileage, lastRMileage;
    double curLSpeed, curRSpeed;
    double lastLSpeed, lastRSpeed;
    double selfSpeed;
    double globalXSpeed, globalYSpeed;
    double lastGlobalXSpeed, lastGlobalYSpeed;
    double globalX, globalY;
    double lastTime, sampleTime;
    Timer timer;

    void updateInertialHeading();
    void updateLMileage();
    void updateRMileage();
    void updateLSpeed();
    void updateRSpeed();
    void updateSelfSpeed();
    void updateGlobalYSpeed();
    void updateGlobalXSpeed();
    void updateGlobalY();
    void updateGlobalX();

public:
    static Position *getInstance() {
        static Position *p = NULL;
        if (p == NULL) {
            p = new Position();
        }
        return p;
    }
    static void deleteInstance() {
        Position *p = Position::getInstance();
        if (p != NULL) {
            delete p;
            p = NULL;
        }
    }

    void updatePos();
    Point getPos() const;
    double getXSpeed() const;
    double getYSpeed() const;
    double getLMileage() const;
    double getRMileage() const;
    void resetYPosition();
    void resetXPosition();
    void setGlobalPosition(double _x, double _y);
    void reset();

};

void updatePosition();

#endif