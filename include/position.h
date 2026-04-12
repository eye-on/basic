#ifndef POSITION_H_
#define POSITION_H_

#include"timer.h"
#include"vex.h"

class Position{
private:
    Position():curIMUHeading_rad(0),curIMUHeading_deg(0),cur_lrad(0),cur_rrad(0),cur_rad(0),last_lrad(0),last_rrad(0),last_rad(0),
               lastTime(0),sampleTime(0){
                left.resetPosition();
                right.resetPosition();
               }
               /*curLSpeed(0),curRSpeed(0),lastLSpeed(0), lastRSpeed(0),selfSpeed(0),
               globalXSpeed(0), globalYSpeed(0),lastGlobalXSpeed(0), lastGlobalYSpeed(0),
               globalX(0), globalY(0),*/

    double curIMUHeading_rad,curIMUHeading_deg;
    double cur_lrad, cur_rrad,cur_rad;
    double last_lrad, last_rrad,last_rad;
    double lastTime, sampleTime;
    /*double curLSpeed, curRSpeed;
    double lastLSpeed, lastRSpeed;
    double selfSpeed;
    double globalXSpeed, globalYSpeed;
    double lastGlobalXSpeed, lastGlobalYSpeed;
    double globalX, globalY;*/

    int32_t left_port=vex::PORT3,right_port=vex::PORT7;

    vex::motor left=vex::motor(left_port),right=vex::motor(right_port);

    Timer timer;

    void updateInertialHeading();
    void update_ldeg();
    void update_rdeg();
    /*void updateLSpeed();
    void updateRSpeed();
    void updateSelfSpeed();
    void updateGlobalYSpeed();
    void updateGlobalXSpeed();
    void updateGlobalY();
    void updateGlobalX();*/

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

    double get_lrad() const;
    double get_rrad() const;
    double get_rad() const;
    double getIMU_rad() const;
    double getIMU_deg() const;

    void updatePos();

    /*Point getPos() const;
    double getXSpeed() const;
    double getYSpeed() const;

    void resetYPosition();
    void resetXPosition();
    void setGlobalPosition(double _x, double _y);*/
    void reset();

};

void updatePosition();

#endif
