#include"position.h"
#include"basic_functions.h"
#include"parameters.h"


void Position::updateInertialHeading() { 
    curIMUHeading_deg = IMUHeading();
    curIMUHeading_rad = deg_to_rad(IMUHeading()); 
}

void Position::update_ldeg() {
    last_lrad = cur_lrad;
    cur_lrad = deg_to_rad(left.position(vex::degrees)); //* WHEEL_TRANSITION_COEFFICIENT;
}

void Position::update_rdeg() {
    last_rrad = cur_rrad;
    cur_rrad = deg_to_rad(right.position(vex::degrees)); //* WHEEL_TRANSITION_COEFFICIENT;
}

void Position::updatePos() {
    double time_cur = timer.getTime_ms();
    sampleTime = time_cur - lastTime;
    lastTime = time_cur;

    if (sampleTime < 0.001) {
        // dealing with the situation that sampleTime is too small
        sampleTime = REFRESH_TIME_ms;
    }

    updateInertialHeading();
    update_ldeg();
    update_rdeg();

    cur_rad=(cur_lrad+cur_rrad)/2.0;
    last_rad=(last_lrad+last_rrad)/2.0;
}

//Point Position::getPos() const { return Point(globalX, globalY); }

//double Position::getXSpeed() const { return globalXSpeed; }

//double Position::getYSpeed() const { return globalYSpeed; }

double Position::get_lrad() const { return cur_lrad; }

double Position::get_rrad() const { return cur_rrad; }

double Position::get_rad() const{ return cur_rad; }

double Position::getIMU_rad() const{ return curIMUHeading_rad; }

double Position::getIMU_deg() const{ return curIMUHeading_deg; }

//void Position::resetXPosition() { globalX = 0; }

//void Position::resetYPosition() { globalY = 0; }

/*void Position::setGlobalPosition(double _x, double _y) {
    globalX = _x;
    globalY = _y;
}*/

void updatePosition() {
    while (true) {
        Position::getInstance()->updatePos();
        vex::this_thread::sleep_for(REFRESH_TIME_ms);
    }
}

void Position::reset() {
    left.resetPosition();
    right.resetPosition();
}