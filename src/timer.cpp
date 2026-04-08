#include "timer.h"
#include <cmath>

int Timer::getTime_ms() const {
    return (std::floor(1000*Brain.Timer.value())-start_time_ms);
}

void Timer::reset(){
    start_time_ms=1000*Brain.Timer.value();
}
