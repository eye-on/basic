#ifndef TIMER_H_
#define TIMER_H_

#include "robot_confing.h"

class Timer{
private :
    int start_time_ms;

public:
    Timer():start_time_ms(Brain.Timer.value()){};
    Timer(int time_ms):start_time_ms(time_ms){};
    void reset();
    int getTime_ms() const;

};

#endif
