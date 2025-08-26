#include <math.h>
#include "DPRComputer.h"

class DPRControl {
    public:
    void regulation(DPRComputer*);
    void pressurization(DPRComputer*);
    void actuate();
    void initialize(DPRComputer*);


    private:
    float tankPressure;
    float copvPressure;
    float initialCopvPressure;
    float limitPressure;
    float rampedPressure;
    float fullScalePressure;
    float error;
    float lastError;
    float integral;
    float derivative;
    float kp;
    float ki;
    float kd;
    float dutyRatio;
    float dutyTime;
    int startTime;
    int lastTime;
    int controlPeriod;
    float pid();
    float computeFullScale();
    float filterTankPressure(DPRComputer*);
};