#include "DPRComputer.h"
#include "DPRControl.h"

//==============================================================================================================
control::control() {
    tankPressure = 0.0;
    copvPressure = 0.0;
    initialCopvPressure = 0.0;
    limitPressure = 60;
    rampedPressure = 0.0;
    fullScalePressure = 1000;
    error = 0.0;
    lastError = 0.0;
    integral = 0.0;
    derivative = 0.0;
    kp = 1.70;
    ki = 0.3;
    kd = 0.0;
    dutyRatio = 0.0;
    dutyTime = 0.0;
    startTime = 0;
    lastTime = 0;
    controlPeriod = 0;
}

//==============================================================================================================
void control::initialize(DPRComputer* computer) {
    // Valves in regulation state
    //digitalWrite(VE, HIGH);
    computer->open_valve(VE);
    //digitalWrite(VN, LOW);
    computer->close_valve(VN);
    //digitalWrite(PN, LOW);
    computer->close_valve(PN);

    //delay(100);

    // Read initial COPV pressure
    initialCopvPressure = computer->read_pressure(COPV);

    // initialize ramped pressure
    rampedPressure = 0.0;

    startTime = 0;
    lastTime = 0;
}

//==============================================================================================================
void control::regulation() {
    if (millis() - lastTime > controlPeriod) {
        lastTime = millis();
        tankPressure = filterTankPressure();
        copvPressure = computer.read_pressure(COPV);
        error = limitPressure - tankPressure;
        fullScalePressure = computeFullScale();
        pid();
        lastError = error;
        dutyTime = dutyRatio * controlPeriod;
    }
}

//==============================================================================================================
void control::pressurization(DPRComputer* computer) {
    if (millis() - lastTime > controlPeriod) {
        lastTime = millis();
        if (rampedPressure < limitPressure) {
            rampedPressure = (limitPressure / 10000)*millis() - (limitPressure*startTime)/10000;
        }
        else {
            rampedPressure = limitPressure;
        }
        tankPressure = filterTankPressure();
        copvPressure = computer->read_pressure(COPV);
        error = rampedPressure - tankPressure;
        fullScalePressure = computeFullScale();
        pid();
        lastError = error;
        dutyTime = dutyRatio * controlPeriod;
    }
}

//==============================================================================================================
float control::filterTankPressure(DPRComputer* computer) {
    float pressure1 = computer->read_pressure(TANK1);
    float pressure2 = computer->read_pressure(TANK2);
    float pressure3 = computer->read_pressure(TANK3);

    if ((abs(pressure1 - pressure2) > abs(pressure2 - pressure3)) && (abs(pressure1 - pressure3) > abs(pressure2 - pressure3))) {
        return 0.5*(pressure2 + pressure3);
    }
    else if ((abs(pressure2 - pressure1) > abs(pressure1 - pressure3)) && (abs(pressure2 - pressure3) > abs(pressure1 - pressure3))) {
        return 0.5*(pressure1 + pressure3);
    }
    else return 0.5*(pressure1 + pressure2);
}

float control::computeFullScale() {
    return (limitPressure+(copvPressure-initialCopvPressure)*(limitPressure/(initialCopvPressure-limitPressure)));
}

//==============================================================================================================
float control::pid() {
    float correction = 0.0;

    // integral contribution
    integral += controlPeriod * error;

    // derivative contribution
    derivative = ((error - lastError)*1000) / controlPeriod;
    
    // PID correction
    correction = kp*error + ki*integral + kd*derivative;

    if (correction > fullScalePressure) {
        dutyRatio = 1.0;
    }
    else if (correction < 0) {
        dutyRatio = 0.0;
        integral = 0.0;
    }
    else { // normalisation
        dutyRatio = correction / fullScalePressure;
    }
}

//==============================================================================================================
void control::actuate() {
    if (millis() - lastTime < dutyTime) {
        digitalWrite(PN, HIGH);
    }
    else {
        digitalWrite(PN, LOW);
    }
}
