// Last update: 11/03/2025
#include <Arduino.h>
#include "vector"

// ================= ifdef defines =================
#define DEBUG
// #define DRY_RUN

// controller constants
#define KP 1.7
#define KI 0.3
#define KD 0.025
#define CONTROL_PERIOD 100

#ifdef DRY_RUN
#define PRESSURIZATION_OX_SET_PRESSURE 29.03
#define PRESSURIZATION_FUEL_SET_PRESSURE 18.79
#else
#define PRESSURIZATION_OX_SET_PRESSURE 13.834
#define PRESSURIZATION_FUEL_SET_PRESSURE 16.214
#endif

#define PASSIVATION_DELAY_NO_COM_DPR 140000
#define PASSIVATION_COPV_DURATION 300000

#define RAMP_DELAY 5000

// ================ pin configuration =================
// Look at the silkscreen to know where to connect each valve
#ifdef PRB_DPR
#define VX 37       //at the place of Me-b
#define PN 36       //at the place of MO-bC
#define VN 35       //at th eplace of MOSFET
#else
#define VX 8
#define VN 9
#define PN 7 
#endif


#ifdef PRB_DPR
#define RESET       9
#define RGB_RED     PIN_A7
#define RGB_GREEN   PIN_A8
#define RGB_BLUE    PIN_A9
#define BUZZER      PIN_A1
#else
#define RESET       1
#define RGB_RED     3
#define RGB_GREEN   2
#define RGB_BLUE    4
#define BUZZER      7
#endif

// ================ I2C configuration =================
#define MUX_ADDR    0x70 // 0xE0
#define SENS_ADDR   0x6C        //or 0x6C
#define TANK1       0x01        // channel 0 --> EIN on PRB
#define TANK2       0x02        // channel 1 --> CCC on PRB
#define TANK3       0x08        // channel 2 --> not on PRB
#define COPV        0x04        // channel 3 --> CIG on PRB

enum DPR_FSM
{
    MANUAL,
    INITIALIZE_PRESSURIZATION,
    PRESSURIZATION,
    INITIALIZE_REGULATION,
    REGULATION,
    PRESSURIZATION_OFF,
    INITIALIZE_PASSIVATION,
    PASSIVATION,
    ABORT_IN_FLIGHT,
    ABORT_ON_GROUND
};

// Status 
#define LED_TIMEOUT 1000 // 1 second

struct RGBColor {
    int red;
    int green;
    int blue;
};

const RGBColor RED    = {HIGH, LOW, LOW};
const RGBColor GREEN  = {LOW, HIGH, LOW};
const RGBColor BLUE   = {LOW, LOW, HIGH};
const RGBColor WHITE  = {HIGH, HIGH, HIGH};
const RGBColor PURPLE = {HIGH, LOW, HIGH};
const RGBColor ORANGE = {HIGH, HIGH, LOW};
const RGBColor TEAL   = {LOW, HIGH, HIGH};
const RGBColor OFF    = {LOW, LOW, LOW};