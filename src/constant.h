// Last update: 11/03/2025
#include <Arduino.h>
#include "vector"

#define DEBUG
#define DPR_LOX

#define DRY_RUN


//#define VENT_SPARE  10

#define VX 8 //spare
#define VN 9
#define PN 7 // DPR

#define KP 1.7
#define KI 0.3
#define KD 0.025

#ifdef DRY_RUN
#define LIMIT_PRESSURE_LOX 3
#define LIMIT_PRESSURE_ETH 3
#else
#define LIMIT_PRESSURE_LOX 49
#define LIMIT_PRESSURE_ETH 59
#endif

#define DELAY_VENT_N2 20

#define RESET       1
#define RGB_RED     3
#define RGB_GREEN   2
#define RGB_BLUE    4
#define BUZZER      7

// Adresses I2C
#define MUX_ADDR    0x70 // 0xE0
#define SENS_ADDR   0x6C        //or 0x6C
#define TANK1       0x01        // channel 0
#define TANK2       0x02        // channel 1
#define TANK3       0x08        // channel 2
#define COPV        0x04        // channel 3

enum DPR_FSM
{
    MANUAL,
    INITIALIZE_PRESSURIZATION,
    PRESSURIZATION,
    INITIALIZE_REGULATION,
    REGULATION,
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