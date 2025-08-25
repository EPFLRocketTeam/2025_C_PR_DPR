// Last update: 11/03/2025
#include <Arduino.h>
#include "vector"

#define DEBUG

// PINs attribition on Teensy
#define DPR         7
#define VENT1       8
#define VENT2       9          //I-GP
#define VENT_SPARE  10

#define RESET       1
#define RGB_RED     3
#define RGB_GREEN   2
#define RGB_BLUE    4
#define BUZZER      7

// Adresses I2C
#define SLAVE_ADDR  0x0C
#define MUX_ADDR    0x70 // 0xE0
#define SENS_ADDR   0x6C        //or 0x6C
#define XTA_CH      0x01        // channel 1
#define NCO_CH      0x02        // channel 2
#define SENSATA_3   0x04        // channel 3
#define SENSATA_4   0x08        // channel 4

// Status 
#define LED_TIMEOUT 1000 // 1 second

enum DPRfsm
{
    IDLE,
    WAKEUP,
    TEST,
    SETUP,
    WAIT,
    ARM,
    ABORT,
    ERROR
};

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