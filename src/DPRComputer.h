// Last update: 11/03/2025
#include "constant.h"
#include "./2024_C_AV_INTRANET/intranet_commands.h"
#include "PTE7300_I2C.h"

typedef struct dpr_memory_t {
    DPRfsm state;
    PTE7300_I2C my_sensor;
    int16_t value_sensor;
    bool status_led;
    int time_led;
    int time_msg;
    bool PN_state;
    bool VX_state;
    bool VENT2_state;
    float tank1_temp;
    float tank2_temp;
    float tank3_temp;
    float tank1_press;
    float tank2_press;
    float tank3_press;
    float copv_temp;
    float copv_press;
} dpr_memory_t;

typedef struct dpr_memory_controller_t {
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
    bool initialized;
} dpr_memory_controller_t;

class DPRComputer
{
private:
    dpr_memory_t memory;
    dpr_memory_controller_t memory_controller;
    PTE7300_I2C my_sensor;
    int16_t value_sensor;

    //sensor reading
    float read_pressure(int sensor);
    float read_temperature(int sensor);

    // =============== Controller ==============
    float pid();
    float computeFullScale();
    void regulation();
    void pressurization();
    void actuate();
    void initialize();

public:
    DPRComputer(DPRfsm);
    ~DPRComputer();

    //valve and motor control
    void open_valve(int valve);
    void close_valve(int valve);
    
    //getters
    dpr_memory_t get_memory();

    //setters
    void set_state(DPRfsm new_state);

    // FSM
    void update(int time);

    float filterTankPressure();
};


bool muxSelect(uint8_t ch);

void status_led(RGBColor color);
void turn_on_sequence();