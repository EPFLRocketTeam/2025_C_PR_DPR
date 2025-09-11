// Last update: 11/03/2025
#include "constant.h"
#include "./2024_C_AV_INTRANET/intranet_commands.h"
#include "PTE7300_I2C.h"

typedef struct dpr_memory_t {
    DPR_FSM state;
    PTE7300_I2C my_sensor;
    int16_t value_sensor;
    bool status_led;
    int time_led;
    int time_msg;
    bool PN_state;
    bool VX_state;
    bool VN_state;
    float tank1_temp;
    float tank2_temp;
    float tank3_temp;
    float tank1_press;
    float tank2_press;
    float tank3_press;
    float copv_temp;
    float copv_press;
    int max_time_passivate;
} dpr_memory_t;

typedef struct dpr_memory_controller_t {
    float tankPressure;
    float copvPressure;
    float initialCopvPressure;
    float limitPressure;
    float rampedPressure;
    float fullScalePressure;
    float tank1_offset;
    float tank2_offset;
    float tank3_offset;
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
    void pid();
    float computeFullScale();
    void regulation();
    void pressurization();
    void actuate();
    void initialize();
    void set_offset();

public:
    DPRComputer(DPR_FSM);
    ~DPRComputer();

    void reset_dpr();

    //valve and motor opening/closing
    void open_valve(int valve);
    void close_valve(int valve);

    // valve actuating
    void actuate_valve(int valve);
    void deactuate_valve(int valve);
    
    //getters
    dpr_memory_t get_memory();

    //setters
    void set_state(DPR_FSM new_state);

    // FSM
    void update(int time);

    float filterTankPressure(bool controller);
    float filterTankTemp();
};


bool muxSelect(uint8_t ch);

void status_led(RGBColor color);
void turn_on_sequence();