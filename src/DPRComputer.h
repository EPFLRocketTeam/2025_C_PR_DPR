// Last update: 11/03/2025
#include "constant.h"
#include "DPRControl.h"
#include "./2024_C_AV_INTRANET/intranet_commands.h"
#include "PTE7300_I2C.h"

typedef struct dpr_memory_t {
    DPRfsm state;
    PTE7300_I2C my_sensor;
    int16_t value_sensor;
    bool status_led;
    int time_led;
    int time_msg;
    bool DPR_state;
    bool VENT1_state;
    bool VENT2_state;
    float tank1_temp;
    float tank2_temp;
    float tank1_press;
    float tank2_press;
    float tank3_temp;
    float tank3_press;
    float copv_temp;
    float copv_press;
} dpr_memory_t;

class DPRComputer
{
private:
    dpr_memory_t memory;
    PTE7300_I2C my_sensor;
    int16_t value_sensor;
    DPRControl dpr_controler;

public:
    DPRComputer(DPRfsm);
    ~DPRComputer();

    //valve and motor control
    void open_valve(int valve);
    void close_valve(int valve);

    //sensor reading
    float read_pressure(int sensor);
    float read_temperature(int sensor);

    
    //getters
    dpr_memory_t get_memory();

    //setters
    void set_state(DPRfsm new_state);


    // sequences


    void update(int time);

    // testing
    std::vector<float> test_read_sensors();
    void stress_test(int cycles, int valve);
    void test_valves();
};


bool muxSelect(uint8_t ch);

void status_led(RGBColor color);
void turn_on_sequence();