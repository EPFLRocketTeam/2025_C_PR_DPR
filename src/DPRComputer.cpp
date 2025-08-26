// Last update: 11/03/2025
#include "DPRComputer.h"
#include "Wire.h"


#define TEST_WITHOUT_PRESSURE

DPRComputer::DPRComputer(DPRfsm init_state)
{
    memory.state = init_state;
    memory.status_led = false;
    memory.time_led = 0;
}

DPRComputer::~DPRComputer()
{
    memory.state = ERROR;
}

// ========= valve and motor control =========
void DPRComputer::open_valve(int valve)
{
    switch (valve)
    {
    case DPR:
        memory.DPR_state = true;
        break;
    case VENT1:
        memory.VENT1_state = true;
        break;
    case VENT2:
        memory.VENT2_state = true;
        break;

    default:
        break;
    }
    digitalWrite(valve, HIGH);
}

void DPRComputer::close_valve(int valve)
{
    switch (valve)
    {
    case DPR:
        memory.DPR_state = false;
        break;
    case VENT1:
        memory.VENT1_state = false;
        break;
    case VENT2:
        memory.VENT2_state = false;
        break;

    default:
        break;
    }
    digitalWrite(valve, LOW);
}


// ========= sensor reading =========
float DPRComputer::read_pressure(int sensor)
{
    int DSP_S = 0;
    float press = 0.0;

    muxSelect(sensor);

    switch (sensor)
    {
    case TANK1:  //TANK1
    case TANK2:  // TANK2
    case TANK3:
        DSP_S = my_sensor.readDSP_S();
        press = DSP_S * 5.0 / 1600 + 50;
    break;

    case COPV:
        // read 400 bar 
        break;
    
    default:
        break;
    }
   

    return press;
}


float DPRComputer::read_temperature(int sensor)
{
    //read temperature
    float temp = 0.0;
    int DSP_T = 0;

    muxSelect(sensor);
    DSP_T = my_sensor.readDSP_T();
    temp = DSP_T * 82.5 / 16000 + 42.5;
    
    return temp;
}


// ========= getter =========
dpr_memory_t DPRComputer::get_memory() { return memory; }

// ========= setter =========
void DPRComputer::set_state(DPRfsm new_state) { memory.state = new_state; }


// ========= sequences =========


bool muxSelect(uint8_t ch) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(ch);
    return Wire.endTransmission() == 0; // true if ACKed
}

void DPRComputer::update(int time)
{
    // Update the state machine
    // Declare variables outside the switch to avoid bypassing initialization
    std::vector<float> sensor_values;
    std::vector<float> pt1000_values;
    float kulite_value = 0.0;

    switch (memory.state)
    {
        case IDLE:
            if (!memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(TEAL);
                memory.time_led = time;
                memory.status_led = true;
            } else if (memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(OFF);
                memory.time_led = time;
                memory.status_led = false;
            }

            break;
        case WAKEUP:
            tone(BUZZER, 480, 1000);
            // Handle WAKEUP state if needed, otherwise do nothing
            break;
        case TEST:
            status_led(ORANGE);
            test_valves();
            sensor_values = test_read_sensors();
            Serial.println("Test done");
            memory.state = IDLE; // Return to IDLE after test
            status_led(OFF);
            break;
        
        case ARM:
            if (!memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(ORANGE);
                memory.time_led = time;
                memory.status_led = true;
            } else if (memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(OFF);
                memory.time_led = time;
                memory.status_led = false;
            }

            dpr_controler.regulation(this)

            break;

        case ABORT:
            status_led(RED);
            // tone(BUZZER, 440, 2500);
            break;

        case REGULATION:
            

        default:
            break;
    }

    memory.tank1_temp = read_temperature(TANK1);
    memory.tank2_temp = read_temperature(TANK2);
    memory.tank3_temp = read_temperature(TANK3);
    memory.copv_temp = read_temperature(COPV);
    memory.tank1_press = read_pressure(TANK1);
    memory.tank2_press = read_pressure(TANK2);
    memory.tank3_press = read_pressure(TANK3);
    memory.copv_press = read_pressure(COPV);

    #ifdef DEBUG
    if (time - memory.time_msg >= LED_TIMEOUT) {
        Serial.print("TANK1 Temp: ");
        Serial.println(memory.tank1_temp);
        Serial.print("TANK1 Press: ");
        Serial.println(memory.tank1_press);
        Serial.print("TANK2 Temp: ");
        Serial.println(memory.tank2_temp);
        Serial.print("TANK2 Press: ");
        Serial.println(memory.tank2_press);
        Serial.print("tank3 Temp: ");
        Serial.println(memory.tank3_temp);
        Serial.print("tank3 Press: ");
        Serial.println(memory.tank3_press);
        Serial.print("copv Temp: ");
        Serial.println(memory.copv_temp);
        Serial.print("copv Press: ");
        Serial.println(memory.copv_press);
        memory.time_msg = time;
    }
    #endif

}

// ================ testing ================
std::vector<float> DPRComputer::test_read_sensors()
{
    // Test reading sensors
    float T1 = read_temperature(TANK1);
    float T2 = read_temperature(TANK2);
    float T3 = read_temperature(TANK3);
    float T4 = read_temperature(COPV);
    float P1 = read_pressure(TANK1);
    float P2 = read_pressure(TANK2);
    float P3 = read_pressure(TANK3);
    float P4 = read_pressure(COPV);

    return { T1, T2, T3, T4, P1, P2, P3, P4 };
}

void DPRComputer::stress_test(int cycles, int valve)
{
    int count_cycles = 0;
    bool valve_open = false;

    while (count_cycles < cycles)
    {
        if (valve_open) {
            close_valve(valve);
            status_led(OFF);
            valve_open = false;
        } else {
            open_valve(valve);
            count_cycles++;
            status_led(GREEN);
            valve_open = true;
        }
        delay(250);
    }
    Serial.print("Stress test completed. Iterations: ");
    Serial.println(count_cycles);
}

void DPRComputer::test_valves()
{
    // Test opening and closing valves
    open_valve(DPR);
    Serial.println("DPR valve opened");
    delay(500);
    open_valve(VENT1);
    Serial.println("VENT1 valve opened");
    delay(1000);
    open_valve(VENT2);
    Serial.println("VENT2 valve opened");
    delay(500);
    close_valve(DPR);
    Serial.println("DPR valve closed");
    delay(500);
    close_valve(VENT1);
    Serial.println("VENT1 valve closed");
    delay(500);
    close_valve(VENT2);
    Serial.println("VENT2 valve closed");
}


void status_led(RGBColor color) {
    digitalWrite(RGB_RED, color.red);
    digitalWrite(RGB_GREEN, color.green);
    digitalWrite(RGB_BLUE, color.blue);
}

void turn_on_sequence()
{
  digitalWrite(LED_BUILTIN, HIGH);

  status_led(BLUE);
  delay(500);
  status_led(GREEN);
  delay(500);
  status_led(RED);
  delay(500);
  status_led(WHITE);
  tone(BUZZER, 440, 1000);
  delay(1000);
  noTone(BUZZER);
  status_led(OFF);

  digitalWrite(LED_BUILTIN, LOW);
}
