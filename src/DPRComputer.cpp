// Last update: 11/03/2025
#include "DPRComputer.h"
#include "Wire.h"


#define TEST_WITHOUT_PRESSURE

DPRComputer::DPRComputer(DPR_FSM init_state)
{
    memory.state = init_state;
    memory.status_led = false;
    memory.time_led = 0;
    memory_controller.tankPressure = 0.0;
    memory_controller.copvPressure = 0.0;
    memory_controller.initialCopvPressure = 0.0;
#ifdef DPR_LOX
    memory_controller.limitPressure = LIMIT_PRESSURE_LOX;
#else
    memory_controller.limitPressure = LIMIT_PRESSURE_ETH;
#endif
    memory_controller.rampedPressure = 0.0;
    memory_controller.fullScalePressure = 1000;
    memory_controller.error = 0.0;
    memory_controller.lastError = 0.0;
    memory_controller.integral = 0.0;
    memory_controller.derivative = 0.0;
    memory_controller.kp = KP;
    memory_controller.ki = KI;
    memory_controller.kd = KD;
    memory_controller.dutyRatio = 0.0;
    memory_controller.dutyTime = 0.0;
    memory_controller.startTime = 0;
    memory_controller.lastTime = 0;
    memory_controller.controlPeriod = 100;
}

DPRComputer::~DPRComputer() {}

// ========= valve and motor control =========
void DPRComputer::open_valve(int valve)
{
    switch (valve)
    {
    case PN:
        memory.PN_state = true;
        digitalWrite(valve, HIGH);
        break;
    case VX:
        memory.VX_state = false;
        digitalWrite(valve, LOW);
        break;
    case VN:
        memory.VN_state = true;
        digitalWrite(valve, HIGH);
        break;
    default:
        break;
    }
}

void DPRComputer::close_valve(int valve)
{
    switch (valve)
    {
    case PN:
        memory.PN_state = false;
        digitalWrite(valve, LOW);
        break;
    case VX:
        memory.VX_state = true;
        digitalWrite(valve, HIGH);
        break;
    case VN:
        memory.VN_state = false;
        digitalWrite(valve, LOW);
        break;
    default:
        break;
    }
}


// ========= valve and motor control =========
void DPRComputer::actuate_valve(int valve)
{
    switch (valve)
    {
    case PN:
        memory.PN_state = true;
        break;
    case VX:
        memory.VX_state = true;
        break;
    case VN:
        memory.VN_state = true;
        break;

    default:
        break;
    }
    digitalWrite(valve, HIGH);
}

void DPRComputer::deactuate_valve(int valve)
{
    switch (valve)
    {
    case PN:
        memory.PN_state = false;
        break;
    case VX:
        memory.VX_state = false;
        break;
    case VN:
        memory.VN_state = false;
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
        press = ((DSP_S - (-16000.0)) * (100.0) / (16000.0 - (-16000.0)));
    break;

    case COPV:
        // read 400 bar
        DSP_S = my_sensor.readDSP_S();
        press = ((DSP_S - (-16000.0)) * (400.0) / (16000.0 - (-16000.0))); // Change formula for 400 bar
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
void DPRComputer::set_state(DPR_FSM new_state) { memory.state = new_state; }


bool muxSelect(uint8_t ch) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(ch);
    return Wire.endTransmission() == 0; // true if ACKed
}


// ========== FSM ===========
void DPRComputer::update(int time)
{
    // Update the state machine
    // Declare variables outside the switch to avoid bypassing initialization

    switch (memory.state)
    {
        case INITIALIZE_PRESSURIZATION:
            initialize();
            memory.state = PRESSURIZATION;
            break;

        case PRESSURIZATION:
            pressurization();
            actuate();

            if (!memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(RED);
                memory.time_led = time;
                memory.status_led = true;
            } else if (memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(OFF);
                memory.time_led = time;
                memory.status_led = false;
            }
            break;

        case INITIALIZE_REGULATION:
            initialize();
            memory.state = REGULATION;
            break;

        case REGULATION:
            regulation();
            actuate();

            if (!memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(BLUE);
                memory.time_led = time;
                memory.status_led = true;
            } else if (memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(OFF);
                memory.time_led = time;
                memory.status_led = false;
            }
            break;

        case INITIALIZE_PASSIVATION:
            initialize();
            memory.state = PASSIVATION;
            break;

        case PASSIVATION:
            if (millis() - memory_controller.startTime >= DELAY_VENT_N2) {
                open_valve(VX);
            }

        case ABORT_ON_GROUND:
            open_valve(VX);
            close_valve(PN);
            open_valve(VN);

            if (!memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(PURPLE);
                memory.time_led = time;
                memory.status_led = true;
            } else if (memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(OFF);
                memory.time_led = time;
                memory.status_led = false;
            }
            break;

        case ABORT_IN_FLIGHT:
            open_valve(VX);
            close_valve(PN);
            open_valve(VN);

            if (!memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(ORANGE);
                memory.time_led = time;
                memory.status_led = true;
            } else if (memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(OFF);
                memory.time_led = time;
                memory.status_led = false;
            }
            break;
        
        case MANUAL:
            if (!memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(GREEN);
                memory.time_led = time;
                memory.status_led = true;
            } else if (memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(OFF);
                memory.time_led = time;
                memory.status_led = false;
            }
            break;

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

//==============================================================================================================
void DPRComputer::initialize() {
    // Valves in regulation state
    close_valve(VX);
    close_valve(VN);
    close_valve(PN);

    // Read initial COPV pressure
    memory_controller.initialCopvPressure = memory.copv_press;

    // initialize ramped pressure
    memory_controller.rampedPressure = 0.0;

    // initialize pid variables
    memory_controller.integral = 0.0;
    memory_controller.derivative = 0.0;
    memory_controller.error = 0.0;

    memory_controller.startTime = 0;
    memory_controller.lastTime = 0;
}

//==============================================================================================================
void DPRComputer::regulation() {
    if (millis() - memory_controller.lastTime > memory_controller.controlPeriod) {
        memory_controller.lastTime = millis();
        memory_controller.tankPressure = filterTankPressure();
        memory_controller.copvPressure = memory.copv_press;
        memory_controller.error = memory_controller.limitPressure - memory_controller.tankPressure;
        memory_controller.fullScalePressure = computeFullScale();
        pid();
        memory_controller.lastError = memory_controller.error;
        memory_controller.dutyTime = memory_controller.dutyRatio * memory_controller.controlPeriod;
    }
}

//==============================================================================================================
void DPRComputer::pressurization() {
    if (millis() - memory_controller.lastTime > memory_controller.controlPeriod) {
        memory_controller.lastTime = millis();
        if (memory_controller.rampedPressure < memory_controller.limitPressure) {
            memory_controller.rampedPressure = (memory_controller.limitPressure / 10000)*millis() - (memory_controller.limitPressure*memory_controller.startTime)/10000;
        }
        else {
            memory_controller.rampedPressure = memory_controller.limitPressure;
        }
        memory_controller.tankPressure = filterTankPressure();
        memory_controller.copvPressure = memory.copv_press;
        memory_controller.error = memory_controller.rampedPressure - memory_controller.tankPressure;
        memory_controller.fullScalePressure = computeFullScale();
        pid();
        memory_controller.lastError = memory_controller.error;
        memory_controller.dutyTime = memory_controller.dutyRatio * memory_controller.controlPeriod;

        if (millis() - memory_controller.startTime > 15000) {
            memory.state = INITIALIZE_REGULATION;
        }
    }
}


//==============================================================================================================
float DPRComputer::filterTankPressure() {
    float pressure1 = memory.tank1_press;
    float pressure2 = memory.tank2_press;
    float pressure3 = memory.tank3_press;

    if ((abs(pressure1 - pressure2) > abs(pressure2 - pressure3)) && (abs(pressure1 - pressure3) > abs(pressure2 - pressure3))) {
        return 0.5*(pressure2 + pressure3);
    }
    else if ((abs(pressure2 - pressure1) > abs(pressure1 - pressure3)) && (abs(pressure2 - pressure3) > abs(pressure1 - pressure3))) {
        return 0.5*(pressure1 + pressure3);
    }
    else return 0.5*(pressure1 + pressure2);
}

float DPRComputer::filterTankTemp() {
    float temp1 = memory.tank1_temp;
    float temp2 = memory.tank2_temp;
    float temp3 = memory.tank3_temp;

    if ((abs(temp1 - temp2) > abs(temp2 - temp3)) && (abs(temp1 - temp3) > abs(temp2 - temp3))) {
        return 0.5*(temp2 + temp3);
    }
    else if ((abs(temp2 - temp1) > abs(temp1 - temp3)) && (abs(temp2 - temp3) > abs(temp1 - temp3))) {
        return 0.5*(temp1 + temp3);
    }
    else return 0.5*(temp1 + temp2);
}

float DPRComputer::computeFullScale() {
    return (memory_controller.limitPressure+(memory_controller.copvPressure-memory_controller.initialCopvPressure)*(memory_controller.limitPressure/(memory_controller.initialCopvPressure-memory_controller.limitPressure)));
}

//==============================================================================================================
void DPRComputer::pid() {
    float correction = 0.0;

    // integral contribution
    memory_controller.integral += memory_controller.controlPeriod * 0.001 * memory_controller.error;

    // derivative contribution
    memory_controller.derivative = ((memory_controller.error - memory_controller.lastError)*1000) / memory_controller.controlPeriod;

    // PID correction
    correction = memory_controller.kp*memory_controller.error + memory_controller.ki*memory_controller.integral + memory_controller.kd*memory_controller.derivative;

    if (correction > memory_controller.fullScalePressure) {
        memory_controller.dutyRatio = 1.0;
    }
    else if (correction < 0) {
        memory_controller.dutyRatio = 0.0;
        memory_controller.integral = 0.0;
    }
    else { // normalisation
        memory_controller.dutyRatio = correction / memory_controller.fullScalePressure;
    }
}

//==============================================================================================================
void DPRComputer::actuate() {
    if (millis() - memory_controller.lastTime < memory_controller.dutyTime) {
        open_valve(PN);
    }
    else {
        close_valve(PN);
    }
}

// ================ testing ================
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
