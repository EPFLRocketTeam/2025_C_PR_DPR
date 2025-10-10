// Last update: 11/03/2025
#include "DPRComputer.h"
#include "Wire.h"
#define TEST_WITHOUT_PRESSURE

//======================================================================================================================
DPRComputer::DPRComputer(DPR_FSM init_state)
{
    memory.state = init_state;
    memory.status_led = false;
    memory.time_led = 0;
    memory_controller.tankPressure = 0.0;
    memory_controller.copvPressure = 0.0;
    memory_controller.initialCopvPressure = 0.0;
    memory_controller.tank1_offset = 0.0;
    memory_controller.tank1_offset = 0.0;
    memory_controller.tank1_offset = 0.0;
    memory.PN_state = false;
    memory.VX_state = false;
    memory.VN_state = false;
    memory.max_time_passivate = 0;
    memory.heating_pad_state = false;
    memory.t_oin_temp = 0.0;
    memory.t_ein_temp = 0.0;
#ifdef DPR_LOX
    memory_controller.limitPressure = PRESSURIZATION_OX_SET_PRESSURE;
#else
    memory_controller.limitPressure = PRESSURIZATION_FUEL_SET_PRESSURE;
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
    memory_controller.controlPeriod = CONTROL_PERIOD;
}

//======================================================================================================================
DPRComputer::~DPRComputer() {}

//======================================================================================================================
void DPRComputer::reset_dpr() {
    memory.state = MANUAL;
    memory.status_led = false;
    memory.time_led = 0;
    memory_controller.tankPressure = 0.0;
    memory_controller.copvPressure = 0.0;
    memory_controller.initialCopvPressure = 0.0;
    memory_controller.tank1_offset = 0.0;
    memory_controller.tank1_offset = 0.0;
    memory_controller.tank1_offset = 0.0;
    memory.PN_state = false;
    memory.VX_state = false;
    memory.VN_state = false;
    memory.max_time_passivate = 0;
    memory.heating_pad_state = false;
    memory.t_oin_temp = 0.0;
    memory.t_ein_temp = 0.0;
#ifdef DPR_LOX
    memory_controller.limitPressure = PRESSURIZATION_OX_SET_PRESSURE;
#else
    memory_controller.limitPressure = PRESSURIZATION_FUEL_SET_PRESSURE;
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
    memory_controller.controlPeriod = CONTROL_PERIOD;
}

// =========================== valve and motor control ===========================

/**
 * @brief Opens the specified valve and updates its state in memory.
 *
 * This function takes an integer identifier for a valve and performs the following actions:
 * - For the PN valve: Sets the PN state to true in memory and sets the corresponding digital pin HIGH.
 * - For the VX valve: Sets the VX state to false in memory and sets the corresponding digital pin LOW.
 * - For the VN valve: Sets the VN state to true in memory and sets the corresponding digital pin HIGH.
 * - For any other value: No action is taken.
 *
 * @param valve The identifier of the valve to open (e.g., PN, VX, VN).
 */
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


/**
 * @brief Closes the specified valve by updating its state and controlling the hardware pin.
 *
 * This function takes an integer identifier for a valve and performs the following actions:
 * - For the PN valve: Sets its state to closed (false) in memory and sets the corresponding hardware pin LOW.
 * - For the VX valve: Sets its state to open (true) in memory and sets the corresponding hardware pin HIGH.
 * - For the VN valve: Sets its state to closed (false) in memory and sets the corresponding hardware pin LOW.
 * - For any other value, no action is taken.
 *
 * @param valve The identifier of the valve to close (e.g., PN, VX, VN).
 *
 * @note The function assumes that the valve identifiers (PN, VX, VN) are defined elsewhere,
 *       and that the memory object and digitalWrite function are available in the class context.
 */
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


/**
 * @brief Actuates the specified valve by setting its state in memory and sending a digital HIGH signal.
 *
 * This function updates the internal memory state for the given valve type (PN, VX, or VN)
 * and sets the corresponding hardware pin to HIGH using digitalWrite.
 * If the valve type does not match any known type, no memory state is updated,
 * but digitalWrite is still called with the provided valve identifier.
 *
 * @param valve The identifier of the valve to actuate. Should be one of PN, VX, or VN.
 */
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


/**
 * @brief Deactivates the specified valve by updating its state in memory and setting its output to LOW.
 *
 * This function sets the internal memory state of the given valve to false (deactivated)
 * and sends a LOW signal to the corresponding hardware pin using digitalWrite.
 *
 * @param valve The identifier of the valve to be deactivated. Valid values are PN, VX, and VN.
 */
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


// =========================== sensor reading ===========================
/**
 * @brief Reads the pressure value from the specified sensor.
 *
 * This function selects the given sensor using the multiplexer, reads its digital signal,
 * and converts it to a pressure value in bar. The conversion formula depends on the sensor type:
 * - For TANK1, TANK2, and TANK3, the output is scaled to a 0-100 bar range.
 * - For COPV, the output is scaled to a 0-400 bar range.
 *
 * @param sensor The identifier of the sensor to read (e.g., TANK1, TANK2, TANK3, COPV).
 * @return The pressure value in bar as a float. Returns 0.0 if the sensor type is not recognized.
 */
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


/**
 * @brief Reads the temperature from the specified sensor.
 *
 * Selects the given sensor using the multiplexer, reads the raw DSP_T value,
 * and converts it to a temperature in degrees Celsius using the formula:
 * temp = DSP_T * 82.5 / 16000 + 42.5.
 *
 * @param sensor The index or identifier of the sensor to read from.
 * @return The temperature in degrees Celsius as a float.
 */
float DPRComputer::read_temperature(int sensor)
{
    //read temperature
    float temp = 0.0;
    int DSP_T = 0;
    int value = 0;
    float voltage = 0.0;
    float resistance_pt1000 = 0.0;
    bool I2C_sensor = false;

    switch (sensor)
    {
        case T_OIN:
        case T_EIN:
            //read analog temperature
            value = analogRead(sensor);
            voltage = (value * 3.3) / 4095.0; // Assuming a 12-bit ADC and 3V3 reference
            resistance_pt1000 = (voltage * 1100.0)/(3.3 - voltage); // formula from a resistor divider
            temp = (resistance_pt1000-1000)/3.85;
        break;

        case TANK1:  //TANK1
        case TANK2:  // TANK2
        case TANK3:
        case COPV:
            muxSelect(sensor);
            I2C_sensor = true;
        break;
    
    default:
        break;
    }

    if (!I2C_sensor) return temp;
    
    DSP_T = my_sensor.readDSP_T();
    temp = DSP_T * 82.5 / 16000 + 42.5;
    
    return temp;
}

bool DPRComputer::check_avail(int sensor) {
    if(!muxSelect(sensor)) return false;
    Wire2.beginTransmission(SENS_ADDR);
    return Wire2.endTransmission() == 0;
}


// ============================ getter ====================================
dpr_memory_t DPRComputer::get_memory() { return memory; }

// ========= setter =========
void DPRComputer::set_state(DPR_FSM new_state) { memory.state = new_state; }
void DPRComputer::activate_heating_pad(bool state) { memory.heating_pad_state = state; }


// =========================== I2C multiplexer ===========================
/**
 * @brief Selects a channel on the I2C multiplexer.
 *
 * Sends a command over I2C to the multiplexer at address MUX_ADDR to select the specified channel.
 *
 * @param ch The channel number to select on the multiplexer.
 * @return true if the multiplexer acknowledges the command (ACK received), false otherwise.
 */
bool muxSelect(uint8_t ch) {
    Wire2.beginTransmission(MUX_ADDR);
    Wire2.write(ch);
    return Wire2.endTransmission() == 0; // true if ACKed
}

// ================================ FSM =================================
/**
 * @brief Updates the DPRComputer state machine and handles system operations.
 *
 * This function is called periodically to update the internal state of the DPRComputer
 * based on the current state, elapsed time, and sensor readings. It manages state transitions,
 * actuates valves, updates status LEDs, and reads temperature and pressure sensors.
 * In debug mode, it also prints sensor values to the serial output.
 *
 * @param time The current system time in milliseconds.
 */
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
                status_led(ORANGE);
                memory.time_led = time;
                memory.status_led = true;
            } else if (memory.status_led && time - memory.time_led >= LED_TIMEOUT) {
                status_led(OFF);
                memory.time_led = time;
                memory.status_led = false;
            }
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

        case PRESSURIZATION_OFF:
            if (memory.max_time_passivate == 0) {
                memory.max_time_passivate = millis();
            }
            close_valve(VX);
            close_valve(PN);
            close_valve(VN);
            if (millis() - memory.max_time_passivate >= PASSIVATION_DELAY_NO_COM_DPR) {
                memory.state = INITIALIZE_PASSIVATION;
            }
            break;

        case INITIALIZE_PASSIVATION:
            memory.max_time_passivate = 0;
            initialize();
            memory.state = PASSIVATION;
            break;

        case PASSIVATION:
            if (millis() - memory_controller.startTime < PASSIVATION_COPV_DURATION) {
                //open_valve(VN);
        
                open_valve(VX);
                #if DPR_LOX
                open_valve(PN);
                #else
                close_valve(PN);
                #endif
            }
            else {
                //deactuate_valve(VN);
                deactuate_valve(VX);
                deactuate_valve(PN);
            }
            break;

        case ABORT_ON_GROUND:
            memory.max_time_passivate = 0;
            open_valve(VX);
            close_valve(PN);
            close_valve(VN);
            memory.heating_pad_state = false;

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
            memory.max_time_passivate = 0;
            open_valve(VX);
            #if DPR_LOX
            open_valve(PN);
            #else
            close_valve(PN);
            #endif

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

    Serial.print("VX state: ");
    Serial.print(memory.VX_state);
    Serial.print("     FSM: ");
    Serial.println(memory.state);


    #ifdef PRB_DPR

        __sensor1_available = check_avail(TANK1);
        __sensor2_available  = check_avail(TANK2);

        memory.tank1_temp = read_temperature(TANK1);
        memory.tank2_temp = read_temperature(TANK2);
        memory.copv_temp = read_temperature(COPV);
        memory.tank1_press = read_pressure(TANK1);
        memory.tank2_press = read_pressure(TANK2);
        memory.copv_press = read_pressure(COPV);
        memory.t_oin_temp = read_temperature(T_OIN);
        memory.t_ein_temp = read_temperature(T_EIN);
    #else
        memory.tank1_temp = read_temperature(TANK1);
        memory.tank2_temp = read_temperature(TANK2);
        memory.tank3_temp = read_temperature(TANK3);
        memory.copv_temp = read_temperature(COPV);
        memory.tank1_press = read_pressure(TANK1);
        memory.tank2_press = read_pressure(TANK2);
        memory.tank3_press = read_pressure(TANK3);
        memory.copv_press = read_pressure(COPV);
    #endif

    #ifdef DEBUG
    if (time - memory.time_msg >= LED_TIMEOUT) {
        Serial.print("TANK1 Temp: ");
        Serial.println(memory.tank1_temp);
        Serial.print("TANK1 Press: ");
        Serial.println(memory.tank1_press);
        Serial.print("TANK1 Avail: ");
        Serial.println(__sensor1_available);
        Serial.print("TANK2 Temp: ");
        Serial.println(memory.tank2_temp);
        Serial.print("TANK2 Press: ");
        Serial.println(memory.tank2_press);
        Serial.print("TANK2 Avail: ");
        Serial.println(__sensor2_available);
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


/**
 * @brief Initializes the DPRComputer system state.
 *
 * This function sets the initial state of the valves to regulation mode by closing
 * the VX, VN, and PN valves. It reads and stores the initial COPV
 * pressure into the memory controller. It also initializes the ramped pressure and PID controller
 * variables (integral, derivative, and error) to zero. Additionally, it records the start time
 * using the current system time and resets the last time variable.
 */
void DPRComputer::initialize() {
    // Valves in regulation state
    close_valve(VX);
    close_valve(PN);
    
    #ifdef DPR_LOX
        if (memory.heating_pad_state) {
            open_valve(VN); // Activate Heating pad
        }
        else {
            close_valve(VN); // Deactivate Heating pad
        }
    #endif

    // Read initial COPV pressure
    memory_controller.initialCopvPressure = memory.copv_press;

    // initialize ramped pressure
    memory_controller.rampedPressure = 0.0;

    // initialize pid variables
    memory_controller.integral = 0.0;
    memory_controller.derivative = 0.0;
    memory_controller.error = 0.0;

    memory_controller.startTime = millis();
    memory_controller.lastTime = 0;
}


/**
 * @brief Performs the regulation loop for the DPRComputer.
 *
 * This function checks if the control period has elapsed since the last regulation step.
 * If so, it updates the last execution time, reads and filters the tank pressure,
 * updates the COPV pressure from memory, computes the pressure error, calculates
 * the full scale pressure, executes the PID controller, stores the last error,
 * and updates the duty time based on the computed duty ratio and control period.
 *
 * The regulation loop is responsible for maintaining the desired tank pressure
 * by adjusting the control output using a PID algorithm.
 */
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


/**
 * @brief Handles the pressurization control logic for the DPR system.
 *
 * This function is responsible for managing the pressurization process by:
 * - Periodically updating control logic based on a defined control period.
 * - Ramping the target pressure up to a specified limit using a linear ramp function.
 * - Transitioning the system state to regulation mode once the ramped pressure reaches the limit.
 * - Filtering and updating the current tank and COPV pressures.
 * - Calculating the pressure error and full-scale pressure for control purposes.
 * - Executing the PID control algorithm to adjust system output.
 * - Updating internal controller memory for error tracking and duty cycle computation.
 *
 * The function should be called regularly (e.g., in a main loop or timer interrupt)
 * to ensure timely control updates.
 */
void DPRComputer::pressurization() {
    if (millis() - memory_controller.lastTime > memory_controller.controlPeriod) {
        memory_controller.lastTime = millis();
        if (memory_controller.rampedPressure < memory_controller.limitPressure) {
            memory_controller.rampedPressure = (memory_controller.limitPressure / RAMP_DELAY)*millis() - (memory_controller.limitPressure*memory_controller.startTime)/RAMP_DELAY;
        }
        else {
            memory.state = REGULATION;
        }
        memory_controller.tankPressure = filterTankPressure();
        memory_controller.copvPressure = memory.copv_press;
        memory_controller.error = memory_controller.rampedPressure - memory_controller.tankPressure;
        memory_controller.fullScalePressure = computeFullScale();
        pid();
        memory_controller.lastError = memory_controller.error;
        memory_controller.dutyTime = memory_controller.dutyRatio * memory_controller.controlPeriod;
    }
}


/**
 * @brief Filters and returns the estimated tank pressure by removing outliers.
 *
 * This function reads the pressures from three tanks and applies an outlier rejection filter.
 * If the @p controller flag is true, it subtracts the corresponding controller offsets from each tank's pressure.
 * It then compares the absolute differences between the tank pressures to identify the outlier.
 * The function returns the average of the two closest pressure readings, effectively filtering out the outlier.
 *
 * @param controller If true, applies controller offsets to the tank pressures before filtering.
 * @return The filtered tank pressure as the average of the two closest pressure readings.
 */
#ifdef PRB_DPR
float DPRComputer::filterTankPressure() {
    float pressure1 = 0.0;
    float pressure2 = 0.0;
    
    pressure1 = memory.tank1_press;
    pressure2 = memory.tank2_press;

    float avg = 0;
    float sens = 0;
    if(__sensor1_available && !isinf(pressure1) && !isnan(pressure1)){
        avg += pressure1;
        sens++;
    }

    if(__sensor2_available && !isinf(pressure2) && !isnan(pressure2)) {
        avg += pressure2;
        sens++;
    }


    if (sens == 0) {
        return 499;
    }
        
    return avg/sens;
}
#else
float DPRComputer::filterTankPressure() {
    float pressure1 = 0.0;
    float pressure2 = 0.0;
    float pressure3 = 0.0;

    pressure1 = memory.tank1_press;
    pressure2 = memory.tank2_press;
    pressure3 = memory.tank3_press;

    if ((abs(pressure1 - pressure2) > abs(pressure2 - pressure3)) && (abs(pressure1 - pressure3) > abs(pressure2 - pressure3))) {
        return 0.5*(pressure2 + pressure3);
    }
    else if ((abs(pressure2 - pressure1) > abs(pressure1 - pressure3)) && (abs(pressure2 - pressure3) > abs(pressure1 - pressure3))) {
        return 0.5*(pressure1 + pressure3);
    }
    else return 0.5*(pressure1 + pressure2);
}
#endif

/**
 * @brief Filters and returns a reliable tank temperature by excluding the most divergent reading.
 *
 * This function takes three tank temperature readings from memory (tank1_temp, tank2_temp, tank3_temp)
 * and computes a filtered value by discarding the reading that differs the most from the other two.
 * It returns the average of the two closest temperature readings, providing a simple outlier rejection
 * to improve measurement reliability.
 *
 * @return The filtered tank temperature as the average of the two closest readings.
 */
#ifdef PRB_DPR
float DPRComputer::filterTankTemp() {
    return memory.tank1_temp;
}
#else

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
#endif

//================================= full scale ========================================
float DPRComputer::computeFullScale() {
    return (memory_controller.limitPressure+(memory_controller.copvPressure-memory_controller.initialCopvPressure)*(memory_controller.limitPressure/(memory_controller.initialCopvPressure-memory_controller.limitPressure)));
}

//===================================== PID ==========================================
/**
 * @brief Computes the PID control output and updates the duty ratio.
 *
 * This function calculates the proportional, integral, and derivative (PID) terms
 * based on the current error and control period stored in the memory_controller.
 * It updates the integral and derivative contributions, computes the total correction,
 * and normalizes the output to determine the appropriate duty ratio for the actuator.
 * The duty ratio is clamped between 0.0 and 1.0 based on the correction value and
 * the full scale pressure. If the correction is negative, the integral term is reset.
 *
 * @note This function assumes that the memory_controller structure contains the necessary
 *       PID parameters (kp, ki, kd), error values, control period, and output limits.
 */
void DPRComputer::pid() {
    float correction = 0.0;

    // integral contribution
    memory_controller.integral += memory_controller.controlPeriod * 0.001 * memory_controller.error;

    // derivative contribution
    memory_controller.derivative = ((memory_controller.error - memory_controller.lastError)*1000) / memory_controller.controlPeriod;

    // PID correction
    correction = memory_controller.kp*memory_controller.error + memory_controller.ki*memory_controller.integral + memory_controller.kd*memory_controller.derivative;

    if (correction < 0 || memory_controller.error < 0) {
        memory_controller.dutyRatio = 0.0;
        memory_controller.integral = 0.0;
    }
    else if (correction > memory_controller.fullScalePressure) {
        memory_controller.dutyRatio = 1.0;
    }
    else { // normalisation
        memory_controller.dutyRatio = correction / memory_controller.fullScalePressure;
    }
}

/**
 * @brief Controls the actuation of a valve based on timing conditions.
 *
 * This function checks if the elapsed time since the last memory controller event
 * is less than the specified duty time. If so, it opens the valve associated with PN.
 * Otherwise, it closes the valve. This mechanism is typically used to implement
 * pulse-width modulation (PWM) or similar timing-based control for the valve.
 */
void DPRComputer::actuate() {
    if (millis() - memory_controller.lastTime < memory_controller.dutyTime) {
        open_valve(PN);
    }
    else {
        close_valve(PN);
    }
}

// =========================== status LED ===========================
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
