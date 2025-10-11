// 
#include <Arduino.h>
#include <Wire.h>
#include "DPRComputer.h"
#include "wiring.h"

// // Define I2C slave address for Raspberry Pi
// #define SLAVE_ADDR 0x08

DPRComputer computer(MANUAL);

// Variables for communication
volatile uint8_t received_buffer[4];
volatile uint8_t received_command = 0;

volatile float resp_val_float = 0.0f; // Hardcoded response
volatile uint32_t resp_val_int = 0x00; // Response value to be sent back
volatile bool is_resp_int = false; // Flag to indicate if an int response has been sent

// I2C receive handler
void receiveEvent(int numBytes) {
  // Clear buffer
  for (int i = 0; i < 4; ++i) received_buffer[i] = 0;

  // Serial.print("Received I2C command, nb bytes:");
  // Serial.println(numBytes);

  int bytesRead = 0;
  if (numBytes >= 1) {
    if (Wire1.available()) {
      received_command = Wire1.read(); // Read the command
      bytesRead++;
    }

    Serial.print("Received command: ");
    Serial.println(received_command);

    if (numBytes == 1) { return; } // If only command is received, exit
    
    for (int i = 0; i < 4 && bytesRead < numBytes && Wire1.available(); ++i) {
      received_buffer[i] = Wire1.read();
      bytesRead++;
    }

    Serial.print("Received I2C bytes: 0x");
    for (int i = 0; i < 4; ++i) {
      Serial.print(received_buffer[i], HEX);
      if (i < 3) Serial.print(", ");
    }
    // Serial.println(); 

    
    // Set responseValue according to command, but do not write here
    switch (received_command) {
      case AV_NET_DPR_TIMESTAMP: {
        // Serial.println("Received AV_NET_DPR_TIMESTAMP command");
        // responseValue = millis(); // Respond with current timestamp
        status_led(WHITE);
        break;
      }
    

      case AV_NET_DPR_PRESSURIZE: {
          status_led(GREEN);
          dpr_memory_t memory = computer.get_memory();
          uint8_t cmd_pressurize = received_buffer[0];
          if (cmd_pressurize == AV_NET_CMD_ON) {
            if (memory.state == MANUAL) {
              #ifdef DPR_LOX
                computer.open_valve(VN); // Activate Heating pad
              #endif
              computer.set_state(INITIALIZE_PRESSURIZATION);
            }
          }
          else if (cmd_pressurize == AV_NET_CMD_OFF) {
            if (memory.state == REGULATION) {
              #ifdef DPR_LOX
                computer.close_valve(VN); // Deactivate Heating pad
              #endif
              computer.set_state(PRESSURIZATION_OFF);
            }
          }
          // Serial.println("Received AV_NET_DPR_PRESSURIZE command");
          break;
        }

      case AV_NET_DPR_ABORT: {
        uint8_t cmd_abort = received_buffer[0];
        if (cmd_abort == AV_NET_CMD_ON) {
          computer.set_state(ABORT_IN_FLIGHT);
        }
        else if (cmd_abort == AV_NET_CMD_OFF) {
          computer.set_state(ABORT_ON_GROUND);
        }
        // Serial.println("Received AV_NET_DPR_ABORT command");
        break;
      }

      case AV_NET_DPR_VALVES_STATE: {
        status_led(PURPLE);
        if (computer.get_memory().state == ABORT_ON_GROUND || computer.get_memory().state == MANUAL) {
          computer.set_state(MANUAL);
          dpr_memory_t memory = computer.get_memory();
          // Serial.println("Received AV_NET_DPR_VALVES_STATE command");
          uint32_t res;
          memcpy(&res, const_cast<const uint8_t*>(received_buffer), sizeof(res));
          uint8_t valves_vx = received_buffer[0];
          uint8_t valves_pn = received_buffer[1];
          uint8_t valves_vn = received_buffer[2];

          if (!memory.heating_pad_state) {
            computer.activate_heating_pad(true);
            computer.open_valve(VN); // Activate Heating pad
          }

          if (valves_pn == AV_NET_CMD_ON) {
            computer.actuate_valve(PN);
            status_led(GREEN);
          } else if (valves_pn == AV_NET_CMD_OFF) {
            computer.deactuate_valve(PN);
            // status_led(ORANGE);
          } else {
            status_led(RED);
          }

          if (valves_vx == AV_NET_CMD_ON) {
            computer.actuate_valve(VX);
            status_led(GREEN);
          } else if (valves_vx == AV_NET_CMD_OFF) {
            computer.deactuate_valve(VX);
            // status_led(ORANGE);
          } else {
            status_led(RED);
          }

          if (valves_vn == AV_NET_CMD_ON) {
            computer.actuate_valve(VN);
          } else if (valves_vn == AV_NET_CMD_OFF) {
            computer.deactuate_valve(VN);
          } else {
            status_led(RED);
          }
        }
        break;
      }
      case AV_NET_DPR_PASSIVATE: {
        // Serial.println("Received AV_NET_DPR_PASSIVATE command");
        status_led(TEAL);
        dpr_memory_t memory = computer.get_memory();
        if (memory.state == PRESSURIZATION_OFF) {
          computer.set_state(INITIALIZE_PASSIVATION);
        }
      }
        break;

      case AV_NET_DPR_RESET : {
        computer.reset_dpr();
        break;
      }

      default:
        // Serial.println("Unknown command received");
        break;
    }
    // Serial.println("End of command processing");
  
    // clear I2C buffer for later communication
    Wire1.flush();
  }
}

// I2C request handler
void requestEvent() {

  if (Wire1.available()) {
    received_command = Wire1.read(); // Read the command
  }

  Serial.print("(requestEvent) Received command: ");
  Serial.println(received_command);

  dpr_memory_t memory = computer.get_memory();

  switch (received_command)
  {

  case AV_NET_DPR_P_XTA:
    // Serial.println("Received AV_NET_DPR_P_TANK1 command");
    resp_val_float = computer.filterTankPressure();
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_T_XTA:
    // Serial.println("Received AV_NET_DPR_T_TANK1 command");
    resp_val_float = computer.filterTankTemp();
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_P_NCO:
    // Serial.println("Received AV_NET_DPR_P_COPV command");
    resp_val_float = memory.copv_press;
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_T_NCO:
    // Serial.println("Received AV_NET_DPR_T_COPV command");
    resp_val_float = memory.copv_temp;
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_T_COPV_EXT:
  case AV_NET_DPR_T_FLS_80:
    // Serial.println("Received AV_NET_DPR_T_EXT_COPV command");
    resp_val_float = memory.t_ein_temp; // Currently no external sensor, return COPV temp
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_T_FLS_90:
  case AV_NET_DPR_T_FLS_50:
      // Serial.println("Received AV_NET_DPR_T_FLS_EXT_ULH command");  
      resp_val_float = memory.t_oin_temp; // Currently no external sensor, return COPV temp
      is_resp_int = false; // Ensure we are sending a float response
      break;

  case AV_NET_DPR_VALVES_STATE: {
      status_led(GREEN);
      // Serial.println("Received AV_NET_PRB_VALVES_STATE read command");

      uint8_t response_PN = (memory.PN_state) ? AV_NET_CMD_ON : AV_NET_CMD_OFF;
      uint8_t response_VX = (memory.VX_state) ? AV_NET_CMD_ON : AV_NET_CMD_OFF;
      #ifdef DPR_LOX
      uint8_t response_VN = 0;
      #else 
      uint8_t response_VN = (memory.VN_state) ? AV_NET_CMD_ON : AV_NET_CMD_OFF;
      #endif

      // responseValue = 0; // Reset responseValue
      resp_val_int = (response_VN << 16) | (response_PN << 8) | response_VX;
      is_resp_int = true;
      break;
    }
  
  default:
    break;
  }

  if (is_resp_int) {
    Wire1.write((uint8_t*)&resp_val_int, AV_NET_XFER_SIZE);
    // Serial.print("Sent int response: ");
    // Serial.println(resp_val_int);
  } else {
    Wire1.write((uint8_t*)&resp_val_float, AV_NET_XFER_SIZE);
    // Serial.print("Sent float response: ");
    // Serial.println(resp_val_float);
  }
  Wire1.flush(); // Ensure the data is sent immediately
}

void setup() {

  //PIN configuration
  pinMode(PN, OUTPUT);
  pinMode(VX, OUTPUT);
  pinMode(VN, OUTPUT);

  pinMode(RESET, OUTPUT);
  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLUE, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // initialize valves in normal state
  digitalWrite(VX, LOW);
  digitalWrite(PN, LOW);
  digitalWrite(VN, LOW);

  // Activate MUX
  digitalWrite(RESET, HIGH);

  // I2C with Raspberry Pi (use default Wire)
  #ifdef DPR_LOX
  Wire1.begin(AV_NET_ADDR_DPR_LOX);       // Set as I2C slave
  #else 
  Wire1.begin(AV_NET_ADDR_DPR_ETH);
  #endif
  Wire1.onReceive(receiveEvent); // Register receive handler
  Wire1.onRequest(requestEvent); // Register request handler

  // Begin I2C communication with sensors
  Wire2.begin();

  // Analog sensor precision
  analogReadResolution(12);

  Serial.begin(115200); // For debugging
  // Serial.println("PN Computer started");

  turn_on_sequence();

  // Serial.println("PN Computer setup done");
}

// PTE7300_I2C mySensor; // attach sensor
// int16_t DSP_T1;
// int sleep_time = 100;

void loop() {
  computer.update(millis()); // Update the DPRComputer state machine
  
  // delay(2000);  // Wait 2 seconds before next scan
}