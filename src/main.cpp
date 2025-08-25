// 
#include <Arduino.h>
#include <Wire.h>
#include "DPRComputer.h"
#include "wiring.h"

// // Define I2C slave address for Raspberry Pi
// #define SLAVE_ADDR 0x08

DPRComputer computer(IDLE);

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

  Serial.print("Received I2C command, nb bytes:");
  Serial.println(numBytes);

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
    Serial.println(); 

    
    // Set responseValue according to command, but do not write here
    switch (received_command) {
      case AV_NET_DPR_TIMESTAMP: {
        Serial.println("Received AV_NET_DPR_TIMESTAMP command");
        // responseValue = millis(); // Respond with current timestamp
        status_led(WHITE);
      }

      case AV_NET_DPR_WAKE_UP:
        Serial.println("Received AV_NET_DPR_WAKE_UP command");
        status_led(TEAL);
        break;

      case AV_NET_DPR_IS_WOKEN_UP:
        Serial.println("Received AV_NET_DPR_IS_WOKEN_UP command");
        status_led(PURPLE);
        break;

      case AV_NET_DPR_PRESSURIZE:
        status_led(GREEN);
        Serial.println("Received AV_NET_DPR_PRESSURIZE command");
        break;

      case AV_NET_DPR_ABORT:
        Serial.println("Received AV_NET_DPR_ABORT command");
        // responseValue = computer.read_pressure(P_OIN);
        break;

      case AV_NET_DPR_VALVES_STATE: {
        status_led(PURPLE);
        Serial.println("Received AV_NET_DPR_VALVES_STATE command");
        uint32_t res;
        memcpy(&res, const_cast<const uint8_t*>(received_buffer), sizeof(res));
        uint8_t valves_vent1 = received_buffer[0];
        uint8_t valves_dpr = received_buffer[1];
        uint8_t valves_vent2 = received_buffer[2];
   

        if (valves_dpr == AV_NET_CMD_ON) {
          computer.open_valve(DPR);
          status_led(GREEN);
        } else if (valves_dpr == AV_NET_CMD_OFF) {
          computer.close_valve(DPR);
          status_led(ORANGE);
        } else {
          status_led(RED);
        }

        if (valves_vent1 == AV_NET_CMD_ON) {
          computer.open_valve(VENT1);
          status_led(GREEN);
        } else if (valves_vent1 == AV_NET_CMD_OFF) {
          computer.close_valve(VENT1);
          status_led(ORANGE);
        } else {
          status_led(RED);
        }

        if (valves_vent2 == AV_NET_CMD_ON) {
          computer.open_valve(VENT2);
          status_led(GREEN);
        } else if (valves_vent2 == AV_NET_CMD_OFF) {
          computer.close_valve(VENT2);
          status_led(ORANGE);
        } else {
          status_led(RED);
        }
        break;
      }

      case AV_NET_DPR_NB_REG:
        Serial.println("Received AV_NET_DPR_NB_REG command");
        // responseValue = computer.get_nb_reg();
        break;

      default:
        Serial.println("Unknown command received");
        break;
    }
    Serial.println("End of command processing");
  
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
  case AV_NET_DPR_L_XTA:
    Serial.println("Received AV_NET_DPR_L_XTA command");
    resp_val_float = 0.0f;
    is_resp_int = false; // Ensure we are sending a float response

    break;

    case AV_NET_DPR_T_XTA:
    Serial.println("Received AV_NET_DPR_T_XTA command");
    resp_val_float = memory.xta_temp;
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_P_XTA:
    Serial.println("Received AV_NET_DPR_P_XTA command");
    resp_val_float = memory.xta_press;
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_P_NCO:
    Serial.println("Received AV_NET_DPR_P_NCO command");
    resp_val_float = memory.nco_press;
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_T_NCO:
    Serial.println("Received AV_NET_DPR_T_NCO command");
    resp_val_float = memory.nco_temp;
    is_resp_int = false; // Ensure we are sending a float response
    break;

  case AV_NET_DPR_VALVES_STATE: {
      status_led(GREEN);
      Serial.println("Received AV_NET_PRB_VALVES_STATE read command");
      bool DPR_state = memory.DPR_state;
      bool VENT1_state = memory.VENT1_state;
      bool VENT2_state = memory.VENT2_state;

      uint8_t response_DPR = (DPR_state) ? AV_NET_CMD_ON : AV_NET_CMD_OFF;
      uint8_t response_VENT1 = (VENT1_state) ? AV_NET_CMD_ON : AV_NET_CMD_OFF;
      uint8_t response_VENT2 = (VENT2_state) ? AV_NET_CMD_ON : AV_NET_CMD_OFF;

      // responseValue = 0; // Reset responseValue
      resp_val_int = (response_VENT2 << 16) | (response_DPR << 8) | response_VENT1;
      is_resp_int = true;
      break;
    }
  
  default:
    break;
  }

  if (is_resp_int) {
    Wire1.write((uint8_t*)&resp_val_int, AV_NET_XFER_SIZE);
  } else {
    Wire1.write((uint8_t*)&resp_val_float, AV_NET_XFER_SIZE);
  }
  Wire1.flush(); // Ensure the data is sent immediately
}

void setup() {

  //PIN configuration
  pinMode(DPR, OUTPUT);
  pinMode(VENT1, OUTPUT);
  pinMode(VENT2, OUTPUT);

  pinMode(RESET, OUTPUT);
  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLUE, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Activate MUX
  digitalWrite(RESET, HIGH);

  // I2C with Raspberry Pi (use default Wire)
  Wire1.begin(AV_NET_ADDR_DPR_LOX);       // Set as I2C slave
  Wire1.onReceive(receiveEvent); // Register receive handler
  Wire1.onRequest(requestEvent); // Register request handler

  // Begin I2C communication with sensors
  Wire.begin();

  // Analog sensor precision
  analogReadResolution(12);

  Serial.begin(115200); // For debugging
  Serial.println("DPR Computer started");

  turn_on_sequence();

  Serial.println("DPR Computer setup done");
}

PTE7300_I2C mySensor; // attach sensor
int16_t DSP_T1;
int time;
int sleep_time = 100;

void loop() {
  computer.update(millis()); // Update the DPRComputer state machine
  
  // delay(2000);  // Wait 2 seconds before next scan
}