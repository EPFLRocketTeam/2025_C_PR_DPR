//
// Created by marin on 08.03.2025.
// Maintained by AV team.
//

/**
 * The communication between microcontrollers inside Firehorn uses I2C protocol.
 * This file defines the I2C device and register addresses for each slave present on the main AV bus.
 * To simplify things, each read/write operation involves 4 bytes of data (32-bit)
 * 
 * Intranet reference: https://docs.google.com/spreadsheets/d/159JLAAPnOQsqaWHLeamU7fWXOPAsgL0OkVv0sn9sXFg/edit?gid=0#gid=0
 */

#ifndef AV_INTRANET_COMMANDS_H
#define AV_INTRANET_COMMANDS_H

/* Device addresses */
#define AV_NET_ADDR_DPR_ETH  0x0E
#define AV_NET_ADDR_DPR_LOX  0x0C
#define AV_NET_ADDR_PRB      0x0F
#define AV_NET_ADDR_TRB      0x0A
#define AV_NET_ADDR_CAM_SEP  0x1C
#define AV_NET_ADDR_CAM_UP   0x2C
#define AV_NET_ADDR_CAM_DOWN 0x3C

/* Valves map */
// DPRs
#define AV_NET_DPR_VALVE_DN_NC  (0xFF << 16) // COPV vent
#define AV_NET_DPR_VALVE_PX_NC  (0xFF << 8)  // Tank pressure regulation
#define AV_NET_DPR_VALVE_VX_NO  (0xFF << 0)  // Tank vent
// PRB
#define AV_NET_PRB_VALVE_MO_BC  (0xFF << 8)
#define AV_NET_PRB_VALVE_ME_B   (0xFF << 0)

/* Pyro channels map */
#define AV_NET_TRB_PYRO3 (0xFF << 16) 
#define AV_NET_TRB_PYRO2 (0xFF << 8)
#define AV_NET_TRB_PYRO1 (0xFF << 0)

/* For boolean data type, such as the valves */
#define AV_NET_CMD_ON  0x64  // Activate/Open
#define AV_NET_CMD_OFF 0x0D  // Deactivate/Close

/* Each I2C packet involves 4-bytes of data */
#define AV_NET_XFER_SIZE 4

/* Register addresses */ 
// Havoc DPR
enum AV_NET_REG_DPR {
    AV_NET_DPR_TIMESTAMP      = 0x00, // -W
    AV_NET_DPR_WAKE_UP        = 0x01, // -W
    AV_NET_DPR_IS_WOKEN_UP    = 0x02, // R-
    AV_NET_DPR_PRESSURIZE     = 0x03, // -W
    AV_NET_DPR_ABORT          = 0x04, // -W
    AV_NET_DPR_L_XTA          = 0x05, // R-  Tank level
    AV_NET_DPR_P_XTA          = 0x06, // R-  Tank pressure
    AV_NET_DPR_T_XTA          = 0x07, // R-  Tank temp
    AV_NET_DPR_P_NCO          = 0x08, // R-  COPV pressure
    AV_NET_DPR_T_NCO          = 0x09, // R-  COPV temp
    AV_NET_DPR_VALVES_STATE   = 0x0A, // RW
    AV_NET_DPR_NB_REG
};

// Prop Board
enum AV_NET_REG_PRB {
    AV_NET_PRB_TIMESTAMP       = 0x00, // -W
    AV_NET_PRB_WAKE_UP         = 0x01, // -W
    AV_NET_PRB_IS_WOKEN_UP     = 0x02, // R-
    AV_NET_PRB_CLEAR_TO_IGNITE = 0x03, // -W
    AV_NET_PRB_FSM_PRB         = 0x04, // R-
    AV_NET_PRB_ABORT           = 0x05, // -W
    AV_NET_PRB_P_OIN           = 0x06, // R-
    AV_NET_PRB_T_OIN           = 0x07, // R-
    AV_NET_PRB_P_EIN           = 0x08, // R-
    AV_NET_PRB_T_EIN           = 0x09, // R-
    AV_NET_PRB_P_CCC           = 0x0A, // R-
    AV_NET_PRB_T_CCC           = 0x0B, // R-
    AV_NET_PRB_VALVES_STATE    = 0x0C, // RW
    AV_NET_PRB_IGNITER         = 0x0D, // RW
    AV_NET_PRB_NB_REG
};

// Trigger Board
enum AV_NET_REG_TRB {
    AV_NET_TRB_TIMESTAMP        = 0x00, // -W
    AV_NET_TRB_WAKE_UP          = 0x01, // -W
    AV_NET_TRB_IS_WOKEN_UP      = 0x02, // R-
    AV_NET_TRB_CLEAR_TO_TRIGGER = 0x03, // -W
    AV_NET_TRB_PYROS            = 0x04, // RW
    AV_NET_TRB_HAS_TRIGGERED    = 0x05, // R-
    AV_NET_TRB_NB_REG
};

// Cameras
enum AV_NET_REG_CAM {
    AV_NET_CAM_TIMESTAMP      = 0x00, // -W
    AV_NET_CAM_WAKE_UP        = 0x01, // -W
    AV_NET_CAM_IS_WOKEN_UP    = 0x02, // R-
    AV_NET_CAM_RECORDING      = 0x03  // RW
};

#endif /* AV_INTRANET_COMMANDS_H */
