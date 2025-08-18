/*
 * CANMessages.h
 *
 *  Created on: Aug 18, 2025
 *  Author: Max Lan
 *  Description: CANSimple protocol message IDs for ODrive.
 *  Docs: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-msg-get-version
 */

#ifndef INC_CANMESSAGES_H_
#define INC_CANMESSAGES_H_

/* -----------------------------
 * ODrive CANSimple ID helpers
 * 11-bit CAN ID = (node_id << 5) | cmd_id
 * node_id : 6 bits [10:5]
 * cmd_id  : 5 bits [4:0]
 * ----------------------------- */
#define ODRIVE_CMD_ID_MASK      0x1F
#define ODRIVE_NODE_ID_MASK     0x3F

#define ODRIVE_MAKE_CAN_ID(node_id, cmd_id) \
    ((((uint16_t)(node_id)) & ODRIVE_NODE_ID_MASK) << 5 | ((uint16_t)(cmd_id) & ODRIVE_CMD_ID_MASK))

#define ODRIVE_GET_NODE_ID(can_id)  ( (uint16_t)(((can_id) >> 5) & ODRIVE_NODE_ID_MASK) )
#define ODRIVE_GET_CMD_ID(can_id)   ( (uint16_t)((can_id) & ODRIVE_CMD_ID_MASK) )

// CANSimple command IDs
#define GET_VERSION                 0x000  // ODrive -> Host | Protocol/Hw/Fw version fields
#define HEARTBEAT                   0x001  // ODrive -> Host | Axis_Error, Axis_State, Procedure_Result, Trajectory_Done_Flag
#define ESTOP                       0x002  // Host  -> ODrive
#define GET_ERROR                   0x003  // ODrive -> Host | Active_Errors, Disarm_Reason
#define RX_SDO                      0x004  // Host  -> ODrive | Opcode, Endpoint_ID, Value
#define TX_SDO                      0x005  // ODrive -> Host | Endpoint_ID, Value
#define ADDRESS                     0x006  // Host<->ODrive | Node_ID, Serial_Number, Connection_ID
#define SET_AXIS_STATE              0x007  // Host  -> ODrive | Axis_Requested_State
#define GET_ENCODER_ESTIMATES       0x009  // ODrive -> Host | Pos_Estimate, Vel_Estimate
#define SET_CONTROLLER_MODE         0x00B  // Host  -> ODrive | Control_Mode, Input_Mode
#define SET_INPUT_POS               0x00C  // Host  -> ODrive | Input_Pos, Vel_FF, Torque_FF
#define SET_INPUT_VEL               0x00D  // Host  -> ODrive | Input_Vel, Input_Torque_FF
#define SET_INPUT_TORQUE            0x00E  // Host  -> ODrive | Input_Torque
#define SET_LIMITS                  0x00F  // Host  -> ODrive | Velocity_Limit, Current_Limit
#define SET_TRAJ_VEL_LIMIT          0x011  // Host  -> ODrive | Traj_Vel_Limit
#define SET_TRAJ_ACCEL_LIMITS       0x012  // Host  -> ODrive | Traj_Accel_Limit, Traj_Decel_Limit
#define SET_TRAJ_INERTIA            0x013  // Host  -> ODrive | Traj_Inertia
#define GET_IQ                      0x014  // ODrive -> Host | Iq_Setpoint, Iq_Measured
#define GET_TEMPERATURE             0x015  // ODrive -> Host | FET_Temperature, Motor_Temperature
#define REBOOT                      0x016  // Host  -> ODrive | Action
#define GET_BUS_VOLTAGE_CURRENT     0x017  // ODrive -> Host | Bus_Voltage, Bus_Current
#define CLEAR_ERRORS                0x018  // Host  -> ODrive | Identify
#define SET_ABSOLUTE_POSITION       0x019  // Host  -> ODrive | Position
#define SET_POS_GAIN                0x01A  // Host  -> ODrive | Pos_Gain
#define SET_VEL_GAINS               0x01B  // Host  -> ODrive | Vel_Gain, Vel_Integrator_Gain
#define GET_TORQUES                 0x01C  // ODrive -> Host | Torque_Target, Torque_Estimate
#define GET_POWERS                  0x01D  // ODrive -> Host | Electrical_Power, Mechanical_Power
#define ENTER_DFU_MODE              0x01F  // Host  -> ODrive

#endif /* INC_CANMESSAGES_H_ */
