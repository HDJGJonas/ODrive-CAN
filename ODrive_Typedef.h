#ifndef __ODRIVE_TYPEDEF_H
#define __ODRIVE_TYPEDEF_H

#include <cstdint>

typedef enum {
    INVALID_STATE                   =0x1,
    MOTOR_FAILED                    =0x40,
    SENSORLESS_ESTIMATOR_FAILED     =0x80,
    ENCODER_FAILED                  =0x100,
    CONTROLLER_FAILED               =0x200,
    WATCHDOG_TIMER_EXPIRED          =0x800,
    MIN_ENDSTOP_PRESSED             =0x1000,
    MAX_ENDSTOP_PRESSED             =0x2000,
    ESTOP_REQUESTED                 =0x4000,
    HOMING_WITHOUT_ENDSTOP          =0x20000,
    OVER_TEMP                       =0x40000,
    UNKNOWN_POSITION                =0x80000,
    TIMEOUT_OCCURED                 =0x99999999,
} AxisError;

typedef enum {
    UNDEFINED,                        
    IDLE,                             
    STARTUP_SEQUENCE,                 
    FULL_CALIBRATION_SEQUENCE,        
    MOTOR_CALIBRATION,                
    ENCODER_INDEX_SEARCH = 0x6,            
    ENCODER_OFFSET_CALIBRATION,       
    CLOSED_LOOP_CONTROL,              
    LOCKIN_SPIN,                      
    ENCODER_DIR_FIND,                  
    HOMING,                            
    ENCODER_HALL_POLARITY_CALIBRATION, 
    ENCODER_HALL_PHASE_CALIBRATION,    
} AxisState;

typedef enum {
    PHASE_RESISTANCE_OUT_OF_RANG
    PHASE_INDUCTANCE_OUT_OF_RANGE
    DRV_FAULT
    CONTROL_DEADLINE_MISSED
    MODULATION_MAGNITUDE
    CURRENT_SENSE_SATURATION
    CURRENT_LIMIT_VIOLATION
    MODULATION_IS_NAN
    MOTOR_THERMISTOR_OVER_TEMP
    FET_THERMISTOR_OVER_TEMP
    TIMER_UPDATE_MISSED
    CURRENT_MEASUREMENT_UNAVAILABLE
    CONTROLLER_FAILED
    I_BUS_OUT_OF_RANGE
    BRAKE_RESISTOR_DISARMED
    SYSTEM_LEVEL
    BAD_TIMING
    UNKNOWN_PHASE_ESTIMATE
    UNKNOWN_PHASE_VEL
    UNKNOWN_TORQUE
    UNKNOWN_CURRENT_COMMAND
    UNKNOWN_CURRENT_MEASUREMENT
    UNKNOWN_VBUS_VOLTAGE
    UNKNOWN_VOLTAGE_COMMAND
    UNKNOWN_GAINS
    CONTROLLER_INITIALIZING
    UNBALANCED_PHASES
} MotorError;

typedef enum {
    VOLTAGE_CONTROL,
    TORQUE_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
} ControlMode;

typedef enum {
    INACTIVE,  
    PASSTHROUGH, 
    VEL_RAMP,    
    POS_FILTER,  
    MIX_CHANNELS,
    TRAP_TRAJ,   
    TORQUE_RAMP, 
    MIRROR,      
    TUNING,      
} InputMode;

typedef enum {
    GetVersion,
    Heartbeat,
    Endstop,
    GetError,
    SetAxisNodeID = 0x6,
    SetAxisState,
    GetEncoderEstimates = 0x9,
    SetControllerMode = 0xb,
    SetInputPosition,
    SetInputVelocity,
    SetInputTorque,
    SetLimits,
    StartAnticogging,
    SetTrajectoryVelocityLimit,
    SetTrajectoryAccelerationLimit,
    SetTrajectoryInertia,
    GetIQ,
    GetTemperature,
    Reboot,
    GetBusVoltageCurrent,
    ClearErrors,
    SetAbsolutePosition,
    SetPositionGain,
    SetVelocityGains,
    GetADCVoltage,
    GetControllerError,
    DFU,
} Commands;

typedef struct {
    uint8_t Protocol;
    uint8_t HW_Major;
    uint8_t HW_Minor;
    uint8_t HW_Variant;
    uint8_t FW_Major;
    uint8_t FW_Minor;
    uint8_t FW_Revision;
    uint8_t FW_Unreleased;
} VersionQuery;

typedef struct {
    uint32_t AxisError;
    uint8_t AxisState;
    uint8_t ProcedureResult;
    bool TrajectoryDone;
} HeartbeatRequest;

typedef struct {
    uint32_t Active;
    uint32_t Reason;
} ErrorStatus;

typedef struct {
    uint32_t Position;
    uint32_t Velocity;
} EncoderEstimates;

typedef struct {
    uint32_t SetPoint;
    uint32_t Measured;
} IQRequest;

typedef struct {
    uint32_t SetPoint;
    uint32_t Measured;
} TemperatureRequest;

typedef struct {
    uint32_t Voltage;
    uint32_t Current;
} PowerRequest;

#endif
