#ifndef __ODRIVE_TYPEDEF_H
#define __ODRIVE_TYPEDEF_H

#include <cstdint>

typedef enum {
    INVALID_STATE                   = 0x1,
    MOTOR_FAILED                    = 0x40,
    SENSORLESS_ESTIMATOR_FAILED     = 0x80,
    ENCODER_FAILED                  = 0x100,
    CONTROLLER_FAILED_AXIS          = 0x200,
    WATCHDOG_TIMER_EXPIRED          = 0x800,
    MIN_ENDSTOP_PRESSED             = 0x1000,
    MAX_ENDSTOP_PRESSED             = 0x2000,
    ESTOP_REQUESTED                 = 0x4000,
    HOMING_WITHOUT_ENDSTOP          = 0x20000,
    OVER_TEMP                       = 0x40000,
    UNKNOWN_POSITION                = 0x80000,
    TIMEOUT_OCCURED                 = 0x99999999,
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
    PHASE_RESISTANCE_OUT_OF_RANG        = 0x1,
    PHASE_INDUCTANCE_OUT_OF_RANGE       = 0x2,
    DRV_FAULT                           = 0x8,
    CONTROL_DEADLINE_MISSED             = 0x10,
    MODULATION_MAGNITUDE                = 0x80,
    CURRENT_SENSE_SATURATION            = 0x400,
    CURRENT_LIMIT_VIOLATION             = 0x1000,
    MODULATION_IS_NAN                   = 0x10000,
    MOTOR_THERMISTOR_OVER_TEMP          = 0x20000,
    FET_THERMISTOR_OVER_TEMP            = 0x40000,
    TIMER_UPDATE_MISSED                 = 0x80000,
    CURRENT_MEASUREMENT_UNAVAILABLE     = 0x100000,
    CONTROLLER_FAILED_MOTOR             = 0x200000,
    I_BUS_OUT_OF_RANGE                  = 0x400000,
    BRAKE_RESISTOR_DISARMED             = 0x800000,
    SYSTEM_LEVEL                        = 0x1000000,
    BAD_TIMING                          = 0x2000000,
    UNKNOWN_PHASE_ESTIMATE              = 0x4000000,
    UNKNOWN_PHASE_VEL                   = 0x8000000,
    UNKNOWN_TORQUE                      = 0x10000000,
    UNKNOWN_CURRENT_COMMAND             = 0x20000000,
    UNKNOWN_CURRENT_MEASUREMENT         = 0x40000000,
    UNKNOWN_VBUS_VOLTAGE                = 0x80000000,
    UNKNOWN_VOLTAGE_COMMAND             = 0x100000000,
    UNKNOWN_GAINS                       = 0x200000000,
    CONTROLLER_INITIALIZING             = 0x400000000,
    UNBALANCED_PHASES                   = 0x800000000,
} MotorError;

typedef enum {
    UNSTABLE_GAIN_ENCODER       = 0x1,
    CPR_POLEPAIRS_MISMATCH      = 0x2,
    NO_RESPONSE                 = 0x4,
    UNSUPPORTED_ENCODER_MODE    = 0x8,
    ILLEGAL_HALL_STATE          = 0x10,
    INDEX_NOT_FOUND_YET         = 0x20,
    ABS_SPI_TIMEOUT             = 0x40,
    ABS_SPI_COM_FAIL            = 0x80,
    ABS_SPI_NOT_READY           = 0x100,
    HALL_NOT_CALIBRATED_YET     = 0x200,
} EncoderError;

typedef enum {
    UNSTABLE_GAIN_SENSORLESS                = 0x1,
    UNKNOWN_CURRENT_MEASUREMENT_SENSORLESS  = 0x2,
} SensorlessError;

typedef enum {
    OVERSPEED               = 0x1,
    INVALID_INPUT_MODE      = 0x2,
    UNSTABLE_GAIN_CONTROLLER= 0x4,
    INVALID_MIRROR_AXIS     = 0x8,
    INVALID_LOAD_ENCODER    = 0x10,
    INVALID_ESTIMATE        = 0x20,
    INVALID_CIRCULAR_RANGE  = 0x40,
    SPINOUT_DETECTED        = 0x80,
} ControllerError;

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
    GetHearbeat            = 0x001,
    Endstop                = 0x002,
    GetMotorError          = 0x003,
    GetEncoderError        = 0x004,
    GetSensorlessError     = 0x005,
    SetAxisNodeId          = 0x006,
    SetAxisRequestedState  = 0x007,
    GetEncoderEstimate     = 0x009,
    GetEncoderCount        = 0x00A,
    SetControllerMode      = 0x00B,
    SetInputPos            = 0x00C,
    SetInputVel            = 0x00D,
    SetInputTorque         = 0x00E,
    SetLimits              = 0x00F,
    StartAnticogging       = 0x010,
    SetTrajVelLimit        = 0x011,
    SetTrajAccelLimits     = 0x012,
    SetTrajInertia         = 0x013,
    GetIQ                  = 0x014,
    GetSensorlessEstimates = 0x015,
    Reboot                 = 0x016,
    GetBusVoltageAndCurrent= 0x017,
    ClearErrors            = 0x018,
    SetLinearCount         = 0x019,
    SetPositionGain        = 0x01A,
    SetVelGains            = 0x01B,
    GetADCVoltage          = 0x01C,
    GetControllerError     = 0x01D,
} Commands;

typedef struct {
    uint32_t AxisError;
    uint8_t AxisCurrentState;
    bool MotorErrorFlag;
    bool EncoderErrorFlag;
    bool ControllerErrorFlag;
    bool TrajectoryDoneFlag;
} HeartbeatRequest;

typedef struct {
    uint32_t Position;
    uint32_t Velocity;
} EncoderEstimates;

typedef struct {
    uint32_t ShadowCount;
    uint32_t CountInCPR;
} EncoderCount;

typedef struct {
    uint32_t Setpoint;
    uint32_t Measured;
} IQRequest;

typedef struct {
    uint32_t PosEstimate;
    uint32_t VelEstimate;
} SensorlessEstimates;

typedef struct {
    uint32_t Voltage;
    uint32_t Current;
} PowerRequest;

#endif
