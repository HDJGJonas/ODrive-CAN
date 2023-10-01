#ifndef __ODRIVE_H
#define __ODRIVE_H

#include "CAN.h"
#include "ODrive_Typedef.h"
#include <vector>

namespace ODrive {
    class ODrive {
        private:
            CAN *CommChannel;
            int Write(can_frame *Data);
            int Read(can_frame *Data);
            can_frame ConstructCANMessage(uint16_t NodeID, Commands Cmd, uint8_t *Data, uint32_t DataSize);
        public:
            // [De]constructor
            ODrive();
            ~ODrive();

            // Function declarations
            HeartbeatRequest GetHeartbeat(uint16_t Node);
            void Endstop(uint16_t Node);
            uint64_t GetMotorError(uint16_t Node);
            uint32_t GetEncoderError(uint16_t Node);
            uint32_t GetSensorlessError(uint16_t Node);
            void SetAxisNodeID(uint16_t Node, uint32_t ID);
            void SetAxisRequestedState(uint16_t Node, AxisState State);
            EncoderEstimates GetEncoderEstimate(uint16_t Node);
            EncoderCount GetEncoderCount(uint16_t Node);
            void SetControllerMode(uint16_t Node, ControlMode Ctrl, InputMode Input);
            void SetInputPos(uint16_t Node, uint32_t Position, uint16_t VelocityFF, uint16_t TorqueFF);
            void SetInputVel(uint16_t Node, uint32_t Velocity, uint16_t TorqueFF);
            void SetInputTorque(uint16_t Node, uint32_t Torque);
            void SetLimits(uint16_t Node, uint16_t VelocityLimit, uint16_t CurrentLimit);
            void StartAnticogging(uint16_t Node);
            void SetTrajVelLimit(uint16_t Node, uint32_t Limit);
            void SetTrajAccelLimits(uint16_t Node, uint32_t AccelLimit, uint32_t DeaccelLimit);
            void SetTrajInertia(uint16_t Node, uint32_t Inertia);
            IQRequest GetIQ(uint16_t Node);
            SensorlessEstimates GetSensorlessEstimates(uint16_t Node);
            void Reboot(uint16_t Node);
            PowerRequest GetBusVoltageAndCurrent(uint16_t Node);
            void ClearErrors(uint16_t Node);
            void SetLinearCount(uint16_t Node, uint32_t Position);
            void SetPositionGain(uint16_t Node, uint32_t Gain);
            void SetVelGains(uint16_t Node, uint32_t VelGain, uint32_t VelIntegratorGain);
            uint32_t GetADCVoltage(uint16_t Node);
            uint32_t GetControllerError(uint16_t Node);
    };
};

#endif