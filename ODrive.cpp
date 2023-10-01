#include "ODrive.h"
#include <iostream>

ODrive::ODrive::ODrive() {
    this->CommChannel = new CAN();
}

ODrive::ODrive::~ODrive() {
    delete this->CommChannel;
}

int ODrive::ODrive::Write(can_frame *Data) {
    return this->CommChannel->Write(Data);
}

int ODrive::ODrive::Read(can_frame *Data) {
    return this->CommChannel->Read(Data);
}

uint16_t ReadU16(uint8_t *Buffer, uint32_t Offset) {
    return Buffer[Offset] | (Buffer[Offset + 1] << 0x8);
}

uint32_t ReadU32(uint8_t *Buffer, uint32_t Offset) {
    return Buffer[Offset] | (Buffer[Offset + 1] << 0x8) | (Buffer[Offset + 2] << 0x10) | (Buffer[Offset + 3] << 0x18);
}

uint32_t ReadU64(uint8_t *Buffer, uint32_t Offset) {
    return Buffer[Offset] | (Buffer[Offset + 1] << 0x8) | (Buffer[Offset + 2] << 0x10) | (Buffer[Offset + 3] << 0x18) | (Buffer[Offset + 4] << 0x20) | (Buffer[Offset + 5] << 0x28) | (Buffer[Offset + 6] << 0x30) | (Buffer[Offset + 7] << 0x38) | (Buffer[Offset + 8] << 0x40);
}

can_frame ODrive::ODrive::ConstructCANMessage(uint16_t NodeID, Commands Cmd, uint8_t *Data = NULL, uint32_t DataSize = 0) {
    can_frame Frame;
    memset(&Frame, 0, sizeof(can_frame));
    // Set arbitration ID.
    Frame.can_id = ((NodeID & 0x3F) << 0x5) | ((uint16_t)Cmd & 0x1F);
    Frame.can_dlc = DataSize;
    // Copy parameter data if exists.
    if (Data && DataSize > 0) {
        memcpy(Frame.data, Data, DataSize);
    }

    return Frame;
}

HeartbeatRequest ODrive::ODrive::GetHeartbeat(uint16_t Node)   {
    can_frame Request = ConstructCANMessage(Node, Commands::GetHearbeat);
    Write(&Request);
    
    Read(&Request);
    HeartbeatRequest Response = {
        .AxisError = ReadU32(Request.data, 0),
        .AxisCurrentState = Request.data[4],
        .MotorErrorFlag = Request.data[5],
        .EncoderErrorFlag = Request.data[6],
        .ControllerErrorFlag = Request.data[7] & 0xFE,
        .TrajectoryDoneFlag = Request.data[7] & 0x1,
    };
    return Response;
}

void ODrive::ODrive::Endstop(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::Endstop);
    Write(&Request);
}

uint64_t ODrive::ODrive::GetMotorError(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetMotorError);
    Write(&Request);
    
    Read(&Request);
    return ReadU64(Request.data, 0);
}

uint32_t ODrive::ODrive::GetEncoderError(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetEncoderError);
    Write(&Request);
    
    Read(&Request);
    return ReadU32(Request.data, 0);
}

uint32_t ODrive::ODrive::GetSensorlessError(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetSensorlessError);
    Write(&Request);
    
    Read(&Request);
    return ReadU32(Request.data, 0);
}

void ODrive::ODrive::SetAxisNodeID(uint16_t Node, uint32_t ID) {
    std::vector<uint32_t> Params;
    Params.push_back(ID);
    can_frame Request = ConstructCANMessage(Node, Commands::SetAxisNodeId, (uint8_t *)&Params[0]);
    Write(&Request);
}

void ODrive::ODrive::SetAxisRequestedState(uint16_t Node, AxisState State) {
    std::vector<uint32_t> Params;
    Params.push_back(State);
    can_frame Request = ConstructCANMessage(Node, Commands::SetAxisRequestedState, (uint8_t *)&Params[0], sizeof(State));
    Write(&Request);
}

EncoderEstimates ODrive::ODrive::GetEncoderEstimate(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetEncoderEstimate);
    Write(&Request);
    
    Read(&Request);
    EncoderEstimates Response = {
        .Position = ReadU32(Request.data, 0),
        .Velocity = ReadU32(Request.data, 4),
    };
    return Response;
}

void ODrive::ODrive::SetControllerMode(uint16_t Node, ControlMode Ctrl, InputMode Input) {
    std::vector<uint32_t> Params;
    Params.push_back(Ctrl);
    Params.push_back(Input);
    can_frame Request = ConstructCANMessage(Node, Commands::SetControllerMode, (uint8_t *)&Params[0], sizeof(Ctrl) + sizeof(Input));
    Write(&Request);
}

void ODrive::ODrive::SetInputPos(uint16_t Node, uint32_t Position, uint16_t VelocityFF, uint16_t TorqueFF) {
    std::vector<uint32_t> Params;
    Params.push_back(Position);
    Params.push_back(VelocityFF | (TorqueFF << 0x10));
    can_frame Request = ConstructCANMessage(Node, Commands::SetInputPos, (uint8_t *)&Params[0], sizeof(Position) + sizeof(VelocityFF) + sizeof(TorqueFF));
    Write(&Request);
}

void ODrive::ODrive::SetInputVel(uint16_t Node, uint32_t Velocity, uint16_t TorqueFF) {
    std::vector<uint32_t> Params;
    Params.push_back(Velocity);
    Params.push_back(TorqueFF);
    can_frame Request = ConstructCANMessage(Node, Commands::SetInputVel, (uint8_t *)&Params[0], sizeof(Velocity) + sizeof(TorqueFF));
    Write(&Request);
}

void ODrive::ODrive::SetInputTorque(uint16_t Node, uint32_t Torque) {
    std::vector<uint32_t> Params;
    Params.push_back(Torque);
    can_frame Request = ConstructCANMessage(Node, Commands::SetInputTorque, (uint8_t *)&Params[0], sizeof(Torque));
    Write(&Request);
}

void ODrive::ODrive::SetLimits(uint16_t Node, uint16_t VelocityLimit, uint16_t CurrentLimit) {
    std::vector<uint32_t> Params;
    Params.push_back(VelocityLimit | (CurrentLimit << 0x10));
    can_frame Request = ConstructCANMessage(Node, Commands::SetInputTorque, (uint8_t *)&Params[0], sizeof(VelocityLimit) + sizeof(CurrentLimit));
    Write(&Request);
}

void ODrive::ODrive::StartAnticogging(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::StartAnticogging);
    Write(&Request);
}

void ODrive::ODrive::SetTrajVelLimit(uint16_t Node, uint32_t Limit) {
    std::vector<uint32_t> Params;
    Params.push_back(Limit);
    can_frame Request = ConstructCANMessage(Node, Commands::SetTrajVelLimit, (uint8_t *)&Params[0], sizeof(Limit));
    Write(&Request);
}

void ODrive::ODrive::SetTrajAccelLimits(uint16_t Node, uint32_t AccelLimit, uint32_t DeaccelLimit) {
    std::vector<uint32_t> Params;
    Params.push_back(AccelLimit);
    Params.push_back(DeaccelLimit);
    can_frame Request = ConstructCANMessage(Node, Commands::SetTrajectoryAccelerationLimit, (uint8_t *)&Params[0], sizeof(AccelLimit) + sizeof(DeaccelLimit));
    Write(&Request);
}

void ODrive::ODrive::SetTrajInertia(uint16_t Node, uint32_t Inertia) {
    std::vector<uint32_t> Params;
    Params.push_back(Inertia);
    can_frame Request = ConstructCANMessage(Node, Commands::SetTrajInertia, (uint8_t *)&Params[0], sizeof(Inertia));
    Write(&Request);
}

IQRequest ODrive::ODrive::GetIQ(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetIQ);
    Write(&Request);
    
    Read(&Request);
    IQRequest Response = {
        .Setpoint = ReadU32(Request.data, 0),
        .Measured = ReadU32(Request.data, 4),
    };
    return Response;
}

SensorlessEstimates ODrive::ODrive::GetSensorlessEstimates(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetSensorlessError);
    Write(&Request);
    
    Read(&Request);
    SensorlessEstimates Response = {
        .PosEstimate = ReadU32(Request.data, 0),
        .VelEstimate = ReadU32(Request.data, 4),
    };
    return Response;
}

void ODrive::ODrive::Reboot(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::Reboot);
    Write(&Request);
}

PowerRequest ODrive::ODrive::GetBusVoltageAndCurrent(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetBusVoltageAndCurrent);
    Write(&Request);
    
    Read(&Request);
    PowerRequest Response = {
        .Voltage = ReadU32(Request.data, 0),
        .Current = ReadU32(Request.data, 4),
    };
    return Response;
}

void ODrive::ODrive::ClearErrors(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::ClearErrors);
    Write(&Request);
}

void ODrive::ODrive::SetLinearCount(uint16_t Node, uint32_t Position) {
    std::vector<uint32_t> Params;
    Params.push_back(Position);
    can_frame Request = ConstructCANMessage(Node, Commands::SetLinearCount, (uint8_t *)&Params[0], sizeof(Position));
    Write(&Request);
}

void ODrive::ODrive::SetPositionGain(uint16_t Node, uint32_t Gain) {
    std::vector<uint32_t> Params;
    Params.push_back(Gain);
    can_frame Request = ConstructCANMessage(Node, Commands::SetPositionGain, (uint8_t *)&Params[0], sizeof(Gain));
    Write(&Request);
}

void ODrive::ODrive::SetVelGains(uint16_t Node, uint32_t VelGain, uint32_t VelIntegratorGain) {
    std::vector<uint32_t> Params;
    Params.push_back(VelGain);
    Params.push_back(VelIntegratorGain);
    can_frame Request = ConstructCANMessage(Node, Commands::SetVelGains, (uint8_t *)&Params[0], sizeof(VelGain) + sizeof(VelIntegratorGain));
    Write(&Request);
}

uint32_t ODrive::ODrive::GetADCVoltage(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetADCVoltage);
    Write(&Request);
    Read(&Request);
    return ReadU32(Request.data, 0);
}

uint32_t ODrive::ODrive::GetControllerError(uint16_t Node) {
    can_frame Request = ConstructCANMessage(Node, Commands::GetControllerError);
    Write(&Request);
    Read(&Request);
    return ReadU32(Request.data, 0);
}