#include "IPC_RPC_Comm.h"
#include "foc.h"
#include "Encoder.h"

#define SERIALIZE_FIELD(bufferPtr, structPtr, field) \
    memcpy(bufferPtr, &(structPtr->field), sizeof(structPtr->field)); \
    buffer += sizeof(structPtr->field);

#define DESERIALIZE_FIELD(bufferPtr, structPtr, field) \
    memcpy(&(structPtr->field), bufferPtr, sizeof(structPtr->field)); \
    bufferPtr += sizeof(structPtr->field);


void PopulateInverterStreamPacket(Motor_t* motorStructHandle, PosSpeed_Object* posSpeedStructHandle)
{

}

void SerialiseInverterStreamPacket(const InverterStreamPacket_t* packet, char* buffer)
{
    SERIALIZE_FIELD(buffer, packet, IsrTick);
    SERIALIZE_FIELD(buffer, packet, Ia);
    SERIALIZE_FIELD(buffer, packet, Ib);
    SERIALIZE_FIELD(buffer, packet, Ic);
    SERIALIZE_FIELD(buffer, packet, DcBus);
    SERIALIZE_FIELD(buffer, packet, Id);
    SERIALIZE_FIELD(buffer, packet, Iq);
    SERIALIZE_FIELD(buffer, packet, theta_e);
    SERIALIZE_FIELD(buffer, packet, omega_e);
    SERIALIZE_FIELD(buffer, packet, Out_Vd);
    SERIALIZE_FIELD(buffer, packet, Out_Vq);
    SERIALIZE_FIELD(buffer, packet, RegSpeed_Fback);
    SERIALIZE_FIELD(buffer, packet, RegSpeed_Output);
    SERIALIZE_FIELD(buffer, packet, RegId_Fback);
    SERIALIZE_FIELD(buffer, packet, RegId_Output);
    SERIALIZE_FIELD(buffer, packet, RegIq_Fback);
    SERIALIZE_FIELD(buffer, packet, RegIq_Output);
    SERIALIZE_FIELD(buffer, packet, EncoderTheta);
    SERIALIZE_FIELD(buffer, packet, EncoderOmega);
    SERIALIZE_FIELD(buffer, packet, SpeedRef);
    SERIALIZE_FIELD(buffer, packet, MotorRunStop);
    SERIALIZE_FIELD(buffer, packet, RegSpeed_RefVal);
    SERIALIZE_FIELD(buffer, packet, RegSpeed_Kp);
    SERIALIZE_FIELD(buffer, packet, RegSpeed_Ki);
    SERIALIZE_FIELD(buffer, packet, RegId_RefVal);
    SERIALIZE_FIELD(buffer, packet, RegId_Kp);
    SERIALIZE_FIELD(buffer, packet, RegId_Ki);
    SERIALIZE_FIELD(buffer, packet, RegIq_RefVal);
    SERIALIZE_FIELD(buffer, packet, RegIq_Kp);
    SERIALIZE_FIELD(buffer, packet, RegIq_Ki);
}
