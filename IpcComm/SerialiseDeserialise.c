#include "IPC_RPC_Comm.h"
#include "foc.h"
#include "Encoder.h"
#include "FOC_loop.h"

#define SERIALIZE_FIELD(bufferPtr, structPtr, field) \
    memcpy(bufferPtr, &(structPtr->field), sizeof(structPtr->field)); \
    buffer += sizeof(structPtr->field);


void PopulateInverterStreamPacket(InverterStreamPacket_t* packetHandle, Motor_t* motorStructHandle, PosSpeed_Object* posSpeedStructHandle, uint32_t isrTick)
{
    packetHandle->IsrTick = isrTick;
    packetHandle->Ia = motorStructHandle->I_abc_A[0];
    packetHandle->Ib = motorStructHandle->I_abc_A[1];
    packetHandle->Ic = motorStructHandle->I_abc_A[2];
    packetHandle->DcBus = motorStructHandle->dcBus_V;
    packetHandle->Id = motorStructHandle->I_dq_A[0];
    packetHandle->Iq = motorStructHandle->I_dq_A[1];
    packetHandle->theta_e = motorStructHandle->theta_e;
    packetHandle->omega_e = motorStructHandle->omega_e;
    packetHandle->Out_Vd = motorStructHandle->Vout_dq_V[0];
    packetHandle->Out_Vq = motorStructHandle->Vout_dq_V[2];
    packetHandle->RegSpeed_Fback = motorStructHandle->pi_spd.fbackValue;
    packetHandle->RegSpeed_Output = motorStructHandle->pi_spd.outValue;
    packetHandle->RegId_Fback = motorStructHandle->pi_id.fbackValue;
    packetHandle->RegId_Output = motorStructHandle->pi_id.outValue;
    packetHandle->RegIq_Fback = motorStructHandle->pi_iq.fbackValue;
    packetHandle->RegIq_Output = motorStructHandle->pi_id.outValue;
    packetHandle->EncoderTheta = posSpeedStructHandle->thetaElec;
    packetHandle->EncoderOmega = posSpeedStructHandle->speedPR;
    packetHandle->SpeedRef = FOC_getSpeedRef();
    packetHandle->MotorRunStop = (float32_t) FOC_getMotorRunState();
    packetHandle->RegSpeed_RefVal = motorStructHandle->pi_spd.refValue;
    packetHandle->RegSpeed_Kp = motorStructHandle->pi_spd.Kp;
    packetHandle->RegSpeed_Ki = motorStructHandle->pi_spd.Ki;
    packetHandle->RegId_RefVal = motorStructHandle->pi_id.refValue;
    packetHandle->RegId_Kp = motorStructHandle->pi_id.Kp;
    packetHandle->RegId_Ki = motorStructHandle->pi_id.Ki;
    packetHandle->RegIq_RefVal = motorStructHandle->pi_iq.refValue;
    packetHandle->RegIq_Kp = motorStructHandle->pi_iq.Kp;
    packetHandle->RegIq_Ki = motorStructHandle->pi_iq.Ki;
}

void SerialiseInverterStreamPacket(const InverterStreamPacket_t* packet, char buffer[])
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
