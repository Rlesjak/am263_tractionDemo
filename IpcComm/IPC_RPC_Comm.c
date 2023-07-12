
#include "IPC_RPC_Comm.h"
#include "FOC_loop.h"
#include "Encoder.h"
// #include "foc.h"
#include <drivers/ipc_rpmsg.h>

#define RECEVE_ENDPOINT 10
#define TCP_STREAM_ENDPOINT 12
#define TCP_SERVER_CORE_ID CSL_CORE_ID_R5FSS0_1


typedef struct __CommandPacket {
   int endpoint;
   float32_t value;
} CommandPacket;


RPMessage_Object gRecvMsgObject;
void IPC_SendMessage(void *data, uint16_t dataLen);
void IPC_SendInverterDataPacket();

// Konfiguracija frekvencije slanja mjerenja jezgri za tcp komunikaciju
// Counter broji brzinom uzorkovanja, dakle 10kHz
#define PACKETS_TO_SKIP 20
uint16_t skipPacketsCounter = 0;

unsigned short SendNewIPCRPPacket;
uint32_t packetTime;


void setup_IPC()
{
    RPMessage_CreateParams createParams;
    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = RECEVE_ENDPOINT;
    RPMessage_construct(&gRecvMsgObject, &createParams);

    SendNewIPCRPPacket = 0;
    packetTime = 0;
}

void processIPC()
{
    if (SendNewIPCRPPacket == 1) {
        // Reset the flag
        SendNewIPCRPPacket = 0;
        IPC_SendInverterDataPacket();
    }

    CommandPacket commandPacket = { -1, 0 };

    char receveBuffer[sizeof(commandPacket)] = {0};
    uint16_t replyMsgSize = sizeof(receveBuffer);
    uint16_t remoteCoreId, remoteCoreEndPt;

    int32_t status = RPMessage_recv(
        &gRecvMsgObject,
        receveBuffer,
        &replyMsgSize,
        &remoteCoreId,
        &remoteCoreEndPt,
        1);

    memcpy(&(commandPacket.endpoint), receveBuffer, sizeof(commandPacket.endpoint));
    memcpy(&(commandPacket.value), receveBuffer + sizeof(commandPacket.endpoint), sizeof(commandPacket.value));

    if ( commandPacket.endpoint == -1) {
        return;
    }

    // MotorEnable
    if ( commandPacket.endpoint == 1) {
        if (commandPacket.value > 0.98 && commandPacket.value < 1.02) {
            FOC_setMotorRunState(1);
        }
        else {
            FOC_setMotorRunState(0);
        }
    }
    // Id_ref
    else if ( commandPacket.endpoint == 2) {
        FOC_setIdref(commandPacket.value);
    }
    // N_ref
    else if ( commandPacket.endpoint == 3) {
        FOC_setSpeedRef(commandPacket.value);
    }
    // SpdKp
    else if ( commandPacket.endpoint == 4) {
        Motor_t* motorhandle = FOC_DANGER_getMotorStructPointer();
        motorhandle->pi_spd.Kp = commandPacket.value;
    }
    // SpdKi
    else if ( commandPacket.endpoint == 5) {
        Motor_t* motorhandle = FOC_DANGER_getMotorStructPointer();
        motorhandle->pi_spd.Ki = commandPacket.value;
    }
    // IdKp
    else if ( commandPacket.endpoint == 6) {
        Motor_t* motorhandle = FOC_DANGER_getMotorStructPointer();
        motorhandle->pi_id.Kp = commandPacket.value;
    }
    // IdKi
    else if ( commandPacket.endpoint == 7) {
        Motor_t* motorhandle = FOC_DANGER_getMotorStructPointer();
        motorhandle->pi_id.Ki = commandPacket.value;
    }
    // IqKp
    else if ( commandPacket.endpoint == 8) {
        Motor_t* motorhandle = FOC_DANGER_getMotorStructPointer();
        motorhandle->pi_iq.Kp = commandPacket.value;
    }
    // IqKi
    else if ( commandPacket.endpoint == 9) {
        Motor_t* motorhandle = FOC_DANGER_getMotorStructPointer();
        motorhandle->pi_iq.Ki = commandPacket.value;
    }
//    // ackPer
//    else if ( commandPacket.endpoint == 10) {
//        // TODO requires refactor
//    }
}

void raiseIPCTransmissionFlag(uint32_t isrTick)
{
    skipPacketsCounter++;
    if (skipPacketsCounter >= PACKETS_TO_SKIP) {
        skipPacketsCounter = 0;
        SendNewIPCRPPacket = 1;
        packetTime = isrTick;
    }
}


void IPC_SendInverterDataPacket()
{
    Motor_t* motorHandle = FOC_DANGER_getMotorStructPointer();
    PosSpeed_Object* posSpeedHandle = FOC_DANGER_getPosSpeedStructPointer();

    InverterStreamPacket_t packet;
    PopulateInverterStreamPacket(&packet, motorHandle, posSpeedHandle, packetTime);

    char serialisedPacket[sizeof(packet)] = {0};
    SerialiseInverterStreamPacket(&packet, serialisedPacket);

    IPC_SendMessage(serialisedPacket, sizeof(serialisedPacket));
}


void IPC_SendMessage(void *data, uint16_t dataLen)
{
    RPMessage_send(
        data,
        dataLen,
        TCP_SERVER_CORE_ID,
        TCP_STREAM_ENDPOINT,
        RPMessage_getLocalEndPt(&gRecvMsgObject),
        0);
}



void IPC_ReceveMessage()
{
    char sendMsg[64] = "hello, tcp!!!";
    char replyMsg[64];
    uint16_t replyMsgSize, remoteCoreId, remoteCoreEndPt;
    /* set 'replyMsgSize' to size of recv buffer,
     * after return `replyMsgSize` contains actual size of valid data in recv buffer
     */
    replyMsgSize = sizeof(replyMsg); /*  */
    RPMessage_recv(
        &gRecvMsgObject,
        replyMsg,
        &replyMsgSize,
        &remoteCoreId,
        &remoteCoreEndPt,
        SystemP_WAIT_FOREVER);
}
