
#include "IPC_RPC_Comm.h"
#include "FOC_loop.h"
// #include "foc.h"
#include <drivers/ipc_rpmsg.h>

#define RECEVE_ENDPOINT 10
#define TCP_STREAM_ENDPOINT 12
#define TCP_SERVER_CORE_ID CSL_CORE_ID_R5FSS0_1

RPMessage_Object gRecvMsgObject;
void IPC_SendMessage(void *data, uint16_t dataLen);
void IPC_SendInverterDataPacket();

// Konfiguracija frekvencije slanja mjerenja jezgri za tcp komunikaciju
// Counter broji brzinom uzorkovanja, dakle 10kHz
#define PACKETS_TO_SKIP 10
uint16_t skipPacketsCounter = 0;


void setup_IPC()
{
    RPMessage_CreateParams createParams;
    RPMessage_CreateParams_init(&createParams);
    createParams.localEndPt = RECEVE_ENDPOINT;
    RPMessage_construct(&gRecvMsgObject, &createParams);

    SendNewIPCRPPacket = 0;
}

void processIPC()
{
    if (SendNewIPCRPPacket == 1) {
        // Reset the flag
        SendNewIPCRPPacket = 0;
        IPC_SendInverterDataPacket();
    }
}

void raiseIPCTransmissionFlag()
{
    skipPacketsCounter++;
    if (skipPacketsCounter >= PACKETS_TO_SKIP) {
        skipPacketsCounter = 0;
        SendNewIPCRPPacket = 1;
    }
}


void IPC_SendInverterDataPacket()
{
    Motor_t* motorHandle = FOC_DANGER_getMotorStructPointer();

    IPC_SendMessage(motorHandle, sizeof(motorHandle));
}


void IPC_SendMessage(void *data, uint16_t dataLen)
{

    RPMessage_send(
        data,
        dataLen,
        TCP_SERVER_CORE_ID,
        TCP_STREAM_ENDPOINT,
        RPMessage_getLocalEndPt(&gRecvMsgObject),
        100);
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
