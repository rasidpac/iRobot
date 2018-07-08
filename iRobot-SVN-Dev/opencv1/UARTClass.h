#include <stdio.h>
#include <windows.h>

#define MAX_TX_MESSAGE_SIZE 4096
#define MAX_RX_MESSAGE_SIZE 4096
#define SLEEP_TIME 20

class UARTCHANNEL
{
public:
   UARTCHANNEL();
   ~UARTCHANNEL();
   void setData(UINT32 uBR, UINT8 uBS, UINT8 uP, UINT8 uSB, UINT8 uCP);
   void initialize();
   bool transmitData(UINT32 uSize);
   bool receiveData(UINT32 uSize);
   void clearRxBuffer();
   void close();
   void copyReceiveBuffer(void *pBuffer);
   UINT32 uTxMessageSize;
   UINT32 uRxMessageSize;
   UINT8 auTxBuffer[MAX_TX_MESSAGE_SIZE];
   UINT8 auRxBuffer[MAX_RX_MESSAGE_SIZE];

private:
   UINT32 uBaudRate;
   UINT8 uByteSize;
   UINT8 uParity;
   UINT8 uStopBits;
   UINT32 uComPort;
   //COM_PORT_ENHANCED Settings;
   COMMTIMEOUTS CommunicationTimeOuts;
   OVERLAPPED OST;
   OVERLAPPED OSR;
   HANDLE Port;
};