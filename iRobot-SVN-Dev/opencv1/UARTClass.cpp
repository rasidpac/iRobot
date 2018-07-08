#include "UARTClass.h"

UARTCHANNEL::UARTCHANNEL()
{
   uBaudRate = 57600; // Baud Rate
   uByteSize = 8; // Byte Size
   uParity = 0; // No Parity
   uStopBits = 0; // 1 Stop Bit
   uComPort = 2;

   memset(&OSR, 0, sizeof(OVERLAPPED)) ;        //wipe the overlapped struct
   memset(&OST, 0, sizeof(OVERLAPPED)) ;        //wipe the overlapped struct
   //memset(&Settings, 0, sizeof(COM_PORT_ENHANCED)); //wipe the COM_PORT_ENHANCED struct

   CommunicationTimeOuts.ReadIntervalTimeout = 25;
   CommunicationTimeOuts.ReadTotalTimeoutMultiplier = 2;
   CommunicationTimeOuts.ReadTotalTimeoutConstant = 0;
   CommunicationTimeOuts.WriteTotalTimeoutMultiplier = 2;
   CommunicationTimeOuts.WriteTotalTimeoutConstant = 0;

   uTxMessageSize = MAX_TX_MESSAGE_SIZE;
   memset(auTxBuffer, 0, MAX_TX_MESSAGE_SIZE);       //wipe the txbuffer

   uRxMessageSize = MAX_RX_MESSAGE_SIZE;
   memset(auRxBuffer, 0, MAX_RX_MESSAGE_SIZE);       //wipe the rxbuffer
}

void UARTCHANNEL::setData(UINT32 uBR, UINT8 uBS, UINT8 uP, UINT8 uSB, UINT8 uCP)
{
   uBaudRate = uBR; // Baud Rate
   uByteSize = uBS; // Byte Size
   uParity = uP; // Parity
   uStopBits = uSB; // Stop Bit
   uComPort = uCP;
}

void UARTCHANNEL::initialize()
{
   char acBuf[15];
   DCB MDCB;

   // create I/O event used for overlapped write
   OST.hEvent = CreateEvent(NULL,     // no security
                            TRUE,    // explicit reset req
                            FALSE,   // initial event reset
                            NULL) ;  // no name
   if (OST.hEvent == NULL)
   {
      //Failed to create event for thread!
      return;
   }

   // create I/O event used for overlapped read

   OSR.hEvent = CreateEvent(NULL,     // no security
                            TRUE,    // explicit reset req
                            FALSE,   // initial event reset
                            NULL) ;  // no name
   if (OSR.hEvent == NULL)
   {
      //Failed to create event for thread!
      CloseHandle(OST.hEvent);
      return;
   }

   sprintf(acBuf, "\\\\.\\COM%u", uComPort);
   //wsprintf( acBuf, "\\\\.\\COM%u", uComPort );
   Port = CreateFileA(acBuf,
                     GENERIC_READ | GENERIC_WRITE,
                     0,
                     NULL,
                     OPEN_EXISTING,
                     FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                     NULL
                    );

   if (Port == INVALID_HANDLE_VALUE)
   {
      //fubar here
      //cannot open handle to COM Port
      CloseHandle(OST.hEvent);
      CloseHandle(OSR.hEvent);
      return;
   }

   GetCommState(Port, &MDCB);

   MDCB.BaudRate = uBaudRate;
   MDCB.fBinary = 1;
   MDCB.fParity = 0;
   MDCB.fOutxCtsFlow = 0;
   MDCB.fOutxDsrFlow = 0;
   MDCB.fOutX = 0;
   MDCB.fInX = 0;
   MDCB.fRtsControl = 0;
   MDCB.ByteSize = uByteSize;
   MDCB.Parity = uParity;
   MDCB.StopBits = uStopBits;

   SetCommState(Port, &MDCB);

   SetupComm(Port, 5000, 5000);

   PurgeComm(Port, PURGE_TXCLEAR | PURGE_RXCLEAR);

   SetCommTimeouts(Port, &CommunicationTimeOuts);
}


bool UARTCHANNEL::transmitData(UINT32 uSize)
{
   bool bReturn = false;
   ULONG uReturn;
   ULONG uNumberOfWords;
   UINT32 uWaitTimeOut = 0;
   UINT32 uIndex = 0;

   uReturn = WriteFile(Port, auTxBuffer, uSize, &uNumberOfWords, &OST);
   uTxMessageSize = uNumberOfWords;

   if (uReturn == FALSE)
   {
      if (GetLastError() == ERROR_IO_PENDING)
      {
         do
         {
            uIndex = WaitForSingleObject(OST.hEvent, 250);  //250 ms delay
            if (uIndex == WAIT_TIMEOUT)
            {
               uWaitTimeOut++;
               if (uWaitTimeOut > 4)
               {
                  printf("COM5: Write timeout.\n");
                  break;
               }
            }
         }
         while (uIndex != WAIT_OBJECT_0);

         if (uWaitTimeOut > 4)
         {
            printf("Writefile failed! Purging buffers and canceling Write()!\n");
            PurgeComm(Port, PURGE_TXABORT | PURGE_TXCLEAR);
         }
      }
      else
      {
         printf("\r\na Write ERROR occured on COM5\r\n");
         printf("Failed with 0x%X error", GetLastError());
      }
   }

   if (uIndex == 0)
   {
      bReturn = true;
   }
   Sleep(SLEEP_TIME);
   return bReturn;
}

bool UARTCHANNEL::receiveData(UINT32 uSize)
{
   bool bReturn = true;
   ULONG uReturn;
   ULONG uNumberOfWords;
   UINT32 uWaitTimeOut = 0;
   UINT32 uIndex = 0;

   uReturn = ReadFile(Port, auRxBuffer, uSize, &uNumberOfWords, &OSR);

   if ((uReturn == FALSE) && (GetLastError() == ERROR_IO_PENDING))
   {
      uWaitTimeOut = uIndex = 0;
      while (uIndex != WAIT_OBJECT_0)
      {
         uIndex = WaitForSingleObject(OSR.hEvent, 250);  //250 ms timeout
         if (uIndex == WAIT_TIMEOUT)
         {
            uWaitTimeOut++;
            if (uWaitTimeOut > 4)
            {
               break;
            }
            printf("\nCOM3: Read timeout.\n");
         }
         else if (uIndex == WAIT_ABANDONED)
         {
            // Hopefully you won't ever get here.
            printf("\nCOM3: Read abandoned.\n");
            bReturn = false;
            break;
         }
      }
      if (uWaitTimeOut > 4)
      {
         printf("Readfile failed! Purging buffers and canceling Read()!\n");
         bReturn = false;
         PurgeComm(Port, PURGE_RXABORT | PURGE_RXCLEAR);
      }
      GetOverlappedResult(Port, &OSR, &uNumberOfWords, TRUE); //here to get the actual nobytesread!!!
      uRxMessageSize = uNumberOfWords;
   }

   return bReturn;
}

void UARTCHANNEL::clearRxBuffer()
{
   SetupComm(Port, 5000, 5000);
   PurgeComm(Port, PURGE_RXABORT | PURGE_RXCLEAR);
   Sleep(50);
}

void UARTCHANNEL::close()
{
   CloseHandle(Port);
   CloseHandle(OST.hEvent);
   CloseHandle(OSR.hEvent);
}

UARTCHANNEL::~UARTCHANNEL()
{
}