#include "iRobotClass.h"

iRobotClass::iRobotClass()
{
	iComPort = IRC_ComPort;
	iBaudRate = IRC_Baud_56700;
	iByteSize = IRC_Bytes;
	iParity = IRC_Parity;
	iStopBits = IRC_StopBits;

}

iRobotClass::~iRobotClass()
{
}

void iRobotClass::initialize(UARTCHANNEL &serport)
{
	serport.setData(iBaudRate, iByteSize, iParity, iStopBits, iComPort);	// set baud rate, data bits, parity, stop bit, and the port number
	serport.initialize();
	serport.auTxBuffer[0] = IRC_Start;	// send 128 opcode
	serport.transmitData(2);			// transmit data
	Sleep(100);							// wait 100ms

}

void iRobotClass::setData(UINT32 iBR, UINT8 iBS, UINT8 iP, UINT8 iSB, UINT32 iCP)
{
   iBaudRate = iBR; // Baud Rate
   iByteSize = iBS; // Byte Size
   iParity = iP; // Parity
   iStopBits = iSB; // Stop Bit
   iComPort = iCP;
}

void iRobotClass::test(UARTCHANNEL &serport)
{
	char line[6];


	//insert send code here


		// if pins 2 and 3 of the RS232 port are shorted, sent data will echo
	if( serport.receiveData(5) )			// see if the same number of bytes received
	{
		line[0] = serport.auRxBuffer[0];
		line[1] = serport.auRxBuffer[1];
		line[2] = serport.auRxBuffer[2];
		line[3] = serport.auRxBuffer[3];
		line[4] = serport.auRxBuffer[4];
		line[5] = '\0';

		printf(line);
	}
	else
		printf("Size %d", serport.uRxMessageSize);

	printf("\r\nStopping..\r\n");

	Sleep(3000);
	
	serport.clearRxBuffer();	// clear rx buffer before exiting
	serport.close();			// release the port before exiting
}

void iRobotClass::sing(UARTCHANNEL &serport)
{
	UINT8 a = 140;
	UINT8 b = 1;
	UINT8 c = 48;
	UINT8 d = 20;
	UINT8 e = 141;
	serport.auTxBuffer[0] = a;
	serport.auTxBuffer[2] = b;
	serport.auTxBuffer[4] = b;
	serport.auTxBuffer[6] = c;
	serport.auTxBuffer[8] = b;
	serport.transmitData(10);
	Sleep(1000);
	serport.auTxBuffer[0] = e;
	serport.auTxBuffer[2] = b;
	serport.transmitData(4);
	Sleep(1000);
}

void iRobotClass::close(UARTCHANNEL &serport)
{
	serport.close();
}