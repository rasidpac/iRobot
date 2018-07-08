#include <stdio.h>
#include <windows.h>
#include "UARTClass.h"

#define IRC_ComPort 2
#define IRC_Start 128
#define IRC_Baud 129
#define IRC_Safe 131
#define IRC_Full 132
#define IRC_Drive 137
#define IRC_DirDrive 145
#define IRC_Baud_56700 10
#define IRC_Bytes 8
#define IRC_Parity 0
#define IRC_StopBits 1


class iRobotClass
{
public:
	iRobotClass();
	~iRobotClass();
	void iRobotClass::setData(UINT32 iBR, UINT8 iBS, UINT8 iP, UINT8 iSB, UINT32 iCP);
    void iRobotClass::initialize(UARTCHANNEL &serport);
	void iRobotClass::sing(UARTCHANNEL &serport);
	void iRobotClass::close(UARTCHANNEL &serport);
	void iRobotClass::test(UARTCHANNEL &serport);
private:
	UINT32 iComPort;
	UINT8 iBaudRate;
	UINT8 iByteSize;
	UINT8 iParity;
	UINT32 iStopBits;
};