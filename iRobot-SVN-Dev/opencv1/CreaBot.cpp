#include "CreaBot.h"
#include <math.h>

CreaBot::CreaBot(int portNo)
{
	serport.auTxBuffer[0] = START;

	serport.setData(BAUDRATE, DATABIT, PARITY, STOPBIT, portNo);	// set baud rate, data bits, parity, stop bit, and the port number
	serport.initialize();	
	serport.transmitData(1);

	serport.auTxBuffer[0] = FULL;
	serport.transmitData(1);
}

CreaBot::~CreaBot()
{
	quickStop();
	serport.clearRxBuffer();	// clear rx buffer before exiting	
	serport.close();
}

void CreaBot::demo(signed char demoNum)
{
	char command[2] = {DEMO, demoNum};
	serport.auTxBuffer[0] = command[0];
	serport.auTxBuffer[1] = command[1];
	
	serport.transmitData(2);
}

void CreaBot::directDrive(double v, double w)
{
	double vR = v - w*RADIUS;
	double vL = v + w*RADIUS;
	
	if ( abs(vR) > MAX_SPEED || abs(vL) > MAX_SPEED )
	{
		if( abs(vR) > abs(vL) )
		{
			vL = vL*MAX_SPEED/abs(vR);
			if( vR > 0 ){
				vR = MAX_SPEED;
			}
			else{
				vR = -MAX_SPEED;
			}
		}
		else
		{
			vR = vR*MAX_SPEED/abs(vL);
			if( vL > 0 ){
				vL = MAX_SPEED;
			}
			else{
				vL = -MAX_SPEED;
			}
		}
	}

	serport.auTxBuffer[0] = DDRIVE;
	serport.auTxBuffer[1] = (char) ( ((signed short) vR) >> 8 );
	serport.auTxBuffer[2] = (char) ( ((signed short) vR) );
	serport.auTxBuffer[3] = (char) ( ((signed short) vL) >> 8 );
	serport.auTxBuffer[4] = (char) ( ((signed short) vL) );

	serport.transmitData(5);
}

void CreaBot::quickStop(void)
{
	directDrive(0.0, 0.0);
}

void CreaBot::PIDControl(double errDist, double errAngle)
{
	double v = KPV*errDist;
	double w = KPW*errAngle;

	printf("v = %.1f w = %.1f\r\n", v, w);
	directDrive(v, w);	
}
