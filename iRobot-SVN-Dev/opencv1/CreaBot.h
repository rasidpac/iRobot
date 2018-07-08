#include "UARTClass.h"

#define COMPORT 6		//Change this
#define BAUDRATE 57600	//57600 bps
#define DATABIT 8		//8 data bits
#define PARITY 0		//No parity
#define STOPBIT 0		//1 stop bit

#define START 128
#define BAUD 129
#define SAFE 131
#define FULL 132
#define PASSIVE 128
#define DEMO 136
#define DDRIVE 145

#define DIAMETER 262.5
#define RADIUS (DIAMETER/2)
#define MAX_SPEED 500		// maximum direct drive speed in mm/s

#define KPV 25		// PID gain constant for translational and rotational velocity
#define KIV 0
#define KDV 0
#define KPW 10
#define KIW 0
#define KDW 0

class CreaBot
{
public:
   CreaBot(int);
   ~CreaBot();
   void directDrive(double, double);
   void quickStop(void);
   void demo(signed char);
   void PIDControl(double, double);
private:
   	UARTCHANNEL serport;
};