void onMouse(int event, int u, int v, int flags, void *param); //This will be called when the user performs a mouse action.

typedef enum state {INIT, RED_CALIB, BLUE_CALIB, CALIB_DONE, ANGLE_SET, PLOT_TRAJ, BLOB_LOST, READY, ON_MOVE, ON_PID, DUMMY};

typedef enum motion_type {TRAJ, P2P};

struct testPoint
{
	double x, y, th, e0, e1, u, w;
};

#define CAMNO 0		// 0 for integrated webcam, 1 for usb camera

#define ESC 27
#define ENTER 13
#define SPACEBAR 32

#define PERIOD 1		//
#define STEP 0.01
#define BEAMLEN 2

#define LAMBDA_1 0.05
#define LAMBDA_2 0.05

#define PI 3.14159
#define STOP_BAND 15 // pixel distance for stopping