#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <math.h>

#define HUERED_H 140
#define HUERED_L 90
#define SATRED_H 255
#define SATRED_L 170

#define HUEBLUE_H 40
#define HUEBLUE_L 0
#define SATBLUE_H 255
#define SATBLUE_L 170

#define RED 255
#define BLUE 180
#define HYST 30

#define CURVE_K 800.0

//typedef enum error_type {PID, CURVE};

struct doublePt
{
	double x;
	double y;
};

struct trajectory
{
	doublePt trajInitPt, trajEndPt, trajCurrPt, blobsCurrPt;
	double trajInitAngle, trajEndAngle, trajCurrAngle, blobsCurrAngle;
	double trajCurrVelX, trajCurrVelY, trajCurrVelOmega;
};

class blob2
{
	public:
		blob2();
		~blob2();
		void blobHSV(IplImage* , IplImage*);
		void clrPose(void);
		void setThresholdsA(void);
		void setThresholdsB(void);
		void blob2::setThresholdsC(IplImage*, CvRect*, CvRect*);				
		
		void setTrajEndPt(unsigned int, unsigned int, double);		
		void setTrajInitPt(unsigned int, unsigned int, double);
		const trajectory* getCurrTraj(double);
		const trajectory* getTrajPtr(void);
		const doublePt getBlobsCurrPt(void);
		double getBlobsCurrAngle(void);
		double getTrajEndAngle(void);
		const doublePt getTrajEndPt(void);
		void plotTrajectory(IplImage*);
		unsigned char getAvgHue(unsigned char);
		unsigned char getAvgSat(unsigned char);
	private:
		double redU, redV, blueU, blueV;
		unsigned char hueRed_h, hueRed_l, hueBlue_h, hueBlue_l, satRed_h, satRed_l, satBlue_h, satBlue_l;
		unsigned char avgHueRed, avgSatRed, avgHueBlue, avgSatBlue;
		trajectory traj;
};