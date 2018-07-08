#include "blobClass.h"
#include <iostream>

using namespace cv;
using namespace std;

blob2::blob2()
{
	setThresholdsB();
	traj.blobsCurrAngle = -1;
	traj.blobsCurrPt.x = -1;
	traj.blobsCurrPt.y = -1;
	traj.trajCurrAngle = -1;
	traj.trajCurrPt.x = -1;
	traj.trajCurrPt.y = -1;
	traj.trajCurrVelOmega = -1;
	traj.trajEndAngle = -1;
	traj.trajEndPt.x = -1;
	traj.trajEndPt.y = -1;
	traj.trajInitAngle = -1;
	traj.trajInitPt.x = -1;
	traj.trajInitPt.y = -1;
}

blob2::~blob2()
{

}

void blob2::blobHSV(IplImage* srcImg, IplImage* dstImg)
{
	int i,j;//for iterations
	uchar* srcData = (uchar*) srcImg->imageData;
	uchar* dstData = (uchar*) dstImg->imageData;
	int height = srcImg->height;
	int width = srcImg->width;
	int step = srcImg->widthStep;	
	int channels = 3;
	int stepmono = (int) step/channels;

	unsigned int nRed = 0;
	unsigned int nBlue = 0;

	//int rectX = (int) ( (double) width/3);
	//int rectY = (int) ( (double) height/3);
	//CvRect roi = cvRect(rectX, rectY, rectX, rectY);
	//cvSetImageROI( srcImg, roi);
	
	clrPose();

	for(i=0; i<height; i++)
	{
		for(j=0; j<width; j++)
		{			
			if( ( hueRed_h > hueRed_l && (srcData[i*step+j*channels] <= hueRed_h && srcData[i*step+j*channels] >= hueRed_l) ) ||
				( hueRed_h < hueRed_l && (srcData[i*step+j*channels] <= hueRed_h || srcData[i*step+j*channels] >= hueRed_l) ) )
				if( (srcData[i*step+j*channels+1] <= satRed_h) && (srcData[i*step+j*channels+1] >= satRed_l) )
				{
					dstData[i*stepmono+j] = RED;									
					redU = redU + j;
					redV = redV + i;
					nRed++;
				}
				else
					dstData[i*stepmono+j] = 0;

			else if( ( hueBlue_h > hueBlue_l && (srcData[i*step+j*channels] <= hueBlue_h && srcData[i*step+j*channels] >= hueBlue_l) ) ||
				     ( hueBlue_h < hueBlue_l && (srcData[i*step+j*channels] <= hueBlue_h || srcData[i*step+j*channels] >= hueBlue_l) ) )
				if( (srcData[i*step+j*channels+1] <= satBlue_h) && (srcData[i*step+j*channels+1] >= satBlue_l) )				
				{
					dstData[i*stepmono+j] = BLUE;									
					blueU = blueU + j;
					blueV = blueV + i;
					nBlue++;
				}
				else
					dstData[i*stepmono+j] = 0;

			else 
				dstData[i*stepmono+j] = 0;
		}
	}

	cvErode(dstImg,dstImg,0,2);
	cvDilate(dstImg,dstImg,0,2);	

	if(nRed)
	{
		redU = redU/nRed;
		redV = redV/nRed;
	}
	else
	{
		//redU = -1;
		//redV = -1;
	}

	if(nBlue)
	{
		blueU = blueU/nBlue;
		blueV = blueV/nBlue;
	}
	else
	{
		//blueU = -1;
		//blueV = -1;
	}

}

void blob2::clrPose(void)
{
	redU = -1;
	redV = -1;
	blueU = -1;
	blueV = -1;
}

void blob2::setThresholdsA(void)
{
	IplImage* colorRB = cvLoadImage("red.jpg", CV_LOAD_IMAGE_COLOR);

	int height, width, step, chan, i, j;
	uchar* data;

	double totalHue = 0;
	double totalSat = 0;
	
	cvCvtColor(colorRB, colorRB, CV_RGB2HSV);
	data = (uchar*) colorRB->imageData;

	height = colorRB->height;
	width = colorRB->width;
	step = colorRB->widthStep;
	chan = colorRB->nChannels;

	for(i=0; i<height; i++)
	{
		for(j=0; j<width; j++)
		{			
			totalHue = (totalHue + (double) data[i*step+j*chan]);
			totalSat = (totalSat + (double) data[i*step+j*chan+1]);
		}
	}

	totalHue = totalHue/(height*width);
	totalSat = totalSat/(height*width);

	avgHueRed = (unsigned char) totalHue;
	avgSatRed = (unsigned char) totalSat;
	
	hueRed_h = ( (unsigned char) totalHue + HYST ) % 180;

	if( totalHue - HYST < 0 )
		hueRed_l = ( 180 + (unsigned char) totalHue - HYST );
	else
		hueRed_l = ( (unsigned char) totalHue - HYST );
	
	if (totalSat + HYST > 255)
		satRed_h = 255;
	else
		satRed_h = (unsigned char) totalSat + HYST;

	if( totalSat - HYST < 0 )
		satRed_l = 0;
	else
		satRed_l = ( 255 + (unsigned char) totalSat - HYST );
	
	colorRB = cvLoadImage("blue.jpg", CV_LOAD_IMAGE_COLOR);

	cvCvtColor(colorRB, colorRB, CV_RGB2HSV);
	data = (uchar*) colorRB->imageData;

	height = colorRB->height;
	width = colorRB->width;
	step = colorRB->widthStep;
	chan = colorRB->nChannels;
	
	totalHue = 0;
	totalSat = 0;

	for(i=0; i<height; i++)
	{
		for(j=0; j<width; j++)
		{			
			totalHue = (totalHue + (double) data[i*step+j*chan]);
			totalSat = (totalSat + (double) data[i*step+j*chan+1]);
		}
	}

	totalHue = totalHue/(height*width);
	totalSat = totalSat/(height*width);

	avgHueBlue = (unsigned char) totalHue;
	avgSatBlue = (unsigned char) totalSat;
	
	hueBlue_h = ( (unsigned char) totalHue + HYST ) % 180;

	if( totalHue - HYST < 0 )
		hueBlue_l = ( 180 + (unsigned char) totalHue - HYST );
	else
		hueBlue_l = ( (unsigned char) totalHue - HYST );
	
	if (totalSat + HYST > 255)
		satBlue_h = 255;
	else
		satBlue_h = (unsigned char) totalSat + HYST;

	if( totalSat - HYST < 0 )
		satBlue_l = 0;
	else
		satBlue_l = ( (unsigned char) totalSat - HYST );

	cvReleaseImage(&colorRB);
}

void blob2::setThresholdsB(void)
{
	hueRed_h	= HUERED_H;
	hueRed_l	= HUERED_L;
	hueBlue_h	= HUEBLUE_H;
	hueBlue_l	= HUEBLUE_L;

	satRed_h	= SATRED_H;
	satRed_l	= SATRED_L;
	satBlue_h	= SATBLUE_H;
	satBlue_l	= SATBLUE_L;
}

void blob2::setThresholdsC(IplImage* firstImage, CvRect* redRect, CvRect* blueRect)
{
	IplImage* colorRB = cvCloneImage(firstImage);
	cvSetImageROI(colorRB, *redRect);

	int step, chan, i, j;
	uchar* data;

	double totalHue = 0;
	double totalSat = 0;
	
	cvCvtColor(colorRB, colorRB, CV_RGB2HSV);
	data = (uchar*) colorRB->imageData;

	step = colorRB->widthStep;
	chan = colorRB->nChannels;

	for(i=redRect->y; i<redRect->y+redRect->height; i++)
	{
		for(j=redRect->x; j<redRect->x+redRect->width; j++)
		{			
			totalHue = (totalHue + (double) data[i*step+j*chan]);
			totalSat = (totalSat + (double) data[i*step+j*chan+1]);
		}
	}

	totalHue = totalHue/(redRect->height*redRect->width);
	totalSat = totalSat/(redRect->height*redRect->width);

	avgHueRed = (unsigned char) totalHue;
	avgSatRed = (unsigned char) totalSat;
	
	hueRed_h = ( (unsigned char) totalHue + HYST ) % 180;

	if( totalHue - HYST < 0 )
		hueRed_l = ( 180 + (unsigned char) totalHue - HYST );
	else
		hueRed_l = ( (unsigned char) totalHue - HYST );
	
	if (totalSat + HYST > 255)
		satRed_h = 255;
	else
		satRed_h = (unsigned char) totalSat + HYST;

	if( totalSat - HYST < 0 )
		satRed_l = 0;
	else
		satRed_l = ( 255 + (unsigned char) totalSat - HYST );
	
	cvReleaseImage(&colorRB);
	colorRB = cvCloneImage(firstImage);
	cvSetImageROI(colorRB, *blueRect);

	cvCvtColor(colorRB, colorRB, CV_RGB2HSV);
	data = (uchar*) colorRB->imageData;

	step = colorRB->widthStep;
	chan = colorRB->nChannels;
	
	totalHue = 0;
	totalSat = 0;

	for(i=blueRect->y; i<blueRect->y+blueRect->height; i++)
	{
		for(j=blueRect->x; j<blueRect->x+blueRect->width; j++)
		{			
			totalHue = (totalHue + (double) data[i*step+j*chan]);
			totalSat = (totalSat + (double) data[i*step+j*chan+1]);
		}
	}

	totalHue = totalHue/(redRect->height*redRect->width);
	totalSat = totalSat/(redRect->height*redRect->width);
	
	avgHueBlue = (unsigned char) totalHue;
	avgSatBlue = (unsigned char) totalSat;

	hueBlue_h = ( (unsigned char) totalHue + HYST ) % 180;

	if( totalHue - HYST < 0 )
		hueBlue_l = ( 180 + (unsigned char) totalHue - HYST );
	else
		hueBlue_l = ( (unsigned char) totalHue - HYST );
	
	if (totalSat + HYST > 255)
		satBlue_h = 255;
	else
		satBlue_h = (unsigned char) totalSat + HYST;

	if( totalSat - HYST < 0 )
		satBlue_l = 0;
	else
		satBlue_l = ( (unsigned char) totalSat - HYST );

	cvReleaseImage(&colorRB);
}

unsigned char blob2::getAvgHue(unsigned char color)
{
	if(color == RED)
		return avgHueRed;
	else if (color == BLUE)
		return avgHueBlue;
	else
		return 0;
}

unsigned char blob2::getAvgSat(unsigned char color)
{
	if(color == RED)
		return avgSatRed;
	else if (color == BLUE)
		return avgSatBlue;
	else
		return 0;
}

void blob2::setTrajEndPt(unsigned int u, unsigned int v, double theta)
{
	traj.trajEndPt.x = u;
	traj.trajEndPt.y = v;
	traj.trajEndAngle = theta;
	//traj.trajEndPt.x = 284;
	//traj.trajEndPt.y = 328;
	//traj.trajEndAngle = 3.14159/3;
}

void blob2::setTrajInitPt(unsigned int u, unsigned int v, double theta)
{
	traj.trajInitPt.x = u;
	traj.trajInitPt.y = v;
	traj.trajInitAngle = theta;
}

const trajectory* blob2::getCurrTraj(double s)
{
	double k = CURVE_K;
	double Xf[3] = {traj.trajEndPt.x, traj.trajEndPt.y, traj.trajEndAngle};
	double Xi[3] = {traj.trajInitPt.x, traj.trajInitPt.y, traj.trajInitAngle};

	double alpha_x = k*cos(Xf[2])-3*Xf[0];
	double beta_x = k*cos(Xi[2])+3*Xi[0];
	double alpha_y = k*sin(Xf[2])-3*Xf[1];
	double beta_y = k*sin(Xi[2])+3*Xi[1];

	traj.trajCurrPt.x = pow(s,3)*Xf[0] - pow(s-1,3)*Xi[0] + alpha_x*pow(s,2)*(s-1) + beta_x*s*pow(s-1,2);
    traj.trajCurrPt.y = pow(s,3)*Xf[1] - pow(s-1,3)*Xi[1] + alpha_y*pow(s,2)*(s-1) + beta_y*s*pow(s-1,2);
    traj.trajCurrVelX = 3*pow(s,2)*Xf[0] - 3*pow(s-1,2)*Xi[0] + alpha_x*( 2*s*(s-1)+pow(s,2) ) + beta_x*( pow(s-1,2) + 2*s*(s-1) );
    traj.trajCurrVelY = 3*pow(s,2)*Xf[1] - 3*pow(s-1,2)*Xi[1] + alpha_y*( 2*s*(s-1)+pow(s,2) ) + beta_y*( pow(s-1,2) + 2*s*(s-1) );     
	traj.trajCurrVelOmega = pow(traj.trajCurrVelX,2)/( pow(traj.trajCurrVelX,2) + pow(traj.trajCurrVelY,2) );
    traj.trajCurrAngle = atan2(traj.trajCurrVelY, traj.trajCurrVelX);

	return &traj;
}

const trajectory* blob2::getTrajPtr(void)
{
	return &traj;
}

const doublePt blob2::getBlobsCurrPt(void)
{
	traj.blobsCurrPt.x = (int) (redU+blueU)/2;
	traj.blobsCurrPt.y = (int) (redV+blueV)/2;
	return traj.blobsCurrPt;
}

double blob2::getBlobsCurrAngle(void)
{
	//traj.blobsCurrAngle = 180*atan2(redV-blueV, redU-blueU)/3.14159;
	traj.blobsCurrAngle = atan2(redV-blueV, redU-blueU);
	return traj.blobsCurrAngle;
}

double blob2::getTrajEndAngle(void)
{
	return traj.trajEndAngle;
}

const doublePt blob2::getTrajEndPt(void)
{
	return traj.trajEndPt;
}

void blob2::plotTrajectory(IplImage* currImg)
{
	int i;
	int N = 100;
	double s = 0;
	CvPoint pPrev, pCurr;
	CvScalar color = cvScalar(0x00,0xff,0x00);

	const trajectory* traj = getCurrTraj(0);
	pPrev.x = traj->trajInitPt.x;
	pPrev.y = traj->trajInitPt.y;

	for(i=1; i<=N; i++)
	{
		s = ((double) i)/((double) N);
		traj = getCurrTraj(s);
		pCurr.x = traj->trajCurrPt.x;
		pCurr.y = traj->trajCurrPt.y;
		cvLine(currImg, pPrev, pCurr, color, 1, 8, 0 );
		pPrev.x = pCurr.x;
		pPrev.y = pCurr.y;
	}
}