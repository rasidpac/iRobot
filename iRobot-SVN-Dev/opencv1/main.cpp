#include <windows.h>
#include <iostream>
#include <list>

#include "blobClass.h"
#include "CreaBot.h"
#include "main.h"

using namespace cv;
using namespace std;

//--- Global variables partly to exchance data between main function and the mouse callback ---
state progState = INIT;		// State of the program
CvRect redRect, blueRect;
CvRect trajEndAngle;
bool isDrawing = false;

int main( int argc, char** argv )
{ 	
	//------------------------------------------------------
	int i,j,k;//for iterations
	i=j=k=0;
	int height,width,step,channels, depth, origin, order; 
	uchar *data,*datamono;
	char userInput;

	progState = INIT;
	unsigned int timer = 0;
	unsigned int counter = 0;
	double s = 0;
	const trajectory* trajPtr = NULL;

	double tempAngle, desAngle;
	double error[2] = {0, 0};
	double control[2] = {0, 0};
	double invTmat[2][2] = {{1, 0},{0, 1}};	
	double lambda[2] = {LAMBDA_1, LAMBDA_2};
	doublePt currPt;
	currPt.x = -1;
	currPt.y = -1;
	doublePt desPt;
	desPt.x = -1;
	desPt.y = -1;
	doublePt desVel;
	desVel.x = 0;
	desVel.y = 0;
	doublePt prevDesPt;
	prevDesPt.x = -1;
	prevDesPt.y = -1;

	motion_type motion = TRAJ;

	list <testPoint> robotPts;
	testPoint testPt;
	list <testPoint>::iterator testPtItr;
	list <testPoint> trajPts;

	blob2 redblue;

	IplImage* frame = NULL;
	IplImage* hsvFrame = NULL;

	//----- Robot Communication Initialization -----
	printf("----- Create iRobot Initialization -----\r\n");
	printf("Enter COM port number (1, 2, ..., 9) for communication: ");
	while(1)
	{
		cin >> userInput;
		if(userInput < '1' || userInput > '9')
		{
			printf("Wrong port number selection!\r\n");
			printf("Enter COM port number between 1-9): ");
		}
		else
		{
			userInput = userInput - '0';
			break;
		}
	}
	
	CreaBot iCreate(userInput);
	printf("Comport has been opened.\n\r");

	//----- Motion type selection -----
	printf("Choose motion type: 1 for trajectory, 2 for point-to-point: ");
	while(1)
	{
		cin >> userInput;
		if(userInput == '1')
		{
			motion = TRAJ;
			break;
		}
		else if(userInput == '2')
		{
			motion = P2P;
			break;
		}
		else
		{
			printf("Wrong selection!\r\n");
			printf("Choose motion type: 1 for trajectory, 2 for point-to-point: \r\n");
		}
	}
	//----- Console info feed --------------------------------------------
	CvCapture* capture = cvCaptureFromCAM(CAMNO);

	if(capture) // check if we succeeded
	{		
		if(cvGrabFrame(capture))
		{
			frame = cvRetrieveFrame(capture);
			hsvFrame = cvCloneImage(frame);
			height = frame->height;
			width = frame->width;
			step = frame->widthStep;						
			channels = frame->nChannels;
			data = (uchar*) frame->imageData;
			depth  = frame->depth;
			origin = frame->origin;
			order = frame->dataOrder;

			printf("\n\r--- Important image properties ---\n\r");
			printf("1- Height, width, and step: %d, %d, %d\n\r", height, width, step);
			printf("2- Channels: %d\n\r", channels);

			if( depth == IPL_DEPTH_8U )
				printf("3- Depth: Unsigned 8-bit integer (8u)\n\r");
			else if( depth == IPL_DEPTH_8S )
				printf("3- Depth: Signed 8-bit integer (8s)\n\r");
			else if( depth == IPL_DEPTH_16S )
				printf("3- Depth: Signed 16-bit integer (16s)\n\r");
			else if( depth == IPL_DEPTH_32S )
				printf("3- Depth: Signed 32-bit integer (32s)\n\r");
			else if( depth == IPL_DEPTH_32F )
				printf("3- Depth: 32-bit floating-point single-precision (32f)\n\r");
			else if( depth == IPL_DEPTH_64F )
				printf("3- Depth: 64-bit floating-point double-precision (64f)\n\r");
			else
				printf("3- Depth: Unrecognized!\n\r");

			if( origin == IPL_ORIGIN_TL)
				printf("4- Origin: Top left\n\r");
			else if(origin == IPL_ORIGIN_BL) 
				printf("4- Origin: Bottom left\n\r");
			else
				printf("4- Origin: Unrecognized!\n\r");

			if( order == IPL_DATA_ORDER_PIXEL)
				printf("5- Data order: Interleaved (pixel)\n\r");
			else
				printf("5- Data order: Plane\n\r");
		}
		else
		{
			printf("Image couldn't be grabbed!\n\r");
			cvReleaseCapture(&capture);
			return -1;
		}				
	}
	else
	{
		printf("Cannot capture video!\n\r");
		return -1;
	}
	//----- End of console info feed --------------------------------------------

	IplImage* monoImg = cvCreateImage( cvGetSize(frame), 8, 1 );
	datamono = (uchar*) monoImg->imageData;

	//create a window for the video,one for the monochrome image and the other for the original frame
	cvNamedWindow("Tracking", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("SceneVideo", CV_WINDOW_AUTOSIZE);
	cvShowImage("SceneVideo", frame);
	
	cvSetMouseCallback("SceneVideo", onMouse, (void*) frame);

	//----- MAIN PROGRAM LOOP --------------------------------------------
	while(1)
	{				
		switch (progState)
		{
			case INIT:
				if(cvGrabFrame(capture))
				{
					frame = cvRetrieveFrame(capture);
				}
				else
				{
					printf("Image couldn't be grabbed!\n\r");
					break;
				}						
				cvShowImage("SceneVideo", frame);
				break;

			case RED_CALIB:					//----- Red color calibration -----								
								
				break;

			case BLUE_CALIB:				//----- Blue color calibration -----				
								
				break;

			case CALIB_DONE:				
				if(cvGrabFrame(capture))
				{
					frame = cvRetrieveFrame(capture);
				}
				else
				{
					printf("Image couldn't be grabbed!\n\r");
					break;
				}
				redblue.setThresholdsB();
				//redblue.setThresholdsC(frame, &redRect, &blueRect);
				printf("Red color averages : hue = %u, sat = %u\n\r", redblue.getAvgHue(RED), redblue.getAvgSat(RED));
				printf("Blue color averages: hue = %u, sat = %u\n\r", redblue.getAvgHue(BLUE), redblue.getAvgSat(BLUE));
				progState = READY;				
				printf("\n\r----- Main program loop has started -----\n\r");
				break;

			case ANGLE_SET:
				if (!isDrawing)
				{
					//cvSetMouseCallback("SceneVideo", onMouse, (void*) &redblue);					
					redblue.setTrajInitPt( redblue.getBlobsCurrPt().x + BEAMLEN*cos(redblue.getBlobsCurrAngle()),
										  redblue.getBlobsCurrPt().y + BEAMLEN*sin(redblue.getBlobsCurrAngle()), 
										  redblue.getBlobsCurrAngle() );					
					redblue.setTrajEndPt( trajEndAngle.x, trajEndAngle.y, atan2( (double) trajEndAngle.height, (double) trajEndAngle.width) );
					progState = PLOT_TRAJ;
				}				
				break;

			case READY:
				counter = 0;
				if(cvGrabFrame(capture))
				{
					frame = cvRetrieveFrame(capture);
				}
				else
				{
					printf("Image couldn't be grabbed!\n\r");
					break;
				}		
				
				//cvShowImage("SceneVideo", frame);	
				
				cvCvtColor(frame,hsvFrame,CV_RGB2HSV); //Converting the color space of the Video....
								
				redblue.blobHSV(hsvFrame, monoImg);

				printf("Current pos u, v, theta: (%.1f, %.1f, %.1f)    \r", redblue.getBlobsCurrPt().x, redblue.getBlobsCurrPt().y, 180*redblue.getBlobsCurrAngle()/PI);				
				
				cvShowImage("Tracking", monoImg);				

				if( robotPts.size() > 1 )
				{
					testPtItr = robotPts.begin();
					for(int i=1; i<robotPts.size(); i++)
					{
						cvLine(frame, cvPoint((*testPtItr).x, (*testPtItr).y), cvPoint((*testPtItr++).x, (*testPtItr).y), cvScalar(0x00,0x00,0xff), 1, 8, 0 );
					}				
				}

				if( trajPts.size() > 1 )
				{
					testPtItr = trajPts.begin();
					for(int i=1; i<trajPts.size(); i++)
					{
						cvLine(frame, cvPoint((*testPtItr).x, (*testPtItr).y), cvPoint((*testPtItr++).x, (*testPtItr).y), cvScalar(0x00,0xff,0x00), 1, 8, 0 );
					}				
				}
				cvShowImage("SceneVideo", frame);
				break;

			case PLOT_TRAJ:
				counter = 0;
				timer = 0;
				robotPts.clear();
				trajPts.clear();
				redblue.plotTrajectory(frame);
				cvShowImage("SceneVideo", frame);
				progState = ON_MOVE;		// change this dummy to ON_MOVE
				printf("\n\rTarget pos u, v, theta: (%.1f, %.1f, %.1f)\r\n", redblue.getTrajEndPt().x, redblue.getTrajEndPt().y, 180*redblue.getTrajEndAngle()/PI );
				printf("\n\r----- Robot starts moving -----\n\r              ");
				break;

			case ON_MOVE:				
				//----- Display image -----				
				if(cvGrabFrame(capture))
				{
					frame = cvRetrieveFrame(capture);
				}
				else
				{
					printf("Image couldn't be grabbed!\n\r");
					break;
				}					
				
				cvCvtColor(frame,hsvFrame,CV_RGB2HSV); //Converting the color space of the Video....
								
				redblue.blobHSV(hsvFrame, monoImg);

				cvShowImage("Tracking", monoImg);

				//----- Display trajectory and the robot position -----
				if( robotPts.size() > 1 )
				{
					testPtItr = robotPts.begin();
					for(int i=1; i<robotPts.size(); i++)
					{
						cvLine(frame, cvPoint((*testPtItr).x, (*testPtItr).y), cvPoint((*testPtItr++).x, (*testPtItr).y), cvScalar(0x00,0x00,0xff), 1, 8, 0 );
					}				
				}

				if( trajPts.size() > 1 )
				{
					testPtItr = trajPts.begin();
					for(int i=1; i<trajPts.size(); i++)
					{
						cvLine(frame, cvPoint((*testPtItr).x, (*testPtItr).y), cvPoint((*testPtItr++).x, (*testPtItr).y), cvScalar(0x00,0xff,0x00), 1, 8, 0 );
					}				
				}

				cvShowImage("SceneVideo", frame);

				//----- Motion type -----
				if( (timer%PERIOD) == 0 )
				{					
					if(motion == TRAJ)
					{
						timer = 0;
						if( counter > (1.0/STEP) ) //after 400, stops robot
						{
							iCreate.quickStop();
							counter = 0;
							progState = READY;
						}
						else
						{	// Control low implementation
							s = counter*STEP;
							prevDesPt = desPt;		// Store the previous desired point
							trajPtr = redblue.getCurrTraj(s);	// Get new trajectory info for s
							
							currPt.x = redblue.getBlobsCurrPt().x + BEAMLEN*cos(trajPtr->blobsCurrAngle);
							currPt.y = redblue.getBlobsCurrPt().y + BEAMLEN*sin(trajPtr->blobsCurrAngle);
							desPt.x = trajPtr->trajCurrPt.x + BEAMLEN*cos(trajPtr->trajCurrAngle);
							desPt.y = trajPtr->trajCurrPt.y + BEAMLEN*sin(trajPtr->trajCurrAngle);
							
							if (counter > 0)
							{
								desVel.x = desPt.x - prevDesPt.x;
								desVel.y = desPt.y - prevDesPt.y;
							}
							else
							{
								desVel.x = 0;
								desVel.y = 0;
							}
							error[0] = currPt.x - desPt.x;		// error for x
							error[1] = currPt.y - desPt.y;		// error for y
							invTmat[0][0] = cos(trajPtr->blobsCurrAngle);
							invTmat[0][1] = sin(trajPtr->blobsCurrAngle);
							invTmat[1][0] = -sin(trajPtr->blobsCurrAngle)/BEAMLEN;
							invTmat[1][1] = cos(trajPtr->blobsCurrAngle)/BEAMLEN;

							control[0] = 60*(invTmat[0][0]*( -lambda[0]*error[0] + desVel.x) + invTmat[0][1]*( -lambda[1]*error[1] + desVel.y)); 
							control[1] = 1.3*(invTmat[1][0]*( -lambda[0]*error[0] + desVel.x) + invTmat[1][1]*( -lambda[1]*error[1] + desVel.y));

							printf("s = %.4f e_theta = %.01f\n", s, redblue.getBlobsCurrAngle() - trajPtr->trajCurrAngle);
							printf("e0 = %.1f e1 = %.1f v = %.1f w = %.001f\n", error[0], error[1], control[0], control[1]);					
							iCreate.directDrive(control[0], control[1]);
							
							//iCreate.PIDControl( pow(error[0],2)+pow(error[1],2), atan2(-error[1],-error[0]));
						}
						//----- Stopping condition ------
						if(sqrt( pow((trajPtr->trajEndPt.x - redblue.getBlobsCurrPt().x),2) + 
								 pow((trajPtr->trajEndPt.y - redblue.getBlobsCurrPt().y),2)  ) < STOP_BAND )
						{	
							iCreate.quickStop();	
							counter = 0;
							progState = READY;
						}
						counter++;
					}
					else
					{
						//------------- PID Controller Implementation ------------------
						//error[0] is the absolute distance to the target point
						error[0] = sqrt( pow(redblue.getTrajEndPt().y-redblue.getBlobsCurrPt().y,2)+
										 pow(redblue.getTrajEndPt().x-redblue.getBlobsCurrPt().x,2) );
						//desAngle is the bearing of the target point with respect to the robot current position
						desAngle = atan2( redblue.getTrajEndPt().y-redblue.getBlobsCurrPt().y, redblue.getTrajEndPt().x-redblue.getBlobsCurrPt().x ); 
						//tempAngle is the angular difference between vector from the robot position to target position and the vector of robot heading
						tempAngle = acos( cos(desAngle)*cos(redblue.getBlobsCurrAngle()) +  sin(desAngle)*sin(redblue.getBlobsCurrAngle()) );				
			
						if( desAngle*redblue.getBlobsCurrAngle() >= 0 )
						{
							error[1] = desAngle - redblue.getBlobsCurrAngle();
						}
						else
						{
							if( desAngle < 0 )
							{
								tempAngle = desAngle + PI;
								if( tempAngle-redblue.getBlobsCurrAngle() < 0 )
									error[1] = desAngle + 2*PI - redblue.getBlobsCurrAngle();
								else
									error[1] = desAngle - redblue.getBlobsCurrAngle();
							}
							else if( redblue.getBlobsCurrAngle() < 0 )
							{
								tempAngle = redblue.getBlobsCurrAngle() + PI;
								if( desAngle-tempAngle < 0 )
									error[1] = desAngle - redblue.getBlobsCurrAngle();
								else
									error[1] = - (redblue.getBlobsCurrAngle() + 2*PI - desAngle);
							}
						}
						//----- Stopping condition ------
						if (error[0] > STOP_BAND)
						{
							printf("e0 = %.1f, e1 = %.1f ", error[0], error[1]);	// PIDControl will print the control signal
							iCreate.PIDControl(error[0], error[1]);
							//Sleep(20);
						}
						else
						{
							iCreate.quickStop();
							progState = READY;
						}
					}
					testPt.x = redblue.getBlobsCurrPt().x;
					testPt.y = redblue.getBlobsCurrPt().y;
					testPt.th = redblue.getBlobsCurrAngle();
					testPt.e0 = error[0];
					testPt.e1 = error[1];
					testPt.u = control[0];
					testPt.w = control[1];
					robotPts.push_back(testPt);

					if( motion == TRAJ )
					{
						testPt.x = trajPtr->trajCurrPt.x;
						testPt.y = trajPtr->trajCurrPt.y;
						testPt.th = trajPtr->trajCurrAngle;
						trajPts.push_back(testPt);
					}
				}
				timer++;
				break;

			case BLOB_LOST:
				break;
			case DUMMY:
				break;
			default:
				break;
		}				

		userInput = waitKey(30);
		if( userInput == SPACEBAR ) 
		{
			iCreate.quickStop();	//Emergency stop via spacebar
			progState = READY;
		}
		else if( userInput == 'r')
		{
			progState = READY;
		}
		else if( userInput == ENTER )		//Enter input exits
			break;
	}
	
	//--- Deallocate windows and the video capture ---
	cvDestroyWindow("SceneVideo");
	cvDestroyWindow("Tracking");
	cvReleaseCapture(&capture);

	if( hsvFrame != NULL )
		cvReleaseImage( &hsvFrame );

	return 0;    		
}

// ------------------------------------------------------------
void onMouse(int event, int u, int v, int flags, void *param)
{
	//blob2* blobs = NULL;
	IplImage* firstFrame = NULL;
	IplImage* temp = NULL;

	switch(event)
	{		
		case CV_EVENT_LBUTTONDOWN: //left button down
			switch (progState)
			{
				case INIT:
					progState = RED_CALIB;
					printf("\n\r----- Color Calibration Started -----\n\r");
					//break;
				case RED_CALIB:
					firstFrame = (IplImage*) param;
					temp = cvCloneImage( firstFrame );
					redRect = cvRect( u, v, 0, 0 );
					isDrawing = true;
					break;
				case BLUE_CALIB:
					firstFrame = (IplImage*) param;
					blueRect = cvRect( u, v, 0, 0 );
					isDrawing = true;
					break;
				case READY:
					firstFrame = (IplImage*) param;
					trajEndAngle = cvRect( u, v, 0, 0 );
					isDrawing = true;
					progState = ANGLE_SET;
					break;
				default:
					break;
			}		
			break;

		case CV_EVENT_MOUSEMOVE: //drag mouse to draw rectangle
			switch (progState)
			{
				case RED_CALIB:
					if(isDrawing)
					{					
						firstFrame = (IplImage*) param;
						temp = cvCloneImage( firstFrame );
						redRect.width = u-redRect.x;
						redRect.height = v-redRect.y;					
						cvRectangle( temp, cvPoint(redRect.x, redRect.y), cvPoint(redRect.x+redRect.width,redRect.y+redRect.height), cvScalar(0xff,0xff,0xff) );
						cvShowImage("SceneVideo", temp);
					}
					break;
				case BLUE_CALIB:
					if(isDrawing)
					{
						firstFrame = (IplImage*) param;
						temp = cvCloneImage( firstFrame );
						blueRect.width = u-blueRect.x;
						blueRect.height = v-blueRect.y;					
						cvRectangle( temp, cvPoint(blueRect.x, blueRect.y), cvPoint(blueRect.x+blueRect.width,blueRect.y+blueRect.height), cvScalar(0xff,0xff,0xff) );
						cvShowImage("SceneVideo", temp);
					}
					break;
				case ANGLE_SET:
					if(isDrawing)
					{
						firstFrame = (IplImage*) param;
						temp = cvCloneImage( firstFrame );
						trajEndAngle.width = u-trajEndAngle.x;
						trajEndAngle.height = v-trajEndAngle.y;					
						cvLine( temp, cvPoint(trajEndAngle.x, trajEndAngle.y), cvPoint(trajEndAngle.x+trajEndAngle.width,trajEndAngle.y+trajEndAngle.height), cvScalar(0xff,0xff,0xff) );
						cvShowImage("SceneVideo", temp);
					}
					break;
				default:
					break;
			}		
			break;

		case CV_EVENT_LBUTTONUP: //left button up
			switch (progState)
			{
				case RED_CALIB:
					firstFrame = (IplImage*) param;
					if( redRect.width < 0 )
					{
						redRect.x += redRect.width;
						redRect.width *= -1;
					}
					if( redRect.height < 0 )
					{
						redRect.y += redRect.height;
						redRect.height *= -1;
					}					
					cvRectangle( firstFrame, cvPoint(redRect.x, redRect.y), cvPoint(redRect.x+redRect.width,redRect.y+redRect.height), cvScalar(0xff,0xff,0xff) );
					cvShowImage("SceneVideo", firstFrame);
					progState = BLUE_CALIB;
					isDrawing = false;
					break;
				case BLUE_CALIB:
					firstFrame = (IplImage*) param;
					if( blueRect.width < 0 )
					{
						blueRect.x += blueRect.width;
						blueRect.width *= -1;
					}
					if( blueRect.height < 0 )
					{
						blueRect.y += blueRect.height;
						blueRect.height *= -1;
					}					
					cvRectangle( firstFrame, cvPoint(blueRect.x, blueRect.y), cvPoint(blueRect.x+blueRect.width,blueRect.y+blueRect.height), cvScalar(0xff,0xff,0xff) );
					cvShowImage("SceneVideo", firstFrame);
					isDrawing = false;
					progState = CALIB_DONE;
					break;
				case ANGLE_SET:
					isDrawing = false;
					break;
				case READY:					
										
					break;
				default:
					break;
			}		
			break;

		//case CV_EVENT_LBUTTONDBLCLK:
			
	}

	if( temp != NULL )
	{
		cvReleaseImage( &temp );
	}
}