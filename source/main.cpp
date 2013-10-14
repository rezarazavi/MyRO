/*===========================================================================
  Myro Mobile robot main Program
  build includes and arguments:
  compile: g++ -Wall -c "%f" -I/usr/local/include/opencv -I/usr/include/speech_tools -I/usr/include/festival 
  * -I/usr/local/include/pocketsphinx -I/usr/local/include/sphinxbase
  build  : g++ `pkg-config opencv --cflags` main.cpp -o server ServerSocket.o Socket.o -lFestival -lestools -lestbase -leststring 
  * -I/usr/include/speech_tools -I/usr/include/festival `pkg-config opencv --libs` -I/usr/local/include/pocketsphinx 
  * -I/usr/local/include/sphinxbase -lpocketsphinx -lsphinxbase -lpocketsphinx 
  * /home/e-solutions/src/sphinxbase-0.7/src/libsphinxad/ad_alsa.o 
  * /home/e-solutions/src/sphinxbase-0.7/src/libsphinxad/cont_ad_base.o
  ===========================================================================*/
#include <pthread.h>
#include <termios.h>
#include <unistd.h> 
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <setjmp.h>
#include "cv.h"
#include <opencv/cxcore.h>
#include "highgui.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ServerSocket.h"
#include "SocketException.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <math.h>
#include <sys/socket.h>
//#include <CImg.h>
#include <festival.h>
//#include <sphinx.h>
#include <fe.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cmath>
#include <cassert>
#include <cstddef>
#include <sys/types.h>        // read / write function
#include <pocketsphinx.h>
#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>
#include <sphinxbase/cont_ad.h>
#include <prim_type.h>
#include <sphinx_config.h>
#include <sphinxbase_export.h>
#include <iomanip>
#include <locale>
#include <sstream>

#define PointVal(img,x,y,channel) *(img->imageData + (y)*img->widthStep + (x)*img->nChannels + (channel))

using namespace std;
/*===========================================================================*/
//creating main variables and instractures
static int fd = 0;
char a[2];

int min_border[9] = {0,0,0,0,0,0,0,0,0};                          //0-2 is BGR space, 3-5 is HSV, 6-8 is YCrCb
int max_border[9] = {255,255,255,180,255,255,255,255,255};        //3 is H channel, it is circled

int iIn = 0;
char a1, a2;
int LMS,RMS;//left and right motor speed
int CHKcon;// variable to stop robot while there's no command more than .5 seconds
struct uIO {//uchameleon IO device structure
  int A2D[8];
  int D2A[4];
  int DIO[6];
} ;
uIO MyIo;
string AIMode;
std::string festivalst;// once filled it will be pass to synthisizer
CvCapture* capture;//OpenCV vision operator
cv::Mat img0;//opencv image holder
cv::Mat img1;//opencv reserved image for processing
int ObjX,ObjY;// X and Y position for image proceesing object
bool Imginprocess;// indicates if image processing is happening
 #define FOURCCCODEC CV_FOURCC('H','2','6','4') // H.264 codec
 #define FPS 30                                // Frame rate things run at
 static void sleep_usec(int32 us);
 static void sleep_msec(int32 us);
/*===========================================================================*/
//writnig function for uchameleon IO
int cmdWrite(char cmdStr[])
{
    int er1;
    er1 = write(fd, cmdStr, strlen(cmdStr));
        write(fd, a , 2);
    return(er1);
}
/*===========================================================================*/
// Myro comon 4 htreads variables
void * IoThread(void * str);
void * ReservedThread(void * str);
void * ServerThread(void * str);
void * VisionThread(void * str);
void * VisionThread2(void * str);
void runmotors(int rs,int ls);
bool threadlock,threadlock2;

/*===========================================================================*/
//this thread handles process for Robots IO
void * IoThread(void * str)
{
	printf("IoThread Started \n");
	char devret[256];
    struct timeval tv,ctv;
    fd = open("/dev/ttyACM0", O_RDWR );
    //set pwm period
	cmdWrite("pwm 9 period 1000");
	cmdWrite("pwm 10 period 1000");
	cmdWrite("pwm 11 period 1000");
	cmdWrite("pwm 12 period 1000");	
    gettimeofday(&tv,NULL);
    while(1)
    {
		gettimeofday(&ctv,NULL);
        if ((ctv.tv_usec-tv.tv_usec>100000) || (ctv.tv_sec > tv.tv_sec))
            {
				//if the controler does not recieve any comand it will stop the robot
				if(CHKcon<12)
				{
					CHKcon++;
				}
				if(CHKcon==5)
				{
					cmdWrite( "pin 18 high" );//unlocks motors//stops robot if comunication stops more than 500ms
					runmotors(0,0);
				}
				
				tv=ctv;
				//iIn = read( fd,devret, 256);
				//printf("%s \n",devret);
			}
			usleep(1000);
	}
}
/*===========================================================================*/
//runmotors is the function to control the motors
//rs is right motor speed
//rs should be between -300(Maximum backward speed) and 300 ( Maximum forward speed)
//ls is left motor speed
//ls should be between -300(Maximum backward speed) and 300 ( Maximum forward speed)
void runmotors(int rs,int ls)
{
	if ((rs !=0)||(ls !=0))// checks if both motor speeds are 0 or not
	{
		char sendCMD[50];// command string
		cmdWrite( "pin 18 low" );//unlocks motors
		//clamping speeds up to 300 (30%)
		if( rs>600)
			rs=600;
		if( rs<-600)
			rs=-600;
		if( ls>600)
			ls=600;
		if( ls<-600)
			ls=-600;
			
		if ( rs>0)//setting comand to io for right motor
		{
			cmdWrite( "pin 13 low" );//set right motor turn forward
			sprintf(sendCMD,"pwm 9 width %d",rs);
			cmdWrite(sendCMD);//set right motor speed
		}
		else
		{
			cmdWrite( "pin 13 high" );//set right motor turn backward
			sprintf(sendCMD,"pwm 9 width %d",-1*rs);
			cmdWrite(sendCMD);//set right motor speed
		}
		if (ls>0)//setting comand to io for left motor
		{
			cmdWrite( "pin 14 high" );//set left motor turn forward
			sprintf(sendCMD,"pwm 10 width %d",ls);
			cmdWrite(sendCMD);//set left motor speed
		}
		else
		{
			cmdWrite( "pin 14 low" );//set left motor turn backward
			sprintf(sendCMD,"pwm 10 width %d",-1*ls);
			cmdWrite(sendCMD);//set left motor speed
		}
	}
	else
	{
		cmdWrite( "pin 18 high" );//locks motors
	}
}
/* Sleep for specified usec */
static void
sleep_usec(int32 us)
{
    struct timeval tmo;

    tmo.tv_sec = 0;
    tmo.tv_usec = us;

    select(0, NULL, NULL, NULL, &tmo);
}
/* Sleep for specified msec */
static void
sleep_msec(int32 ms)
{
    struct timeval tmo;

    tmo.tv_sec = 0;
    tmo.tv_usec = ms * 1000;

    select(0, NULL, NULL, NULL, &tmo);
}
/*===========================================================================*/
//this thread handles voice recognition

void * ReservedThread(void * str)
{
	printf("ReservedThread Started \n");
		////voice recognition parameters
	try
	{
		//starting festival speech synthisizer
		festival_initialize(1,420000); 
		festival_say_text("Hello, I am My Ro  , I am ready for your cammands");
	}
	catch ( SocketException& e )
    {
       std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }
	static const arg_t cont_args_def[] = {
		POCKETSPHINX_OPTIONS,
		// Argument file. 
		{ "-argfile",
		  ARG_STRING,
		  NULL,
		  "Argument file giving extra arguments." },
		{ "-adcdev", 
		  ARG_STRING, 
		  NULL, 
		  "Name of audio device to use for input." },
		{ "-infile", 
		  ARG_STRING, 
		  NULL, 
		  "Audio file to transcribe." },
		{ "-time", 
		  ARG_BOOLEAN, 
		  "no", 
		  "Print word times in file transcription." },
		CMDLN_EMPTY_OPTION
	};

	static ps_decoder_t *ps;
	static cmd_ln_t *config;
	static FILE* rawfd;
	ad_rec_t *ad;
    int16 adbuf[4096];
    int32 k, ts, rem;
    char const *hyp;
    char const *uttid;
    cont_ad_t *cont;
    string strrcvd;
    char word[256];
    char *argv[3];
    int rv;
    festivalst="";
    config = cmd_ln_init(NULL, cont_args_def, FALSE,
		"-hmm","/home/e-solutions/src/pocketsphinx-0.7/model/hmm/en_US/hub4wsj_sc_8k",
		"-lm", "/home/e-solutions/src/pocketsphinx-0.7/model/lm/en_US/hub4.5000.DMP",
		"-dict","/home/e-solutions/src/pocketsphinx-0.7/model/lm/en_US/cmu07a.dic",
		"-jsgf","/home/e-solutions/Desktop/Myro/Myro.gram",
		NULL);
	ps = ps_init(config);
	if ((ad = ad_open_dev(cmd_ln_str_r(config, "-adcdev"),
                      (int)cmd_ln_float32_r(config, "-samprate"))) == NULL)
    E_FATAL("Failed top open audio device\n");
        // Initialize continuous listening module 
    if ((cont = cont_ad_init(ad, ad_read)) == NULL)
        E_FATAL("Failed to initialize voice activity detection\n");
    if (ad_start_rec(ad) < 0)
        E_FATAL("Failed to start recording\n");
    if (cont_ad_calib(cont) < 0)
        E_FATAL("Failed to calibrate voice activity detection\n");
    for (;;)
     {
		if (festivalst!="")
			{
				festival_say_text(festivalst.c_str());
				festivalst="";
			}
		if(CHKcon>9)
		{
			try
			{
				// Indicate listening for next utterance 
				printf("READY....\n");
				fflush(stdout);
				fflush(stderr);
				// Wait data for next utterance 
				while (((k = cont_ad_read(cont, adbuf, 4096)) == 0)&&(CHKcon>9))
					sleep_msec(100);

				if (k < 0)
					E_FATAL("Failed to read audio\n");

				//
				 // Non-zero amount of data received; start recognition of new utterance.
				 //NULL argument to uttproc_begin_utt => automatic generation of utterance-id.
				 //
				if (ps_start_utt(ps, NULL) < 0)
					E_FATAL("Failed to start utterance\n");
				ps_process_raw(ps, adbuf, k, FALSE, FALSE);
				printf("Listening...\n");
				fflush(stdout);

				// Note timestamp for this first block of data 
				ts = cont->read_ts;
				// Decode utterance until end (marked by a "long" silence, >1sec) 
				for (;;) 
				{
					// Read non-silence audio data, if any, from continuous listening module 
					if ((k = cont_ad_read(cont, adbuf, 4096)) < 0)
						E_FATAL("Failed to read audio\n");
					if (k == 0) {
						//
						 // No speech data available; check current timestamp with most recent
						 // speech to see if more than 1 sec elapsed.  If so, end of utterance.
						 //
						if ((cont->read_ts - ts) > DEFAULT_SAMPLES_PER_SEC)
							break;
					}
					else {
						// New speech data received; note current timestamp 
						ts=cont->read_ts;
	
					}

					//Decode whatever data was read above.

					try
					{
						rem = ps_process_raw(ps, adbuf, k, FALSE, FALSE);
					}
					catch(int e)
					{
					}

					// If no work to be done, sleep a bit 
					if ((rem == 0) && (k == 0))
						sleep_msec(20);
					if(CHKcon<9)
						break;
				}

				//
				 // Utterance ended; flush any accumulated, unprocessed A/D data and stop
				 // listening until current utterance completely decoded

				ad_stop_rec(ad);
				while (ad_read(ad, adbuf, 4096) >= 0);
				cont_ad_reset(cont);

				printf("Stopped listening, please wait...\n");
				fflush(stdout);
				// Finish decoding, obtain and print result 
				rv = ps_end_utt(ps);
				try
				{
					hyp = ps_get_hyp(ps, NULL, &uttid);
				}
				catch(int e)
				{
				}
				printf("you sayed: (%s) \n",hyp);
				if (hyp)
				{
					strrcvd = hyp;
					if (strrcvd.find("MYRO sleep") !=string::npos)
					{
						festivalst="sorry I'm not sleepy";
					}
					if (strrcvd.find("MYRO where are you from") !=string::npos)
					{
						festivalst="I'm from Malaysia";
					}	
					if(strrcvd.find("MYRO how is the weather") !=string::npos)
					{
						festivalst="Sunny today, haha";
					}
					if (strrcvd.find("MYRO hi") !=string::npos)
					{
						festivalst="Hi, How are you ?";
					}
					if (strrcvd.find("MYRO introduce yourself") !=string::npos)
					{
						festivalst="I am My row, First Malaysia Mobile robot platform ";
					}
					if (strrcvd.find("MYRO where are you") !=string::npos)
					{
						festivalst="I am in utem";
					}
								
					if (strrcvd.find("MYRO move forward") !=string::npos)
					{
						cmdWrite( "pin 18 low" );
						runmotors(200,200);
					}
					if (strrcvd.find("MYRO move backward") !=string::npos)
					{
						cmdWrite( "pin 18 low" );
						runmotors(-200,-200);
					}
					if (strrcvd.find("MYRO turn left") !=string::npos)
					{
						cmdWrite( "pin 18 low" );
						runmotors(200,-200);
					}
					if (strrcvd.find("MYRO turn right") !=string::npos)
					{
						cmdWrite( "pin 18 low" );
						runmotors(-200,200);
					}
					if (strrcvd.find("MYRO stop") !=string::npos)
					{
						cmdWrite( "pin 18 high" );//unlocks motors//stops robot if comunication stops more than 500ms
						runmotors(0,0);
						AIMode="NONE";
					}
					if (strrcvd.find("MYRO what is your name") !=string::npos)
					{
						festivalst="My name is Myro";
					}
					if (strrcvd.find("MYRO follow the hand") !=string::npos)
					{
						AIMode="FINDHAND";
						festivalst="I'm searching for the hand";
					}
					if (strrcvd.find("MYRO find the ball") !=string::npos)
					{
						AIMode="FINDBALL";
						festivalst="I'm searching for the ball	";
					}
					if (strrcvd.find("MYRO how many people you can see") !=string::npos)
					{
						AIMode="NONE";
						Imginprocess=1;
						//image processing example of face detection
						//////////////////////////face detection parameters, comment to disable
						string face_cascade_name = "haarcascade_frontalface_alt.xml";
						string eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
						cv::CascadeClassifier face_cascade;
						cv::CascadeClassifier eyes_cascade;
						cv::RNG rng(12345);
						if( !face_cascade.load( face_cascade_name ) )
						{ 
							printf("--(!)Error loading\n"); 
						}
						if( !eyes_cascade.load( eyes_cascade_name ) )
						{
							 printf("--(!)Error loading\n");
						}
						vector<cv::Rect> faces;
						cv::Mat frame_gray;
						cvtColor( img1, frame_gray, CV_BGR2GRAY );
						equalizeHist( frame_gray, frame_gray );
						  //-- Detect faces
						face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
						if (faces.size()>0)
						{
							ostringstream Convert;
							Convert << faces.size();
							strrcvd = "I can see "+ Convert.str()+" person on my camera";
							festivalst=strrcvd;
						}
						else
						{
							festivalst="I can not see anyone on my camera";
						}
						///////////////////////////end of parametres
						Imginprocess=0;
					}
				}
				fflush(stdout);
						// Resume A/D recording for next utterance 
				if (ad_start_rec(ad) < 0)
					E_FATAL("Failed to start recording\n");
			}
			catch(int e)
			{
			}
			
		}
		else
		{

		}
	}
	cont_ad_close(cont);
    ad_close(ad);
	///////////voice recognition parameters ends
}
/*===========================================================================*/
//this thread handles socket connection to the network
void * ServerThread(void * str)
{
	printf("ServerThread Started \n");
	CHKcon=0;
	// Create the socket
	ServerSocket server ( 30000 );
    while(true)
    {
		ServerSocket new_sock;
		server.accept ( new_sock );//start listening to port 30000
		try
		{
			//cvNamedWindow("thresh");/////////////delet it
			while (true)
			{
						////////////ball tracking
						//IplImage frame0;
						//frame0=img0;
						//IplImage* imgHSV = cvCreateImage(cvGetSize(&frame0),8,3);
						//cvCvtColor(&frame0,imgHSV, CV_BGR2HSV);
						//IplImage* imgThreshed= cvCreateImage(cvGetSize(&frame0),8,1);
						//cvInRangeS(imgHSV,cvScalar(20,100,100),cvScalar(30,255,255),imgThreshed);
						//cvReleaseImage(&imgHSV);
						
						//////////////////////
				std::string data;
				new_sock >> data;
				if(data.length()>2)
					if((data[0]=='*')&&(data[1]=='#')&&(data[data.length()-1]=='*'))
					{
						CHKcon=0;//resets connection check timer
						switch (data[2])
						{
							case 'M'://Motors instruction
								if(data.length()==6)
								{
									//printf("%d %d \n",int(data[3])-128,int(data[4])-128);
									runmotors((int(data[3])-64)*4,(int(data[4])-64)*4);
								}
								break;
							case 'S':
								festivalst= data.substr(3,data.length()-4);
							break;
						}
					}
				new_sock << data;
			 }
		}
		catch ( SocketException& ) {}	
		usleep(1000);
	}
}
/*===========================================================================*/
//this thread handles processes for image processing through code vision
void * VisionThread(void * str)
{
	IplImage frame ;
	printf("VisionThread Started \n");
	int key;
	struct timeval tv,ctv;
	capture= cvCaptureFromFile("rtsp://192.168.1.3/channel1");
	CvMemStorage* storage = cvCreateMemStorage(0); //needed for Hough circles
	Imginprocess=0;
	if (capture != NULL)
	{
		cvNamedWindow ("Myro Cam", CV_WINDOW_AUTOSIZE);
		gettimeofday(&ctv,NULL);
		tv=ctv;
		do
		{
			img1 = cvQueryFrame( capture );
			if( !img1.empty())
			{
				
					/////////face recognition
					gettimeofday(&ctv,NULL);
					frame=img1;
					if ((ctv.tv_usec-tv.tv_usec>40000) || (ctv.tv_sec > tv.tv_sec))
					{
						if (threadlock2)
						{
							threadlock=0;
							resize(img1, img0, cv::Size(320,200), 0, 0, cv::INTER_NEAREST);
							if ( (ObjX !=0)||(ObjY !=0))
							{
								CvPoint center = cvPoint(ObjX,ObjY);
								cvCircle(&frame,  center, 10,CV_RGB(0,255,0), -1, CV_AA, 0);
							}
							threadlock=1;
							threadlock2=0;
						}
						tv=ctv;
						//////////////////////
					}
					cv::imshow("Myro Cam", img1);//displays camera
			}
			cvWaitKey(10);
		}while( !img1.empty() );
		printf("Destroying window \n");
		cvDestroyWindow( "Cam" ); // Destroy the window
		cvReleaseCapture( &capture );
		
	}
	else
	{
		printf("can not access IP Camera \n");
	}
}
void * VisionThread2(void * str)
{
	IplImage frame ;
	CvSize Processsize = cvSize(320,200);
    IplImage *  hsv_frame    = cvCreateImage(Processsize, IPL_DEPTH_8U, 3);
    IplImage*  thresholded    = cvCreateImage(Processsize, IPL_DEPTH_8U, 1);
    
    /*
	CvScalar hsv_min = cvScalar(89, 200, 97, 0);
    CvScalar hsv_max = cvScalar(118, 230 , 122, 0);
    CvScalar hsv_min2 = cvScalar(97, 110, 75, 0);
    CvScalar hsv_max2 = cvScalar(129, 243, 141, 0);blue*/
	CvScalar hsv_min = cvScalar(60, 56, 37, 0);//green
    CvScalar hsv_max = cvScalar(85, 145, 197, 0);
    CvScalar hsv_min2 = cvScalar(68, 60, 35, 0);
    CvScalar hsv_max2 = cvScalar(95, 211, 210, 0);
    CvMemStorage* storage = cvCreateMemStorage(0); //needed for Hough circles
    CvMemStorage* storage1 = cvCreateMemStorage(0); //needed for Hough circles
    CvHaarClassifierCascade* cascade = 0;
    CvHaarClassifierCascade* cascade1 = 0;
    const char* cascade_name = "fist.xml";
	//define the path to cascade file
	string cascadeName = "fist.xml"; /*ROBUST-fist detection haartraining file*/
	const char* cascade_name1 = "palm.xml";
	//define the path to cascade file
	string cascadeName1 = "palm.xml"; /*ROBUST-fist detection haartraining file*/
	cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );
	cascade1 = (CvHaarClassifierCascade*)cvLoad( cascade_name1, 0, 0, 0 );
    int bllcnt=0;
	printf("VisionThread2 Started \n");
	cvNamedWindow ("ImgProcess", CV_WINDOW_AUTOSIZE);
	do
	{
		if(AIMode=="FINDHAND")
		{
			if(threadlock)
			{
				threadlock2=0;
				if (!img0.empty())
				{
					try
					{
						int x=0;
						int y=0;
						frame = img0;
						double scale = 1.1;
						IplImage* temp = cvCreateImage( cvSize(Processsize.width/scale
						,Processsize.height/scale), 8, 3 );
						CvPoint pt1, pt2;
						int i;
						cvClearMemStorage( storage );
						if(cascade)
						{
							CvSeq* fists = cvHaarDetectObjects(
							&frame,
							cascade,
							storage,
							scale, 2, CV_HAAR_DO_CANNY_PRUNING,
							cvSize(24, 24) );
							/*
							CvSeq* palms = cvHaarDetectObjects(
							&frame,
							cascade1,
							storage1,
							scale, 2, CV_HAAR_DO_CANNY_PRUNING,
							cvSize(24, 24) );*/
							
							for( i = 0; i < (fists ? fists->total : 0); i++ )
							{
								CvRect* r = (CvRect*)cvGetSeqElem( fists, i );
								pt1.x = r->x*scale;
								pt2.x = (r->x+r->width)*scale;
								pt1.y = r->y*scale;
								pt2.y = (r->y+r->height)*scale;
								cvRectangle( &frame, pt1, pt2, CV_RGB(200, 0, 0), 1, 8, 0 );
							}
							x= pt1.x*2;
							y=pt1.y*2;
							if (fists->total>0)
							{
									cmdWrite( "pin 18 low" );
									runmotors(70,70);
							}
							else
							{
								cmdWrite( "pin 18 high" );
								runmotors(0,0);
							}
						}

					}
					catch(int i)
					{
					}
					
				}
				threadlock2=1;
				threadlock=0;
			}
			
		}
		if(AIMode=="FINDBALL")
		
			if(threadlock)
			{
				threadlock2=0;
				//finidng pink ball codes
				if (!img0.empty())
				{
					try
					{
						frame = img0;
						// color detection using HSV
						cvCvtColor(&frame, hsv_frame, CV_BGR2HSV);
						// to handle color wrap-around, two halves are detected and combined
						cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
						//cvInRangeS(hsv_frame, hsv_min2, hsv_max2, thresholded2);
						//cvOr(thresholded, thresholded2, thresholded);
						// hough detector works better with some smoothing of the image
						cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 13, 13 );
						CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 100, 35, 2, 100);
						int maxRadius = 0;
						int x = 0;
						int y = 0;
						int bll=0;
						for ( int i =0; i< circles->total; i++)
						{
							float *p = (float*)cvGetSeqElem(circles, i);
							if (cvRound(p[2])>maxRadius)
							{
								maxRadius= p[2];
								x=cvRound(p[0]);
								y=cvRound(p[1]);
								bll++;
							}
						}
						x*=2;
						y*=2;
						ObjX=x;
						ObjY=y;
						if (bll)
						{
							if(maxRadius <27)
							{
								printf("ball @ %d , %d \n",x,y);
								if((x>310)&&(x<330))
								{
									cmdWrite( "pin 18 low" );
									runmotors(70,70);
									
								}
								else
								{
									if ((x<=310)&&(x>0))
									{
										cmdWrite( "pin 18 low" );
										runmotors(100+((310-x)*70)/310,100-((310-x)*70)/310);
									}
									if (x>=330)
									{
										cmdWrite( "pin 18 low" );
										runmotors(100-((x-330)*70)/310,100+((x-330)*70)/310);

									}
								}
							}
							else
							{
								cmdWrite( "pin 18 high" );
								runmotors(0,0);
								//AIMode="NONE";
								//festivalst= "the ball is here";
								//CHKcon=0;
							}
							//bllcnt =0;
						}
						else
						{
							//if (bllcnt<5)
							//{
								//bllcnt++;
							//}
							//else
							//{
								cmdWrite( "pin 18 low" );
								runmotors(70,-70);
							//}
						}
						//printf("ball @ %d , %d \n",x,y);
						cvShowImage("ImgProcess", thresholded);
					}
					catch(int i)
					{
					}
				}
				threadlock2=1;
				threadlock=0;
			}
			usleep(10);
	}while (true);
	
}
/*===========================================================================*/
//Threads structure///
typedef struct my_struct 
{
	int data;
	int random_number;
}my_struct_t;
/*===========================================================================*/

/*===========================================================================*/
//  main program to to do initializations and start threads
int main(int argc, char **argv){
	//*******************************************************************
	//Creating Multithread program parametres
	//*******************************************************************
	sigset_t oSignalSet;
	sigemptyset(&oSignalSet);
	sigaddset(&oSignalSet, SIGINT);
	sigaddset(&oSignalSet, SIGABRT);
	sigaddset(&oSignalSet, SIGQUIT);
	pthread_sigmask(SIG_BLOCK, &oSignalSet, NULL);	
	pthread_t iThreadId;
	my_struct_t t1;
	my_struct_t t2;
	my_struct_t t3;
	my_struct_t t4;
	my_struct_t t5;
	threadlock=0;
	threadlock2=1;
	AIMode="NONE";
	//*******************************************************************
	//Starting threads
	//*******************************************************************
	printf("Parent: Creating and calling threads...\n");
	int iReturnValue1 = pthread_create(&iThreadId, NULL, &IoThread, (void *)&t1);
	int iReturnValue2 = pthread_create(&iThreadId, NULL, &ReservedThread, (void *)&t2);
	int iReturnValue3 = pthread_create(&iThreadId, NULL, &ServerThread, (void *)&t3);
	int iReturnValue4 = pthread_create(&iThreadId, NULL, &VisionThread, (void *)&t4);
	int iReturnValue5 = pthread_create(&iThreadId, NULL, &VisionThread2, (void *)&t5);
	
	if (iReturnValue1 < 0||iReturnValue2 < 0||iReturnValue3 < 0 || iReturnValue4 < 0||iReturnValue5 < 0) {
	  printf("error creating thread '%s'.\n", strerror(errno));
	}
	int iSignalNumber;
	sigwait(&oSignalSet, &iSignalNumber);
	cmdWrite( "reset" );	
	printf("exit after receiving signal #%d.\n", iSignalNumber);
	exit(0);
}
/*===========================================================================*/
