#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include "mjpg_streamer.h"

#include "LinuxDARwIn.h"
#include "StatusCheck.h"


#include "Follower.h"

#include "GenericVision.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH      "config_forward.ini"
#define INI_FILE_PATH2      "config_backward.ini"
#define INI_FILE_PATH3     "config_left.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

#define SCRIPT_FILE_PATH    "script.asc"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

using namespace std;
using namespace cv;

int button = 1;
bool pressed = false;
bool ball_found; 
bool stage1 = 1; 
bool stage2 = 0; 
bool stage3 = 0; 
bool stage4 = 0;
bool stage5 = 0;
bool stage6 = 0;
Point3i marker1;
Point3i marker2;
Point3i marker3;

bool DEBUG = false; 

Image *rgb_output; 
minIni* ini_f;
minIni* ini_b;
minIni* ini_l;

Point2D* object;

GenericVision vision;
Mat hsvFrame; 

BallTracker tracker; 
Follower follower; 

///////////////////////////////////////////////////////
///////////////// Functions ///////////////////////////
///////////////////////////////////////////////////////
void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}
void sighandler(int sig)
{
    exit(0);
}

void adjustWalkingLeft();
void adjustWalkingRight();

// Walking gait parameters that could be change in real-time 
void adjustWalk()
{
    Walking::GetInstance()->X_OFFSET = 5.0;
    Walking::GetInstance()->Y_OFFSET = 5.0;
    Walking::GetInstance()->Z_OFFSET = 50.0;
    Walking::GetInstance()->R_OFFSET = 0;
    Walking::GetInstance()->P_OFFSET = 0;
    Walking::GetInstance()->A_OFFSET = 0;
    Walking::GetInstance()->HIP_PITCH_OFFSET = 7.8;
    Walking::GetInstance()->PERIOD_TIME = 650.0;
    Walking::GetInstance()->DSP_RATIO = 0.1;
    Walking::GetInstance()->STEP_FB_RATIO = 0.28;
    Walking::GetInstance()->Z_MOVE_AMPLITUDE = 40.0;
    Walking::GetInstance()->Y_SWAP_AMPLITUDE = 20.0;
    Walking::GetInstance()->Z_SWAP_AMPLITUDE = 5;
    Walking::GetInstance()->PELVIS_OFFSET = 3.0;
    Walking::GetInstance()->ARM_SWING_GAIN = 1.5;
    Walking::GetInstance()->BALANCE_KNEE_GAIN = 0.3;
    Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN = 0.9;
    Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN = 0.5;
    Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN = 1.0;
    Walking::GetInstance()->P_GAIN = JointData::P_GAIN_DEFAULT;
    Walking::GetInstance()->I_GAIN = JointData::I_GAIN_DEFAULT;
    Walking::GetInstance()->D_GAIN = JointData::D_GAIN_DEFAULT;
    
    //Head::GetInstance()->MoveByAngle(0, 0); //move head by angle pan and tilt
}

int main(void) 
{

////////////////////  Initialize Framework////////////////////////////
	bool ball_found; 
	//bool recalibrate = 1;  

	vision.initGUI();

	follower = Follower(); 

	int userInput = 0; 

    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    /// Walking gait files		
    ini_f = new minIni(INI_FILE_PATH); //forward
    ini_b = new minIni(INI_FILE_PATH2); //backward
    ini_l = new minIni(INI_FILE_PATH3); //left

    /// Initialize the camera
    rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default camera setting

    
    /// Initialize the microcontroller cm730	
    if (MotionManager::GetInstance()->Initialize(&cm730) == false) {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if (MotionManager::GetInstance()->Initialize(&cm730) == false) {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    /// Initialize motion manager
    MotionManager::GetInstance()->AddModule((MotionModule *) Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule *) Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule *) Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    /// Set init motion
    int firm_ver = 0;
    if (cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0) != CM730::SUCCESS) {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }
    Action::GetInstance()->LoadFile((char *) MOTION_FILE_PATH); ///load mation file
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);
    
    Action::GetInstance()->Start(1); /// start page 1 in mation file
    while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// END OF INITIALIZING ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// STATUS BUTTON LOOP ////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool isWalkingLeft = true; // a boolean flag that sets the default sidestepping path. 

	int numFrames = 0;
	int consecNumObstaclesInFrame = 0;
	int consecNumYellowObstaclesInFrame = 0;
	bool isLookingRight = false;
	bool isLookingLeft = false;
	int numFramesAfterTurningHead = 0;
	int obstacleTooCloseCount = 0; // We need to back up a bit when the obstacle it located too close.
	while(1) {
		StatusCheck::Check(cm730); ///check microcontroller status
		LinuxCamera::GetInstance()->CaptureFrame(); /// streaming images from camera using guv driver
		memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData,
		LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
    
		/// Convert images to OpenCV Mat
		vision.rawFrame = cv::Mat(rgb_output->m_Height, rgb_output->m_Width, CV_8UC3, rgb_output->m_ImageData); 

		hsvFrame;
		/// PREPROCESSING THE BINARY THRESHOLD
		resize(vision.rawFrame, vision.rawFrame, Size(320, 240), 0, 0, 0); /// reduce the frame size for faster processing
		vision.flushEdges(vision.rawFrame, .01);
		cvtColor(vision.rawFrame, hsvFrame, CV_BGR2HSV); /// convert RGB to HSV for robust thresholding

		/// PREPROCESSING THE THRESHOLD: Thresholding for upto 3 colour (HSV) spaces 
		Vec3i avgHSV1, avgHSV2, avgHSV3, gainHSV1, gainHSV2, gainHSV3;
		vision.readHSVAvg("marker1.xml", avgHSV1, gainHSV1);
		vision.readHSVAvg("marker2.xml", avgHSV2, gainHSV2);
		vision.readHSVAvg("marker3.xml", avgHSV3, gainHSV3);
		
		inRange(hsvFrame, avgHSV1 - gainHSV1, avgHSV1 + gainHSV1, vision.threshold1Frame);
		inRange(hsvFrame, avgHSV2 - gainHSV2, avgHSV2 + gainHSV2, vision.threshold2Frame);
		inRange(hsvFrame, avgHSV3 - gainHSV3, avgHSV3 + gainHSV3, vision.threshold3Frame); 
			
		/// Main function call for detection
		marker1 = vision.findColouredObject(vision.rawFrame, vision.threshold1Frame, Scalar(255, 0, 0), 50, 40000, 2, 7, 0.05);																		
		marker2 = vision.findColouredObject(vision.rawFrame, vision.threshold2Frame, Scalar(0, 255, 255), 50, 40000, 2, 7, 0.05);																					
		//marker3 = vision.findColouredObject(vision.rawFrame, vision.threshold3Frame, Scalar(0, 255, 0), 100, 40000, 4, 7, 0.1);
		//char strMarker1[50];
		//sprintf(strMarker1, "marker1: %d", marker1.z);
		//putText(vision.rawFrame, strMarker1, Point2f(50,50), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255));
		double center = vision.rawFrame.size().width/2;
		double right1 = center +5;
		double left1 = center -5;
		marker1 = vision.findColouredObject(vision.rawFrame, vision.threshold1Frame, Scalar(255, 0, 0), 50, 40000, 2, 7, 0.05);																		
		marker2 = vision.findColouredObject(vision.rawFrame, vision.threshold2Frame, Scalar(0, 255, 255), 50, 40000, 2, 7, 0.05);				
		///set the callback function for any mouse event
		setMouseCallback("Control", vision.CallBackFunc, &hsvFrame);
		
		vision.showGUI(); 
		
		if( isLookingRight || isLookingLeft ){
			numFramesAfterTurningHead++;
			if(numFramesAfterTurningHead < 10){
				stage3 = false;
			}else{
				stage3 = true;
				numFramesAfterTurningHead = 0;
			}
		}
		
		if(waitKey(30) == 27){inRange(hsvFrame, avgHSV1 - gainHSV1, avgHSV1 + gainHSV1, vision.threshold1Frame);
			destroyAllWindows();
			Walking::GetInstance()->Stop(); 
			break;
		}
		
	
	//////////////////////////////////////EVENT CODE////////////////////////////////////////////////

		if( StatusCheck::m_cur_mode == START )
		{
			while (button)
			{           
 
				button = 0;

				/// Load in Page number that is same as walking Stance
				/// To avoid sudden jerks after tuning walk.

				Action::GetInstance()->Start(9); /// Basketball Walk Ready Page
				while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);

				/// Re-Initialize Head / Walking before able to start walk
				Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
				Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
				MotionManager::GetInstance()->SetEnable(true);
				Head::GetInstance()->MoveByAngle(0, -60); 
				Head::GetInstance()->m_LeftLimit = 70;
				Head::GetInstance()->m_RightLimit = -70; 
				Head::GetInstance()->m_TopLimit = 30; 
				Head::GetInstance()->m_BottomLimit = -10;
            inRange(hsvFrame, avgHSV1 - gainHSV1, avgHSV1 + gainHSV1, vision.threshold1Frame);
				cout << "Initializing body complete" << endl;
			}
			/// Start button pressed
			if( StatusCheck::m_old_btn == 2 ){
				cout << "START PROGRAM" << endl;
				pressed = true;
				usleep(8 * 1000);
			}
	
			object = new Point2D(marker1.x , marker1.y);
			
			//If started 
			if( pressed  )
			{
				Walking::GetInstance()->BALANCE_ENABLE = true;
				
                //Enter stage 1 = WALK FORWARD
				if(stage1){			
					cout << "Walk forward until it detects an object" << endl;		
					//Load forward gait
					Walking::GetInstance()->LoadINISettings(ini_f);
					MotionManager::GetInstance()->LoadINISettings(ini_f);

					bool isBlueObstacle = (marker1.x > 30 && marker1.x < 310 && marker1.y > 20 ) ? true : false;
					bool isYellowObstacle = (marker2.x > 20 && marker2.x < 320 && marker2.y > 50 )? true : false;
					if(!isBlueObstacle && !isYellowObstacle){
						follower.setReverse(false);
						Walking::GetInstance()->X_MOVE_AMPLITUDE = 6;
						Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
						Walking::GetInstance()->PERIOD_TIME = 510; 
						Walking::GetInstance()->Start();
					}//found
					else { // If there is any blue obstacle, we go to stage 2 to scan around.
						cout << "An Obstacle is Found" << endl;
						stage1 = false;
						stage2 = true;
					}
				}//stage1
				
				//Stage 2 = STOP AND TURN THE HEAD LEFT/RIGHT depending on isWalkingLeft boolean flag.
				else if(stage2){
					if(DEBUG) cout << "STAGE 2" << endl; 
					//Set a timer for 2 seconds
					Walking::GetInstance()->Stop();
					Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					stage2 = false;
					stage3 = true;
					if( isWalkingLeft ){
						Head::GetInstance()->MoveByAngle(40, 30); 
						isLookingRight = false;
					} else {
						Head::GetInstance()->MoveByAngle(40, 30); 
						isLookingLeft = false;
					}
					usleep(200000);
				}
				
				// 
				else if(stage3) {	
					cout << "isWalkingLeft boolean flag is " << isWalkingLeft << endl;
					Walking::GetInstance()->LoadINISettings(ini_l); // Load in the configuration for walking left.
					MotionManager::GetInstance()->LoadINISettings(ini_l);
					int stepStrategy;
					if( isWalkingLeft ){
						if( !isLookingRight ){ // If looking at the left side (by default)
							stepStrategy = vision.findNumColouredPixels("BottomLeftScan", isWalkingLeft); // we scan left side to see if we can walk left or not
							if( stepStrategy == 0 ){ 
								Walking::GetInstance()->Stop();
								Head::GetInstance()->MoveByAngle(0, -60); 
							}	
							else if( stepStrategy == 1 ){ // start walking left, and turn the stage to 4.(walking until no obstacle)
								isLookingRight = false;
								stage3 = false;
								stage4 = true;
								adjustWalkingLeft();
							} 
						}
						
						bool temp = false;
						if( isLookingRight ){ // Head is turned right.
							stepStrategy = vision.findNumColouredPixels("BottomRightScan", isWalkingLeft);
							cout << "Step(isWalkingLeft): " << stepStrategy << endl;	
							// If there is a way to move on the right side == 2, then we move right. Otherwise, we move left.
							if( stepStrategy == 2 ){ // We start walking right and turn to the stage 4.
								stage3 = false;
								stage4 = true;
								temp = true;		
								adjustWalkingRight();
								isLookingRight = false;
							}
							else if( stepStrategy == 0 ) { // if there is no free space on the right side, just start walking left side first for trying.
								stage3 = false;
								stage4 = true;
								adjustWalkingLeft();
								isLookingRight = false;
								temp = true;
							}
						}
						

						// If stepStrategy == -2, then we turn the head to the right.
						if( stepStrategy == -2 && !isLookingRight && !temp) {
							isLookingRight = true;
							Head::GetInstance()->MoveByAngle(-40, 30); 
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
							usleep( 200000 ); 
						}
					} 
					else {
						if( !isLookingLeft ){ // If looking at the right side (by default)
							stepStrategy = vision.findNumColouredPixels("BottomRightScan", isWalkingLeft); // we scan left side to see if we can walk left or not
							if( stepStrategy == 0 ){ 
								Walking::GetInstance()->Stop();
								Head::GetInstance()->MoveByAngle(0, -60); 
							}	
							else if( stepStrategy == 2 ){ // start walking right, and turn the stage to 4.(walking until no obstacle)
								isLookingRight = false;
								stage3 = false;
								stage6 = true;
								adjustWalkingRight();
							} 
						}
						
						bool temp = false;
						if( isLookingLeft ){ // Head is turned right.
							stepStrategy = vision.findNumColouredPixels("BottomLeftScan", isWalkingLeft);
							cout << "Step(isWalkingRight): " << stepStrategy << endl;	
							// If there is a way to move on the right side == 2, then we move right. Otherwise, we move left.
							if( stepStrategy == 1 ){ // We start walking left and turn to the stage 4 to keep walking
								stage3 = false;
								stage6 = true;
								temp = true;		
								adjustWalkingLeft();
								isLookingLeft = false;
							}
							else if( stepStrategy == 0 ) { // if there is no free space on the left side, just start walking right side first for trying.
								stage3 = false;
								stage6 = true;
								adjustWalkingRight();
								isLookingLeft = false;
								temp = true;
							}
						}
						

						// If stepStrategy == -1, then we turn the head to the left.
						if( stepStrategy == -1 && !isLookingLeft && !temp) {
							isLookingLeft = true;
							Head::GetInstance()->MoveByAngle(-40, 30); 
							Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
							usleep( 200000 ); 
						}
					}
					
				} 
				// STAGE 4: decides when the robot stops whilst walking left
				else if( stage4 ) { 
					isWalkingLeft = true;
					cout << "Stage4: Walking left until there is a free space to walk forward again" << endl;
					bool isBlueObstacle = (marker1.x > 30 && marker1.x < 300 && marker1.y > 30 ) ? true : false;
					bool isYellowObstacle = (marker2.x > 0 && marker2.x < 340 && marker2.y > 0 ) ? true : false;
					int stepStrategy = vision.findNumColouredPixels("BottomForwardScan", isWalkingLeft);
					
					// If the obstacles are detected too close (blue), then we back up a little.
					if( stepStrategy == 3)
						obstacleTooCloseCount++;
						
					// If blue obstacles are not detected for a few frames, that means we don't have a blue obstacle in front anymore
					if(!isBlueObstacle ) 
						consecNumObstaclesInFrame++;
					else 
						consecNumObstaclesInFrame = 0;
						
					// If there exists a yellow obstacle for a few frames, that means this is a dead end, and we should try going right.
					if(isYellowObstacle)
						consecNumYellowObstaclesInFrame++;
					else
						consecNumYellowObstaclesInFrame = 0;
					
					/*if( obstacleTooCloseCount > 3 ) {
						Walking::GetInstance()->Stop();
						obstacleTooCloseCount = 0;
						stage4 = false;
						stage5 = true;
					}*/
					// Whilst walking left, if yellow obstacles are detected, then it is a dead end, we try walking right.
					if( consecNumYellowObstaclesInFrame > 2 ){
						cout << "Yellow colour is detected whilst walking left to avoid the blue obstacles in front" << endl;
						Walking::GetInstance()->Stop();
						consecNumYellowObstaclesInFrame = 0;
						consecNumObstaclesInFrame = 0;
						obstacleTooCloseCount = 0;
						stage4 = false;
						stage6 = true;
						adjustWalkingRight();
						Head::GetInstance()->MoveByAngle(0, -60);
					}
					else if( consecNumObstaclesInFrame > 8 ){
						Walking::GetInstance()->Stop();
						consecNumObstaclesInFrame = 0;
						consecNumYellowObstaclesInFrame = 0;
						obstacleTooCloseCount = 0;
						stage1 = true;
						stage4 = false;
					}
					//found
				}
				else if( stage5 ) {
					cout << "Stage5: walking backward because obstacle was detected too close" << endl;
					bool isBlueObstacle = (marker1.x > 30 && marker1.x < 300 && marker1.y > 30 ) ? true : false;
					bool isYellowObstacle = (marker2.x > 30 && marker2.x < 300 && marker2.y > 80 ) ? true : false;
					Walking::GetInstance()->LoadINISettings(ini_b); // Load in the configuration for walking left.
					MotionManager::GetInstance()->LoadINISettings(ini_b);
					Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
					Walking::GetInstance()->X_MOVE_AMPLITUDE = -6;
					Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					Walking::GetInstance()->Start();
					if( !isBlueObstacle ) 
						consecNumObstaclesInFrame++;
					else
						consecNumObstaclesInFrame = 0;
					if(!isYellowObstacle)
						consecNumYellowObstaclesInFrame++;
					else
						consecNumYellowObstaclesInFrame = 0;
						
					if( consecNumObstaclesInFrame > 5 && consecNumYellowObstaclesInFrame > 5 ){
						Walking::GetInstance()->Stop();
						consecNumObstaclesInFrame = 0;
						consecNumYellowObstaclesInFrame = 0;
						stage1 = true;
						stage5 = false;
					}
				}
				else if( stage6 ) {
					isWalkingLeft = false;
					cout << "Walking Right after detecting a yellow obstacle on at the left end" << endl;
					bool isBlueObstacle = (marker1.x > 30 && marker1.x < 300 && marker1.y > 30 ) ? true : false;
					bool isYellowObstacle = (marker2.x > 20 && marker2.x < 320 && marker2.y > 80 ) ? true : false;
					int stepStrategy = vision.findNumColouredPixels("BottomForwardScan", isWalkingLeft);
					
					// If the obstacles are detected too close (blue), then we back up a little.
					if( stepStrategy == 3)
						obstacleTooCloseCount++;
					if(!isBlueObstacle ) 
						consecNumObstaclesInFrame++;
					else 
						consecNumObstaclesInFrame = 0;
					if(isYellowObstacle)
						consecNumYellowObstaclesInFrame++;
					else
						consecNumYellowObstaclesInFrame = 0;
					
					/*if( obstacleTooCloseCount > 3 ) {
						obstacleTooCloseCount = 0;
						stage6 = false;
						stage5 = true;
					}*/
					// Whilst walking right, if yellow obstacles are detected, then it is a dead end, TODO!!.
					/*if( consecNumYellowObstaclesInFrame > 6 ){
						Walking::GetInstance()->Stop();
						consecNumYellowObstaclesInFrame = 0;
						stage2 = true;
						stage6 = false;
					}*/
					if( consecNumObstaclesInFrame > 8 ){
						Walking::GetInstance()->Stop();
						consecNumObstaclesInFrame = 0;
						stage1 = true;
						stage6 = false;
					}
				}
			}//pressed
		}
  }//end while Status Check for buttons
}//end main


void adjustWalkingLeft(){
	Walking::GetInstance()->LoadINISettings(ini_l); // Load in the configuration for walking left.
	MotionManager::GetInstance()->LoadINISettings(ini_l);
	Walking::GetInstance()->BALANCE_ENABLE = true;
	Head::GetInstance()->MoveByAngle(0, -60);
	Walking::GetInstance()->HIP_PITCH_OFFSET = 13.3;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -3;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 4;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 20;
	Walking::GetInstance()->Start();
}


void adjustWalkingRight(){
	Walking::GetInstance()->LoadINISettings(ini_l); // Load in the configuration for walking left.
	MotionManager::GetInstance()->LoadINISettings(ini_l);
	Walking::GetInstance()->BALANCE_ENABLE = true;
	Head::GetInstance()->MoveByAngle(0, -60);
	Walking::GetInstance()->HIP_PITCH_OFFSET = 14.3;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = -3;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = -4;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = -20;
	Walking::GetInstance()->Start();
}
