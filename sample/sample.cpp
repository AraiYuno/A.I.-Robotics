#include "opencv2/opencv.hpp"
#include <opencv2/line_descriptor.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;
using namespace cv::line_descriptor;

int value_h_Low = 0; int value_h_High = 18;

int value_s_Low = 101; int value_s_High = 255;

int value_v_Low = 160; int value_v_High = 255;

void updateCanny_Low(int, void* ){}

void updateCanny_High(int, void* ){}
int trackBall();
void drawField();

/** @function main */
int main( int argc, char** argv )
{
	drawField();
	//trackBall();
	return 0;
}


int trackBall(){
	Mat mat_frame, canny, gray,cameraFeed;
	VideoCapture cap(0); // start the camera id:1

    if(!cap.isOpened())  // check if the camera starts 
        return -1;

	/// GUI with trackbar
	namedWindow( "Control", CV_WINDOW_AUTOSIZE );
	createTrackbar( "h (Low):", "Control", &value_h_Low, 255, updateCanny_Low);
	createTrackbar( "h (High):", "Control", &value_h_High, 255, updateCanny_High);
	createTrackbar( "s (Low):", "Control", &value_s_Low, 255, updateCanny_Low);
	createTrackbar( "s (High):", "Control", &value_s_High, 255, updateCanny_High);
	createTrackbar( "v (Low):", "Control", &value_v_Low, 255, updateCanny_Low);
	createTrackbar( "v (High):", "Control", &value_v_High, 255, updateCanny_High);
	
	while (1) {
		cap.read(cameraFeed);

		Mat bgr_image, bgr_blur, hsv_image;
		//bgr_image = imread("ball1.jpg", CV_LOAD_IMAGE_COLOR);
		bgr_image = cameraFeed.clone();
		resize(bgr_image, bgr_image, Size(300,300),0,0,CV_INTER_LINEAR);
		
		
		medianBlur(bgr_image, bgr_blur, 5);

		cvtColor(bgr_blur, hsv_image, cv::COLOR_BGR2HSV);
		
		
		Mat hsv_threshold;
		inRange(hsv_image, cv::Scalar(value_h_Low, value_s_Low, value_v_Low), cv::Scalar(value_h_High, value_s_High, value_v_High), hsv_threshold);
		
		dilate(hsv_threshold, hsv_threshold, Mat(), Point(-1, -1), 4);
		erode(hsv_threshold, hsv_threshold, Mat(), Point(-1, -1), 4);

		
		vector<cv::Vec3f> circles;
		size_t pos = -1;
		float maxRadius = 0;
		
		vector<vector<Point> > contours;
		
		imshow("HSV",hsv_threshold);
		findContours(hsv_threshold, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		
		vector<vector<cv::Point> > balls;
		vector<cv::Rect> ballsBox;
		for (size_t i = 0; i < contours.size(); i++)
		{
			cv::Rect bBox;
			bBox = cv::boundingRect(contours[i]);

			float ratio = (float)bBox.width / (float)bBox.height;
			if (ratio > 1.0f)
				ratio = 1.0f / ratio;

			// Searching for a bBox almost square
			if (ratio > 0.75 && bBox.area() >= 400)
			{
				balls.push_back(contours[i]);
				ballsBox.push_back(bBox);
			}
		}
		vector<vector<Point> > cir(balls.size());
		vector<Point2f> center(balls.size());
		vector<float> radius(balls.size());
		for (int i = 0; i< balls.size(); i++)
		{
			cout << "contours " << balls[i].size() << endl;
			approxPolyDP(Mat(balls[i]), cir[i], 3, true);
			minEnclosingCircle(cir[i], center[i], radius[i]);	
			if ((cir[i]).size() > 7 && radius[i] > maxRadius)
			{
				maxRadius = radius[i];
				pos = i;
			}
		}	

		if (pos != -1)
		{
			Point cent(round(center[pos].x), round(center[pos].y));
			float rad = round(radius[pos]);
			circle(bgr_image, cent, rad, cv::Scalar(0, 255, 0), 2);
		}	

		imshow("Control", bgr_image);
		waitKey(30);
	}
}


void drawField(){
	Mat mat_frame, canny, gray,cameraFeed;
	VideoCapture cap(0); // start the camera id:1

    if(!cap.isOpened())  // check if the camera starts 
        return;

	/// GUI with trackbar
	namedWindow( "LSD", CV_WINDOW_AUTOSIZE );


	while(1){
		cap.read(cameraFeed);
		Mat bgr_image, bgr_blur, hsv_image, mask;
		//bgr_image = imread("ball1.jpg", CV_LOAD_IMAGE_COLOR);
		// create the GUI window (web cam)
		bgr_image = cameraFeed.clone();
		resize(bgr_image, bgr_image, Size(700,700),0,0,CV_INTER_LINEAR);
				
		// create a HSV version of image from bgr image
		medianBlur(bgr_image, bgr_blur, 5);
		cvtColor(bgr_blur, hsv_image, cv::COLOR_BGR2HSV);

		Mat hsv_threshold;
		inRange(hsv_image, cv::Scalar(0, 0, 212), cv::Scalar(131, 255, 255), hsv_threshold);
		imshow("HSV",hsv_threshold);

		mask = Mat::ones( hsv_threshold.size(), CV_8UC1 );
		/* create a pointer to an LSDDetector object */
		Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();
		/* compute lines */
		std::vector<KeyLine> keylines;
		lsd->detect( hsv_threshold, keylines, 1, 1, mask );
	
		/* draw lines extracted from octave 0 */
		cv::Mat output = hsv_threshold.clone();
		if( output.channels() == 1 )
			cvtColor( output, output, COLOR_GRAY2BGR );
		
		for ( size_t i = 0; i < keylines.size(); i++ )
		{
			KeyLine kl = keylines[i];
			if( kl.octave == 0)
			{
				/* get a random color */
				int R = ( rand() % (int) ( 255 + 1 ) );
				int G = ( rand() % (int) ( 255 + 1 ) );
				int B = ( rand() % (int) ( 255 + 1 ) );

				/* get extremes of line */
				Point pt1 = Point( kl.startPointX, kl.startPointY );
				Point pt2 = Point( kl.endPointX, kl.endPointY );

				/* draw line */
				if(  norm(pt1 - pt2) > 17 )
					line( output, pt1, pt2, Scalar( R, G, B ), 5 );
			}

		}
		imshow("LSD", output);
		waitKey(30);
	}
}