#include "opencv2/opencv.hpp"
#include <opencv2/line_descriptor.hpp>
#include <iostream>
#include <math.h>

#define PI 3.14159265358979323846

using namespace std;
using namespace cv;
using namespace cv::line_descriptor;

Mat bgr_image, original;

int ball_h_Low = 0; int ball_h_High = 11; int ball_s_Low = 140; int ball_s_High = 255;
int ball_v_Low = 163; int ball_v_High = 255;

int field_h_Low = 60; int field_h_High = 105; int field_s_Low = 75; int field_s_High = 189;
int field_v_Low = 70; int field_v_High = 200;

int line_h_Low = 193; int line_h_High = 255; int line_s_Low = 182; int line_s_High = 255;
int line_v_Low = 157; int line_v_High = 255;

RNG rng(12345);


/* Helper functions */

void updateCanny_Low(int, void* ){}
void updateCanny_High(int, void* ){}

Point3i trackBall(Mat &img);
void extractField(Mat &img, Mat &field);
void fieldHandler(int event, int x, int y, int flags, void* param);
void ballHandler(int event, int x, int y, int flags, void* param);
void drawField(Mat &img);
void cleanUpLines( vector<KeyLine> &keyLines, vector<KeyLine> &mergedLines);
void detectTCorners(vector<KeyLine> &mergedLines, vector<KeyLine> &cornerLines, vector<KeyLine> &normalLines);

float calcDistance(KeyLine *kl1, KeyLine *kl2);
KeyLine checkTIntersection(KeyLine kl1, KeyLine kl2, Point intersectionPoint);
float calcAngle(KeyLine &kl);
bool areSameLines(KeyLine &kl1, KeyLine &kl2);
void drawLines( Mat &output, vector<KeyLine> &keyLines, int colour);
void sortKeyLines(vector<KeyLine> &keyLines);
void mergeLines(KeyLine &mainLine, KeyLine &kl);
bool mergeParalellLine(KeyLine &kl1, KeyLine &kl2);
void switchStartAndEnd(KeyLine &kl);

/* Intersection */
void detectCircleTCorner(vector<KeyLine> &mergedLines, vector<KeyLine> &cornerLines);
void detectLCorners(vector<KeyLine> lines, vector<KeyLine> &cornerLines);
bool isLCorner(KeyLine &kl1, KeyLine &k12);
bool isTCorner(KeyLine &kl1, KeyLine &kl2);
bool doIntersect(Point p1, Point q1, Point p2, Point q2);
int orientation(Point p, Point q, Point r);
bool onSegment(Point p, Point q, Point r);
Point getIntersectionPoint(KeyLine &kl1, KeyLine &kl2 );



/** @function main */
int main( int argc, char** argv )
{
	Mat mat_frame, canny, gray,cameraFeed;
	

	int top, left, right, bottom;
	
	//changr the parameter to 0 if using induilt camera or 1 for external camera
	VideoCapture cap(1); 

    if(!cap.isOpened())  // check if the camera starts 
         return -1;
	
	while (1) {

		//read the frame and store it in cameraFeed
		cap.read(cameraFeed);

		//bgr_blur for storing the blur image and hsv_image for storing the hsc image
		Mat bgr_blur, hsv_image;

		//bgr_image is a global image
		bgr_image = cameraFeed.clone();

		//resizing the image to fit in out laptop screen ;)
		resize(bgr_image, bgr_image, Size(300,300), 0,0,1);

		//original is a global image, which has the camerafeed as it is i.e no changes made at this to the image
		original = bgr_image.clone();

		medianBlur(bgr_image, bgr_blur, 5);
		cvtColor(bgr_blur, bgr_image, cv::COLOR_BGR2HSV);


		//clickable images for getting the hsv values of the field and ball
		imshow("Field Finder", bgr_image);
		setMouseCallback("Field Finder",fieldHandler, 0 );
		imshow("Ball Finder", bgr_image);
		setMouseCallback("Ball Finder", ballHandler, 0 );
		
		//get the ball location and the size of the ball
		//x, y is the coordinates and z is the radius
		// if there is no ball x = y = -1
		Point3i ball = trackBall(bgr_image);

		//if there is a ball draw the circle
		if(ball.x>0)
			circle(original, Point(ball.x, ball.y), ball.z, Scalar(255,0,0), 2);

		Mat field;

		//extract the field and store the field in the variable field
		extractField(bgr_image, field);
		imshow("Field Control",field);

		//draw the lines
		drawField(field);
		
		waitKey(30);
	}
return 0;
}

/*
	fieldHandler
		-get the hsv value of the pixel

		-when clicked on the field finder window this function is evoked
		-get the hsv value and assign it the global values of the field hsv values 
*/
void fieldHandler(int event, int x, int y, int flags, void* param) {
	if(event == CV_EVENT_LBUTTONDOWN){
		Vec3b intensity = bgr_image.at<Vec3b>(y, x);

		cout << intensity << endl;

		if((int)intensity[0] -20 < 0)
			field_h_Low = 0;
		else
			field_h_Low = (int)intensity[0] -20;
		
		if((int)intensity[0] +20 > 255)
			field_h_High = 255;
		else
			field_h_High = (int)intensity[0] +20;

		if((int)intensity[1] -30 < 0)
			field_s_Low = 0;
		else
			field_s_Low = (int)intensity[1] -30;
		
		if((int)intensity[1] +30 > 255)
			field_s_High = 255;
		else
			field_s_High = (int)intensity[1] +30;					

		if((int)intensity[2] -30 < 0)
			field_v_Low = 0;
		else
			field_v_Low = (int)intensity[2] -30;
		
		if((int)intensity[2] +30 > 255)
			field_v_High = 255;
		else
			field_v_High = (int)intensity[2] +30;
	}
}


/*
	ballHandler
		-get the hsv value of the pixel

		-when clicked on the ball finder window this function is evoked
		-get the hsv value and assign it the global values of the ball hsv values 
*/
void ballHandler(int event, int x, int y, int flags, void* param) {
	if(event == CV_EVENT_LBUTTONDOWN){
		Vec3b intensity = bgr_image.at<Vec3b>(y, x);
		cout << intensity << " " << x << " " << y <<endl;
		if((int)intensity[0] -10 < 0)
			ball_h_Low = 0;
		else
			ball_h_Low = (int)intensity[0] -10;
		
		if((int)intensity[0] +10 > 255)
			ball_h_High = 255;
		else
			ball_h_High = (int)intensity[0] +10;

		if((int)intensity[1] -30 < 0)
			ball_s_Low = 0;
		else
			ball_s_Low = (int)intensity[1] -30;
		
		if((int)intensity[1] +30 > 255)
			ball_s_High = 255;
		else
			ball_s_High = (int)intensity[1] +30;					

		if((int)intensity[2] -30 < 0)
			ball_v_Low = 0;
		else
			ball_v_Low = (int)intensity[2] -30;
		
		if((int)intensity[2] +30 > 255)
			ball_v_High = 255;
		else
			ball_v_High = (int)intensity[2] +30;

		cout << ball_h_Low << " " << ball_s_Low << " " << ball_v_Low<< endl;
	}
}

/*
	extractField
		-extract the field and return it

		- img is the hsv image
		- field is for returning the extracted fields
*/
void extractField(Mat &img, Mat &field)
{
	/// GUI with trackbar for tuning the field 
	namedWindow( "Field Control", CV_WINDOW_AUTOSIZE );
	createTrackbar( "h (Low):", "Field Control", &field_h_Low, 255, updateCanny_Low);
	createTrackbar( "h (High):", "Field Control", &field_h_High, 255, updateCanny_High);
	createTrackbar( "s (Low):", "Field Control", &field_s_Low, 255, updateCanny_Low);
	createTrackbar( "s (High):", "Field Control", &field_s_High, 255, updateCanny_High);
	createTrackbar( "v (Low):", "Field Control", &field_v_Low, 255, updateCanny_Low);
	createTrackbar( "v (High):", "Field Control", &field_v_High, 255, updateCanny_High);

	Mat hsv_threshold;

	inRange(img, cv::Scalar(field_h_Low, field_s_Low, field_v_Low), cv::Scalar(field_h_High, field_s_High, field_v_High), hsv_threshold);
	
	Mat element = getStructuringElement(MORPH_RECT,Size(50,50), Point(-1,-1));
	morphologyEx(hsv_threshold, hsv_threshold, MORPH_CLOSE , element);

	Mat canny;

	Mat result = original.clone();
	Mat out;
	result.copyTo(out, hsv_threshold);

	field = out;
}

/*
	trackBall
		-extract the field and return it

		- img is the hsv image

		returns the position of the ball and the radius
		x, y is the position and z is the radius
		x, y is -1 if there is no ball found
*/

Point3i trackBall(Mat &img)
{
	Point3i result;
	namedWindow( "Ball Control", CV_WINDOW_AUTOSIZE );
	createTrackbar( "h (Low):", "Ball Control", &ball_h_Low, 255, updateCanny_Low);
	createTrackbar( "h (High):", "Ball Control", &ball_h_High, 255, updateCanny_High);
	createTrackbar( "s (Low):", "Ball Control", &ball_s_Low, 255, updateCanny_Low);
	createTrackbar( "s (High):", "Ball Control", &ball_s_High, 255, updateCanny_High);
	createTrackbar( "v (Low):", "Ball Control", &ball_v_Low, 255, updateCanny_Low);
	createTrackbar( "v (High):", "Ball Control", &ball_v_High, 255, updateCanny_High);

	Mat hsv_threshold;

	//extract the color using the hsv values
	inRange(img, cv::Scalar(ball_h_Low, ball_s_Low, ball_v_Low), cv::Scalar(ball_h_High, ball_s_High, ball_v_High), hsv_threshold);
	

	//morphological operation to reduce noice
	Mat element = getStructuringElement(MORPH_RECT,Size(20,20));
	morphologyEx(hsv_threshold, hsv_threshold, MORPH_CLOSE , element);
	imshow("Ball Control",hsv_threshold);
	Mat canny;

	Canny(hsv_threshold,canny, 50, 255, 3);

	vector<cv::Vec3f> circles;
	size_t pos = -1;
	float maxRadius = 0;
	
	vector<vector<Point> > contours;
	
	//get all the contours in the image
	findContours(canny, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
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
		if (ratio > 0.75 && bBox.area() > 200 && bBox.area() < 10000)
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
		approxPolyDP(Mat(balls[i]), cir[i], 3, true);
		
		//get the center and the radius of the circle
		minEnclosingCircle(cir[i], center[i], radius[i]);
		
		//get the circle with max radius	
		if ((cir[i]).size() > 3 && radius[i] > maxRadius)
		{
			maxRadius = radius[i];
			pos = i;
		}
	}	

	if (pos != -1)
	{
		//store the results;
		result = Point3i((int)(center[pos].x), (int)round(center[pos].y),(int) (radius[pos]));
	}
	else{
		//if no circles found x, y, z = -1
		result = Point3i(-1,-1,-1);
	}	

	return result;	
}


/* 
	drawField function detects line segments from the extracted field img, and draws the lines as different
	colours per features.
*/
void drawField(Mat &hsv_img){
	namedWindow( "Line Control", CV_WINDOW_AUTOSIZE );
	createTrackbar( "h (Low):", "Line Control", &line_h_Low, 255, updateCanny_Low);
	createTrackbar( "h (High):", "Line Control", &line_h_High, 255, updateCanny_High);
	createTrackbar( "s (Low):", "Line Control", &line_s_Low, 255, updateCanny_Low);
	createTrackbar( "s (High):", "Line Control", &line_s_High, 255, updateCanny_High);
	createTrackbar( "v (Low):", "Line Control", &line_v_Low, 255, updateCanny_Low);
	createTrackbar( "v (High):", "Line Control", &line_v_High, 255, updateCanny_High);

	medianBlur(hsv_img, hsv_img, 5);
	Mat cann, img;
	inRange(hsv_img, cv::Scalar(line_h_Low, line_s_Low, line_v_Low), cv::Scalar(line_h_High, line_s_High, line_v_High), img);
	//bitwise_not ( cann, img );
	imshow("Line Control", img);
	resize(img, img, Size(300,300),0,0,CV_INTER_LINEAR);
	Mat mask = Mat::ones( img.size(), CV_8UC1 );

	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
	    // Detect the lines
	vector<Vec4f> lines_std;
	std::vector<KeyLine> keyLines;
    ls->detect(img, lines_std);

	// convert all the vec4f to KeyLine for calculation convenience.
	for(int i = 0; i < lines_std.size(); i++)
	{
		KeyLine key = KeyLine();
		Vec4f vec = lines_std[i];
		key.startPointX = vec[0];
		key.startPointY = vec[1];
		key.endPointX =  vec[2];
		key.endPointY = vec[3];
		keyLines.push_back(key);
	}

	// sort the lines by x in ascending order using startPointX
	sortKeyLines(keyLines);
	cv::Mat output = original.clone();
	if( output.channels() == 1 )
		cvtColor( output, output, COLOR_GRAY2BGR );

	vector<KeyLine> mergedLines, tCornerLines, tCornerLCornerLines, normalLines, circleLineCorners;
	cleanUpLines( keyLines, mergedLines);
	detectTCorners( mergedLines, tCornerLines, normalLines );
	detectCircleTCorner( mergedLines, circleLineCorners );
	drawLines(output, mergedLines, 2);
	drawLines(output, tCornerLines, 3);
	drawLines(output, circleLineCorners, 0);
	
	imshow("Output", output);
}


/* Summary: cleanUpLines is the main merging function which merges all the colinear and parallel lines detected by the LSD function. 
*/
void cleanUpLines( vector<KeyLine> &lines, vector<KeyLine> &mergedLines){
	vector<KeyLine> tempLines;

	// in this do_while loop, we join all the colinear lines.
	do {
		if( tempLines.size() > 1 ){
			lines = tempLines;
			tempLines = vector<KeyLine>();
		}
		bool mainLineIsSelected = false;
		KeyLine mainLine;
		int size = lines.size();
		for( int i = 0; i < size; i++ ){
			KeyLine line = lines[i];
			if( !mainLineIsSelected ){
				mainLine = line;
				mainLineIsSelected = true;
			} else {
				switchStartAndEnd(mainLine); // to make sure that startPointX is the smaller value than endPointX at all times.
				if( areSameLines(mainLine, line) ){ // we compare the mainLine and kl to see if we need to merge.
					mergeLines(mainLine, line);
				} else {
					tempLines.push_back(lines[i]);	
				}	
			} 
		}
		mergedLines.push_back(mainLine);
	} while(tempLines.size() > 0);


	// to make sure startPointX is always the smaller value than endPointX.
	for( int i = 0; i < mergedLines.size(); i++ ){
		switchStartAndEnd(mergedLines[i]);
	}

	// This segment of code merges parallel lines.
	vector<KeyLine> parallelLines;
	int size = mergedLines.size();
	for( int i = 0; i < size; i++ ){
		for( int j = i + 1; j < size; j++ ){
			if( mergeParalellLine( mergedLines[i], mergedLines[j]))
				parallelLines.push_back(mergedLines[j]);
		}
	}
	for( int i = 0; i < parallelLines.size(); i++ ){
		float toCompareX = parallelLines[i].startPointX;
		float toCompareY = parallelLines[i].startPointY;
		for( int j = 0; j < mergedLines.size(); j++ ){
			if( toCompareX == mergedLines[j].startPointX && toCompareY == mergedLines[j].startPointY ){
				mergedLines.erase(mergedLines.begin() + j);
				break;
			}
		}	
	}
}


/* Summary: drawLines simply draws lines.
 */
void drawLines( Mat &output, vector<KeyLine> &keyLines, int colour){
	for( unsigned int i = 0; i < keyLines.size(); i++ ) {
		KeyLine kl = keyLines[i];
		Point pt1 = Point( kl.startPointX, kl.startPointY );
		Point pt2 = Point( kl.endPointX, kl.endPointY );

		/* draw line */
		if( colour == 0 ) // red T corners.
			line( output, pt1, pt2, Scalar( 0, 0, 255 ), 5 );
		else if( colour == 1 ) // yellow centre circle
			line( output, pt1, pt2, Scalar( 0, 255, 255 ), 5 );
		else if( colour == 2 ) // blue normal lines
			line( output, pt1, pt2, Scalar( 255, 0, 0 ), 5 );
		else if( colour == 3 ) // pink for circle line corners.
			line( output, pt1, pt2, Scalar( 180, 105, 255 ), 5 );			
	}
}


/* Summary: main merging function for the parallel lines.
*/
bool mergeParalellLine(KeyLine &kl1, KeyLine &kl2){
	bool areParallel = false;
	float angleDifference = std::abs(calcAngle(kl1) - calcAngle(kl2));
	float startToStartX = std::abs(kl1.startPointX - kl2.startPointX);
	float endToStartX = std::abs(kl1.startPointX - kl2.endPointX);
	float xDistance = (startToStartX <= endToStartX) ? startToStartX : endToStartX;
	float startToStartY = std::abs(kl1.startPointY - kl2.startPointY);
	float endToStartY = std::abs(kl1.startPointY - kl2.endPointY);
	float yDistance = (startToStartY <= endToStartY) ? startToStartY : endToStartY;

	// we consider any line that has angle less than 25 degrees, with distance less than 45.0f between any extremes as parallel lines.
	if( angleDifference < 25.f ){
		if( (xDistance <= 25.0f || yDistance <= 25.0f) && calcDistance(&kl1, &kl2) < 45.0f){
			areParallel = true;
		}
	}

	// if two lines are parallel, we take the midpoints of each extreme.
	if( areParallel ){
		kl1.startPointX = (kl1.startPointX + kl2.startPointX)/2;
		kl1.startPointY = (kl1.startPointY + kl2.startPointY)/2;
		kl1.endPointX = (kl1.endPointX + kl2.endPointX)/2;
		kl1.endPointY = (kl2.endPointY + kl2.endPointY)/2;
	}
	return areParallel;
}


/* If two lines are near to one another, and have similar angles, we consider
them to be the colinear lines to be merged. */
bool areSameLines(KeyLine &kl1, KeyLine &kl2){
	float distance = calcDistance(&kl1, &kl2);
	float angleDifference = std::abs(calcAngle(kl1) - calcAngle(kl2));
	return (distance < 33.f  && angleDifference < 30.f) ? true : false;
}


/* Summary: since the lines are already sorted by x in ascending order, we can simply extend mainLine
by setting its endPoint to kl's endPoint. */
void mergeLines(KeyLine &mainLine, KeyLine &kl){
	mainLine.endPointX = kl.endPointX;
	mainLine.endPointY = kl.endPointY;
}


/* Summary: calcDistance calculates and returns the minimum distance amongst the combinations of 4 points (2 line segments)*/
float calcDistance(KeyLine *kl1, KeyLine *kl2){
	Point kl1StartPt = Point( kl1->startPointX, kl1->startPointY );
	Point kl1EndPt = Point( kl1->endPointX, kl1->endPointY );
	Point kl2StartPt = Point( kl2->startPointX, kl2->startPointY);
	Point kl2EndPt = Point( kl2->endPointX, kl2->endPointY );

	float startToStartXDiff = kl1StartPt.x - kl2StartPt.x;
	float startToEndXDiff = kl1StartPt.x - kl2EndPt.x;
	float startToStartYDiff = kl1StartPt.y - kl2StartPt.y;
	float startToEndYDiff = kl1StartPt.y - kl2EndPt.y;
	float endToStartXDiff = kl1EndPt.x - kl2StartPt.x;
	float endToEndXDiff = kl1EndPt.x - kl2EndPt.x;
	float endToStartYDiff = kl1EndPt.y - kl2StartPt.y;
	float endToEndYDiff = kl1EndPt.y - kl2EndPt.y;
	
	float startToStart = sqrt(startToStartXDiff*startToStartXDiff + startToStartYDiff*startToStartYDiff);
	float startToEnd = sqrt(startToEndXDiff*startToEndXDiff + startToEndYDiff*startToEndYDiff);
	float endToStart = sqrt(endToStartXDiff*endToStartXDiff + endToStartYDiff*endToStartYDiff);
	float endToEnd = sqrt(endToEndXDiff*endToEndXDiff + endToEndYDiff*endToEndYDiff);

	float startToStartEnd = (startToStart <= startToEnd) ? startToStart : startToEnd;
	float endToStartEnd = (endToStart <= endToEnd ) ? endToStart : endToEnd;
	return ( (startToStartEnd <= endToStartEnd ) ? startToStartEnd : endToStartEnd );
}


/* Summary: calcAngle simply caculates the angle of the given line segment in range of 0 ~ 180 degrees. */
float calcAngle(KeyLine &kl){
	float angle;
	angle = atan2(kl.startPointY, kl.startPointX)  - atan2(kl.endPointY, kl.endPointX);
	angle = angle * 360.0f / (2*PI);
	if( angle < 0 )
		angle += 360;
	return (angle > 180.0f) ? angle - 180.0f : angle;
}


/* Summary: sortKeyLines simply sorts the given keyLines by x in ascending order using startPointX. 
If the startPointX is larger than its endPointX, we simply switch the extreme the other way.*/
void sortKeyLines(vector<KeyLine> &keyLines){
	if( keyLines.size() > 1 ){
		int i, key, j; 
		for (i = 1; i < keyLines.size(); i++) 
		{ 
			switchStartAndEnd(keyLines[i]);
			if( i == 1 )
				switchStartAndEnd(keyLines[0]);
			float key = keyLines[i].startPointX; 
			KeyLine keyLine = keyLines[i];
			j = i-1; 
		
			while (j >= 0 && keyLines[j].startPointX > key) 
			{ 
				keyLines[j+1] = keyLines[j]; 
				j = j-1; 
			} 
			keyLines[j+1] = keyLine; 
		} 
	}
}


/* Summary: helper function to ensure that startPointX is always the smaller value. */
void switchStartAndEnd(KeyLine &kl){
	if( kl.startPointX > kl.endPointX ){
		float tempX = kl.startPointX;
		float tempY = kl.startPointY;
		kl.startPointX = kl.endPointX;
		kl.endPointX = tempX;
		kl.startPointY = kl.endPointY;
		kl.endPointY = tempY;
	}
}


/* Summary: detectLCorners detect all the L corners using intersection points, and distance between extremes. */
void detectLCorners(vector<KeyLine> lines, vector<KeyLine> &cornerLines){
	int linesSize = lines.size();
	for( int i = 0; i < linesSize - 1; i++ ){
		KeyLine mainLine = lines[i];
		for( int j = i + 1; j < linesSize; j++ ){
			KeyLine toCompare = lines[j];
			if( isLCorner( mainLine, toCompare ) ){
				cornerLines.push_back(mainLine);
				cornerLines.push_back(toCompare);
				break;
			}
		}
	}
}


/* Summary: isLCorner simply returns true or false boolean value by checking if kl1 and kl2 are L corner relation. */
bool isLCorner(KeyLine &kl1, KeyLine &kl2){
	bool isLCorner = false;
	float angleDifference = std::abs(calcAngle(kl1) - calcAngle(kl2));
	if( angleDifference > 50 ){
		Point kl1StartPt = Point( kl1.startPointX, kl1.startPointY);
		Point kl1EndPt = Point( kl1.endPointX, kl1.endPointY);
		Point kl2StartPt = Point( kl2.startPointX, kl2.startPointY);
		Point kl2EndPt = Point( kl2.endPointX, kl2.endPointY);
		// if two lines intersect when the angle > 50, then we check each extremes of the line 
		// to see if the lines are intersecting as T.
		if( doIntersect(kl1StartPt, kl1EndPt, kl2StartPt, kl2EndPt) ){
			float distance = calcDistance(&kl1, &kl2); 
			if( distance <= 25.0f )
				isLCorner = true;
		}
	}
	return isLCorner;
}


/* Summary: detectCircleTCorner detects the centre line T corner. The idea is that if there no L corner present 
on the field other, then we consider the T corner as centire line T corner. (can be updated using robot's position in 
the future.) */
void detectCircleTCorner(vector<KeyLine> &mergedLines, vector<KeyLine> &cornerLines){
	int mergedLinesSize = mergedLines.size();
	for( int i = 0; i < mergedLinesSize-1; i++ ){
		KeyLine mainLine = mergedLines[i];
		for( int j = i + 1; j < mergedLinesSize; j++ ){
			KeyLine toCompare = mergedLines[j];
			if( isTCorner( mainLine, toCompare ) ){
				if( isTCorner( mainLine, toCompare ) ){
					KeyLine tCornerLine = checkTIntersection(mainLine, toCompare, getIntersectionPoint( mainLine, toCompare ));

					bool isCirCleLineCorner = true;
					vector<KeyLine> LCorners;
					detectLCorners(mergedLines, LCorners);
					if( LCorners.size() >= 1 ){
						isCirCleLineCorner = false;
					}
					if( isCirCleLineCorner )
						cornerLines.push_back(tCornerLine);
					break;
				}
			}
		}
	}
}


/* Summary: checkTIntersection simply checks if there exists a L corner that is connected to the goal line T corner */
KeyLine checkTIntersection(KeyLine kl1, KeyLine kl2, Point intersectionPoint){
	float kl1StartPtDist = sqrt( pow( kl1.startPointX - intersectionPoint.x, 2 ) + pow(kl1.startPointY - intersectionPoint.y, 2));
	float kl1EndPtDist = sqrt( pow( kl1.endPointX - intersectionPoint.x, 2 ) + pow(kl1.endPointY - intersectionPoint.y, 2));
	float kl2StartPtDist = sqrt( pow( kl2.startPointX - intersectionPoint.x, 2 ) + pow(kl2.startPointY - intersectionPoint.y, 2));
	float kl2EndPtDist = sqrt( pow( kl2.endPointX - intersectionPoint.x, 2 ) + pow(kl2.endPointY - intersectionPoint.y, 2));
	float kl1Dist = (kl1StartPtDist < kl1EndPtDist) ? kl1StartPtDist : kl1EndPtDist;
	float kl2Dist = (kl2StartPtDist < kl2EndPtDist) ? kl2StartPtDist : kl2EndPtDist;
	return (kl1Dist < kl2Dist) ? kl1 : kl2;
}


/* detectTCorners: detects T corners on the field by looking at the distance between extremes and intersection point as well as the angle */
void detectTCorners(vector<KeyLine> &mergedLines, vector<KeyLine> &cornerLines, vector<KeyLine> &normalLines){
	int mergedLinesSize = mergedLines.size();
	for( int i = 0; i < mergedLinesSize-1; i++ ){
		KeyLine mainLine = mergedLines[i];
		for( int j = i + 1; j < mergedLinesSize; j++ ){
			KeyLine toCompare = mergedLines[j];
			if( isTCorner( mainLine, toCompare ) ){
				KeyLine tCornerLine = checkTIntersection(mainLine, toCompare, getIntersectionPoint( mainLine, toCompare ));
				cornerLines.push_back(tCornerLine);

				for( int k = 0; k < mergedLinesSize; k++ ){
					if( isLCorner(tCornerLine, mergedLines[k])) {
						cornerLines.push_back(mergedLines[k]);
					}
				}
			}
		}
	}
}


/* Summary: isTCorner simply returns true if the two lines are T corner related, false otherwise. */ 
bool isTCorner(KeyLine &kl1, KeyLine &kl2){
	bool isTCorner = false;
	float angleDifference = std::abs(calcAngle(kl1) - calcAngle(kl2));
	if( angleDifference > 50 ){
		Point kl1StartPt = Point( kl1.startPointX, kl1.startPointY);
		Point kl1EndPt = Point( kl1.endPointX, kl1.endPointY);
		Point kl2StartPt = Point( kl2.startPointX, kl2.startPointY);
		Point kl2EndPt = Point( kl2.endPointX, kl2.endPointY);
		
		// if two lines intersect when the angle > 50, then we check each extremes of the line 
		// to see if the lines are intersecting as T.
		if( doIntersect(kl1StartPt, kl1EndPt, kl2StartPt, kl2EndPt) ){
			float distance = calcDistance(&kl1, &kl2);
			if( distance > 15.0f ){
				isTCorner = true;
			}
		}
	}
	return isTCorner;
}


// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point p, Point q, Point r) 
{ 
    // for details of below formula. 
    int val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 


// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
       return true; 
  
    return false; 
} 


// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point p1, Point q1, Point p2, Point q2) 
{ 
	float lineExtensionLength = 30;
	if( p1.x < q1.x ) {
		p1.x = p1.x - lineExtensionLength;
		q1.x = q1.x + lineExtensionLength;
	} else {
		p1.x = p1.x + lineExtensionLength;
		q1.x = q1.x - lineExtensionLength;
	}
	if( p1.y < q1.y ) {
		p1.y = p1.y - lineExtensionLength;
		q1.y = q1.y + lineExtensionLength;
	} else {
		p1.y = p1.y + lineExtensionLength;
		q1.y = q1.y - lineExtensionLength;
	}
	if( p2.x < q2.x ) {
		p2.x = p2.x - lineExtensionLength;
		q2.x = q2.x + lineExtensionLength;
	} else {
		p2.x = p2.x + lineExtensionLength;
		q2.x = q2.x - lineExtensionLength;
	}
	if( p2.y < q2.y ) {
		p2.y = p2.y - lineExtensionLength;
		q2.y = q2.y + lineExtensionLength;
	} else {
		p2.y = p2.y + lineExtensionLength;
		q2.y = q2.y - lineExtensionLength;
	}
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
} 


/* Summary: getIntersectionPoint receives two KeyLines and finds the intersection point of these two lines. 
If none, it returns Point(-1, -1). */
Point getIntersectionPoint(KeyLine &kl1, KeyLine &kl2 ){
    float dx, dy;
    dx = kl1.endPointX - kl1.startPointX;
    dy = kl1.endPointY - kl1.startPointY;
	float m1 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    float c1 = kl1.startPointY - m1 * kl1.startPointX; // which is same as y2 - slope * x2
    dx = kl2.endPointX - kl2.startPointX;
    dy = kl2.endPointY - kl2.startPointY;
    float m2 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    float c2 = kl2.startPointY - m2 * kl2.startPointX; // which is same as y2 - slope * x2

    if( (m1 - m2) == 0)
        return Point(-1, -1);
    else {
        float intersection_X = (c2 - c1) / (m1 - m2);
        float intersection_Y = m1 * intersection_X + c1;
		return Point(intersection_X, intersection_Y);
    }
}