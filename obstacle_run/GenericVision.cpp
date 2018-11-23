/*
 * GenericVision.cpp
 * Created on: June 20 2017
 * Author: MengCheng Lau
 * Author: Andy Chi Fung Lun
 */
#include "GenericVision.h"

using namespace cv;
using namespace std;

GenericVision gv;

static int marker_xml = 1;

/// This is used later by std::sort()
struct sortComparator
{
		bool operator()(Point pt1, Point pt2)
		{
			return (pt1.y < pt2.y);
		}
} sortbyx;

GenericVision::GenericVision(void)
{
	sbHSVGain = Vec3i(10, 20, 20);
	sbCannyLow = 100;
	sbCannyHigh = 255;
}

GenericVision::GenericVision(Mat m_rawFrame, Mat m_hsvFrame)
{
	rawFrame = m_rawFrame;
}

GenericVision::~GenericVision(void)
{
}

char* getXmlName() {
	char* xml_name = (char*) malloc(12);
	snprintf(xml_name, 12, "marker%d.xml", marker_xml);
	return xml_name;
}

/// Mouse callback function to track the mouse event
void GenericVision::CallBackFunc(int event, int x, int y, int flags, void *userdata)
{
	Mat *HSVFrame = (Mat *) userdata;
	/// Left button
	if (event == EVENT_LBUTTONDOWN)
	{
		gv.mousePt1 = Point2f(x, y);
		cout << "Mouse click at" << gv.mousePt1 << endl;
	}
	else if (event == EVENT_LBUTTONUP)
	{
		gv.mousePt2 = Point2f(x, y);
		cout << "Mouse release at" << gv.mousePt2 << endl;
		Mat selection = Mat(*HSVFrame, Rect(gv.mousePt1, gv.mousePt2));
		Scalar averageHSV = mean(selection); /// get the average HSV in the selection area
		Vec3i avgHSV = Vec3i(int(averageHSV.val[0]), int(averageHSV.val[1]),
												 int(averageHSV.val[2])); /// convert scalar to vec3i

		char* xml_name = getXmlName();
		gv.saveHSVAvg(xml_name, avgHSV, gv.sbHSVGain); /// save the parameters to xml
		free(xml_name);
	}
}

void hsvTrackbarCbFunc(int posn, int index)
{
	char* xml_name = getXmlName();

	Vec3i hsvAvg, hsvGain;
	GenericVision vision;
	vision.readHSVAvg(xml_name, hsvAvg, hsvGain);
	gv.sbHSVGain[index] = posn;

	gv.saveHSVAvg(xml_name, hsvAvg, gv.sbHSVGain);
	free(xml_name);
}

void hTrackbarCbFunc(int posn, void* userdata) {
	hsvTrackbarCbFunc(posn, 0);
}

void sTrackbarCbFunc(int posn, void* userdata) {
	hsvTrackbarCbFunc(posn, 1);
}

void vTrackbarCbFunc(int posn, void* userdata) {
	hsvTrackbarCbFunc(posn, 2);
}

void markerTrackbarCbFunc(int posn, void* userdata) {
	Vec3i hsvAvg, hsvGain;
	GenericVision vision;
	char* xml_name = getXmlName();
	vision.readHSVAvg(xml_name, hsvAvg, hsvGain);

	setTrackbarPos("H (Gain):", "Control", hsvGain[0]);
	setTrackbarPos("S (Gain):", "Control", hsvGain[1]);
	setTrackbarPos("V (Gain):", "Control", hsvGain[2]);

	free(xml_name);
}

double GenericVision::getDist(Point p0, Point p1)
{
	return sqrt((p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y));
}

/// save selected colour space to XML using FileStorage
void GenericVision::saveHSVAvg(const char *filename, Vec3i hsvAvg, Vec3i hsvGain)
{
	FileStorage fs(filename, FileStorage::WRITE);

	fs << "hsvAvg" << hsvAvg;
	fs << "hsvGain" << hsvGain;
	fs.release(); // explicit close
	//cout << "Write Done." << endl;
}

/// read saved colour space from XML using FileStorage
void GenericVision::readHSVAvg(const char *filename, Vec3i &hsvAvg, Vec3i &hsvGain)
{
	FileStorage fs(filename, FileStorage::READ);

	fs["hsvAvg"] >> hsvAvg;
	fs["hsvGain"] >> hsvGain;
	fs.release(); // explicit close
}

void GenericVision::saveCannyThresholds(const char *filename, Vec3i minHSV, Vec3i maxHSV)
{
	FileStorage fs(filename, FileStorage::WRITE);

	fs << "minHSV" << minHSV;
	fs << "maxHSV" << maxHSV;

	fs.release(); // explicit close
	//cout << "Write Done." << endl;
}

void GenericVision::readCannyThresholds(const char *filename, Vec3i &minHSV, Vec3i &maxHSV)
{
	FileStorage fs(filename, FileStorage::READ);
	fs["minHSV"] >> minHSV;
	fs["maxHSV"] >> maxHSV;
	fs.release(); // explicit close
}

void GenericVision::initGUI()
{
	/// GUI with trackbar
	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	namedWindow("Colour1", CV_WINDOW_AUTOSIZE);
	namedWindow("Colour2", CV_WINDOW_AUTOSIZE);
	namedWindow("Colour3", CV_WINDOW_AUTOSIZE);
	namedWindow("Debug", CV_WINDOW_AUTOSIZE);
	createTrackbar("Marker:", "Control", &marker_xml, 3, markerTrackbarCbFunc);
	createTrackbar("Canny Threshold (Low):", "Control", &sbCannyLow, 255);
	createTrackbar("Canny Threshold (High):", "Control", &sbCannyHigh, 255);
	createTrackbar("H (Gain):", "Control", &sbHSVGain[0], 60, hTrackbarCbFunc);
	createTrackbar("S (Gain):", "Control", &sbHSVGain[1], 100, sTrackbarCbFunc);
	createTrackbar("V (Gain):", "Control", &sbHSVGain[2], 100, vTrackbarCbFunc);
	markerTrackbarCbFunc(0, NULL);
}

void GenericVision::showGUI()
{
	imshow("Control", rawFrame);
	imshow("Colour1", threshold1Frame);
	imshow("Colour2", threshold2Frame);
	imshow("Colour3", threshold3Frame);
}

/// find ball with HoughCircles 
/// input image: gray
Point3i GenericVision::findBall(Mat &camFrame, Mat &hsvThreshold, int minSize, int maxSize, double flushRatio/*0~1*/)
{
	bool checkBall = false;
	Point3i ball;
	vector<Vec3f> circles;

	Mat cannyFrame;

	/// flush/crop left and right of the input image with black borders
	rectangle(camFrame, Point(0, 0), Point(camFrame.size().width * flushRatio, camFrame.size().height), Scalar(0, 0, 0),
						-1, 1);
	rectangle(camFrame, Point(camFrame.size().width * (1.0 - flushRatio), 0),
						Point(camFrame.size().width, camFrame.size().height), Scalar(0, 0, 0), -1, 1);

	/// closing operation to fill the black pixels
	Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
	morphologyEx(hsvThreshold, hsvThreshold, 3, element); /// Opening: MORPH_OPEN : 2 Closing: MORPH_CLOSE: 3

	Canny(hsvThreshold, cannyFrame, sbCannyLow, sbCannyHigh, 3);

	/// Apply the Hough Transform to find the circles
	//HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, 10, 200, 100, 0, 0 );
	HoughCircles(cannyFrame, circles, CV_HOUGH_GRADIENT, 1, 300, 50, 10);
	//HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1,gray.rows/8/*min_dst*/,200/*thres_canny*/,100/*thres_ctr*/, 20/*min_rad*/,200/*max_rad*/);
	// HoughCircles( canny, circles, 3, 1, canny.rows/32, 255, 240, 5, 0 );

	/// Draw the circles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		/// circle center
		circle(camFrame, center, 3, Scalar(0, 255, 0), -1, 5, 0);
		/// circle outline
		circle(camFrame, center, radius, Scalar(0, 0, 255), 3, 5, 0);

		if ((radius > minSize) && (radius < maxSize))
		{
			ball = Point3i(center.x, center.y, radius);
			cout << "(" << center.x << "," << center.y << ")" << endl;

			break;
		}
		else
		{
			ball = Point3i(-1, -1, -1);
		}
	}

	return ball;
} /// findBall

/// find ball with two tones of thresholds (no houghcircle)  
/// input: main camera frame for marking, 2 Thresholded HSV image, default value: minSize =200, maxSize = 10000, flushRatio = 0.1
/// return marker: (x,y,size)
Point3i
GenericVision::findMixColouredObject(Mat &camFrame, Mat &hsvThreshold1, Mat &hsvThreshold2, Scalar colour, int minSize,
																		 int maxSize, int minEdge, int maxEdge, double flushRatio)
{

	Point3i marker;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<cv::Point> patchCorners;
	Mat canny, hsvThreshold;

	/// flush/crop left and right of the input image with black borders
	rectangle(camFrame, Point(0, 0), Point(camFrame.size().width * flushRatio, camFrame.size().height), Scalar(0, 0, 0),
						-1, 1);
	rectangle(camFrame, Point(camFrame.size().width * (1.0 - flushRatio), 0),
						Point(camFrame.size().width, camFrame.size().height), Scalar(0, 0, 0), -1, 1);


	hsvThreshold = Mat(camFrame.size(), CV_8U);

	bitwise_or(hsvThreshold1, hsvThreshold2, hsvThreshold);

	/// closing operation to fill the black pixels
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20)); /// MORPH_RECT, MORPH_ELLIPSE, MORPH_CROSS
	morphologyEx(hsvThreshold, hsvThreshold, 3, element); /// Opening: MORPH_OPEN : 2 Closing: MORPH_CLOSE: 3

	Canny(hsvThreshold, canny, sbCannyLow, sbCannyHigh, 3);

	///Find all contours from canny edges
	findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//imshow("Debug", hsvThreshold);
	/// Get the moments
	vector<Moments> mu(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}
imshow("Debug", hsvThreshold);
	/// Get the mass centers:
	vector<Point2f> mc(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	vector<Point> approxMarker;

	///Use contours to match polygons
	for (int i = 0; i < contours.size(); i++)
	{
		int patchSize = mu[i].m00;

		if (patchSize >= minSize && patchSize <= maxSize) /// moment area: if relatively large contour (frame) is detected
		{

			//drawContours(result, contours, i, CV_RGB(255,255,255 ), 1, 8, hierarchy, 0, cv::Point() );
			//approxPolyDP(contours[i], approxArrow, arcLength(Mat(contours[i]), true)*0.02, true);
			//convexHull( contours[i], approxArrow, false );
			//drawContours(result, approxArrow, i, CV_RGB(255,255,255 ), 1, 8, hierarchy, 0, cv::Point() );
			approxPolyDP(contours[i], approxMarker, arcLength(Mat(contours[i]), true) * 0.05, true);
			//for (int j = 0; j < approxArrow.size(); ++j)
			//cout << "Convex" << approxArrow.size() << endl;

			/// check number of corners
			if (approxMarker.size() >= minEdge && approxMarker.size() <= maxEdge)
			{
				//drawContours(camFrame, contours, i, colour, 2, 8, hierarchy, 0, Point());

				marker = Point3i(mc[i].x, mc[i].y, patchSize);

			} /// IF number of edges is confirmed 

			else
			{
				marker = Point3i(-1, -1, -1);
			}

		} // IF large moments
	} // FOR contour loop
	return marker;
}

/// find marathon arrow signs
/// input: coloured frame, default value: minSize = 2000, maxSize = 18000
int GenericVision::findArrowPatch(Mat &camFrame, int minSize, int maxSize, double flushRatio/*0~1*/)
{
	int arrowType = 0;
	Mat gray, canny;
	vector<Point> patchCorners;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat result;
	Mat signROI(cv::Size(60, 60), CV_8U); /// default ROI for marathon arrow sign

	/// Load template for Marathon 
	Mat tempLeft = imread("left.png", CV_LOAD_IMAGE_GRAYSCALE);
	threshold(tempLeft, tempLeft, 0, 255, THRESH_BINARY + THRESH_OTSU /*0: white arrow 1: black arrow*/);
	//Canny( tempLeft, tempLeft, thresh, thresh*2, 3 );
	Mat tempRight = imread("right.png", CV_LOAD_IMAGE_GRAYSCALE);
	threshold(tempRight, tempRight, 0, 255, THRESH_BINARY + THRESH_OTSU /*0: white arrow 1: black arrow*/);
	//Canny( tempRight, tempRight, thresh, thresh*2, 3 );
	Mat tempForward = imread("forward.png", CV_LOAD_IMAGE_GRAYSCALE);
	threshold(tempForward, tempForward, 0, 255, THRESH_BINARY + THRESH_OTSU /*0: white arrow 1: black arrow*/);

	/// flush/crop left and right of the input image with black borders
	rectangle(camFrame, Point(0, 0), Point(camFrame.size().width * flushRatio, camFrame.size().height), Scalar(0, 0, 0),
						-1, 1);
	rectangle(camFrame, Point(camFrame.size().width * (1.0 - flushRatio), 0),
						Point(camFrame.size().width, camFrame.size().height), Scalar(0, 0, 0), -1, 1);

	cvtColor(camFrame, gray, CV_BGR2GRAY);
	Canny(gray, canny, sbCannyLow, sbCannyHigh, 3);

	///Find all contours from canny edges
	findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	result = Mat::zeros(camFrame.size(), CV_8UC3);

	/// Get the moments
	vector<Moments> mu(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	///  Get the mass centers:
	vector<Point2f> mc(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	vector<Point> approxArrow;

	///Use contours to match polygons
	for (int i = 0; i < contours.size(); i++)
	{
		int patchSize = mu[i].m00;

		bool checkSign = false; // rectangle of checks
		if (patchSize > minSize && patchSize < maxSize) // moment area: if relatively large contour (frame) is detected
		{

			//drawContours(result, contours, i, CV_RGB(255,255,255 ), 1, 8, hierarchy, 0, cv::Point() );
			//approxPolyDP(contours[i], approxArrow, arcLength(Mat(contours[i]), true)*0.02, true);
			//convexHull( contours[i], approxArrow, false );
			//drawContours(result, approxArrow, i, CV_RGB(255,255,255 ), 1, 8, hierarchy, 0, cv::Point() );
			approxPolyDP(contours[i], approxArrow, arcLength(Mat(contours[i]), true) * 0.05, true);
			//for (int j = 0; j < approxArrow.size(); ++j)
			//cout << "Convex" << approxArrow.size() << endl;

			/// arrow sign with 4 points
			if (approxArrow.size() == 4)
			{
				//cout << "Detect Rect:" << mu[i].m00 << endl;
				int angle[4];
				/// sort vector using myobject as comparator
				sort(approxArrow.begin(), approxArrow.end(), sortbyx);
				patchCorners = approxArrow;

				/// arrange the corner position if the drawing pad is not align properly, only if skew to the left
				/// clockwise order from top-left
				if (approxArrow[0].x > approxArrow[1].x)
				{
					patchCorners[0] = approxArrow[1];
					patchCorners[1] = approxArrow[0];
					patchCorners[2] = approxArrow[2];
					patchCorners[3] = approxArrow[3];
				}
				else
				{
					patchCorners[0] = approxArrow[0];
					patchCorners[1] = approxArrow[1];
					patchCorners[2] = approxArrow[3];
					patchCorners[3] = approxArrow[2];
				}

				double pointDist[4];
				pointDist[0] = getDist(patchCorners[0], patchCorners[1]);
				pointDist[1] = getDist(patchCorners[1], patchCorners[2]);
				pointDist[2] = getDist(patchCorners[2], patchCorners[3]);
				pointDist[3] = getDist(patchCorners[3], patchCorners[0]);

				int distError = 60;  /// threshold for error between edges of square
				if ((abs(pointDist[0] - pointDist[1]) + abs(pointDist[1] - pointDist[2]) + abs(pointDist[2] - pointDist[3]) +
						 abs(pointDist[3] - pointDist[0])) < distError)
				{
					checkSign = true;
				}


				if (checkSign) /// confirmation of square patch
				{
					//cout<<"SIGN FOUND: " << endl;
					//drawContours(mat_frame, contours, i, Scalar(0, 255, 255), cv::Point()); // fill Yellow
					drawContours(camFrame, contours, i, Scalar(255, 0, 0), 2, 8, hierarchy, 0, Point());

					for (int p = 0; p < 4; ++p)
					{
						if (p == 0)
							circle(camFrame, patchCorners[p], 3, Scalar(0, 0, 255), 1); // red
						else if (p == 1)
							circle(camFrame, patchCorners[p], 3, Scalar(0, 255, 0), 1); // green
						else if (p == 2)
							circle(camFrame, patchCorners[p], 3, Scalar(255, 0, 0), 1); // blue
						//else circle(mat_frame, patchCorners[p], 3, Scalar(0, 0, 0), 1);
						//cout<< "Point " << p << patchCorners[p] << endl;
					}

					/// ***** Check arrow type by pixel ROI ****************			
					Point2f signSrcCorners[4], signROICorners[4];
					//Mat signROI(cv::Size(60, 60), CV_8U);

					for (int j = 0; j < 4; j++)
						signSrcCorners[j] = patchCorners[j];

					/// clockwise order from top-left
					signROICorners[0].x = 0;
					signROICorners[0].y = 0;
					signROICorners[1].x = 60;
					signROICorners[1].y = 0;
					signROICorners[2].x = 60;
					signROICorners[2].y = 60;
					signROICorners[3].x = 0;
					signROICorners[3].y = 60;

					/// map the four corners to a perspective transform image
					Mat signPerspectiveTransform = getPerspectiveTransform(signSrcCorners, signROICorners);
					/// warp perspective to correct the transformation
					warpPerspective(gray, signROI, signPerspectiveTransform, cvSize(60, 60), cv::INTER_LINEAR,
													cv::BORDER_REPLICATE, 0);
					/// get the binary output of the arrow patch using thresholding
					GaussianBlur(signROI, signROI, Size(5, 5), 2, 2);
					threshold(signROI, signROI, 0, 255, THRESH_BINARY + THRESH_OTSU /*0: white arrow 1: black arrow*/);
					//threshold( signROI, signROI, 150, 255,THRESH_BINARY /*0: white arrow 1: black arrow*/); 
					//adaptiveThreshold(signROI, signROI, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);
					imwrite("signROI.png", signROI);
					/// comparing the arrow's ROI to the template 60x60
					int checkArrowLeft = 0;
					int checkArrowRight = 0;
					int checkArrowForward = 0;
					for (int w = 0; w < 60; ++w)
					{
						for (int h = 0; h < 60; ++h)
						{
							if (signROI.at < unsigned char > (h, w) == tempLeft.at < unsigned char > (h, w))
								checkArrowLeft++;
							else
								checkArrowLeft--;

							if (signROI.at < unsigned char > (h, w) == tempRight.at < unsigned char > (h, w))
							checkArrowRight++;
							else
								checkArrowRight--;

							if (signROI.at < unsigned char > (h, w) == tempForward.at < unsigned char > (h, w))
								checkArrowForward++;
							else
								checkArrowForward--;
						}
					}

					//cout<< checkArrowLeft << "," << checkArrowRight << ","<< checkArrowForward << endl;
					/// will consider all back pointing signs and forward, adjust the threshold or modify the conditions to fit your needs
					if (checkArrowForward > 2600) // all arrow pointing to front, use 3200 for FORWARD only
					{
						arrowType = FORWARD_ARROW;
						cout << "FORWARD_ARROW" << endl;
					}
					else if (checkArrowLeft > 2800)
					{
						arrowType = LEFT_ARROW;
						cout << "LEFT_ARROW" << endl;
					}
					else if (checkArrowRight > 2800)
					{
						arrowType = RIGHT_ARROW;
						cout << "RIGHT_ARROW" << endl;
					}
					else if (checkArrowLeft > 1900 && checkArrowRight > 2200)  // condition for flip-left and flip-right
					{
						if (checkArrowRight > 2400)
						{
							arrowType = LEFT_ARROW;
							cout << "LEFT_ARROW" << endl;
						}
						else
						{
							arrowType = RIGHT_ARROW;
							cout << "RIGHT_ARROW" << endl;
						}
					}
					else
					{
						arrowType = NO_ARROW;
						//cout << "NO_ARROW"<< endl;
					}
				} // IF confirmed rectangle

			} // IF four corner polygon 
		} // IF large moments
	} // FOR contour loop
	return arrowType;
} /// findArrowPatch

/// input: Mat frame, flush ration: 0~1
/// return: Flush Frame 
void GenericVision::flushEdges(Mat &camFrame, double flushRatio)
{

	/// flush/cover left and right of the input image with black borders
	rectangle(camFrame, Point(0, 0), Point(camFrame.size().width * flushRatio, camFrame.size().height), Scalar(0, 0, 0),
						-1, 1);
	rectangle(camFrame, Point(camFrame.size().width * (1.0 - flushRatio), 0),
						Point(camFrame.size().width, camFrame.size().height), Scalar(0, 0, 0), -1, 1);
	/// flush/cover top and bottom of the input image with black borders
	rectangle(camFrame, Point(0, 0), Point(camFrame.size().width, camFrame.size().height* flushRatio), Scalar(0, 0, 0),
						-1, 1);
	rectangle(camFrame, Point(0, camFrame.size().height * (1.0 - flushRatio)),
						Point(camFrame.size().width, camFrame.size().height), Scalar(0, 0, 0), -1, 1);					
}


/// INPUT: top left corner x and y coordinates of the hsvThreshold screen to calculate the number of white pixels.
/*private int countWhitePixels(Mat hsvThreshold, int startX, int startY){
	int countWhite = 0;
	for( int y = 0; y < startY + 120; y++ ){
		for( int x = 0; x < startX + 160; x++ ){
			if( hsvThreshold.at<uchar>(y,x) != 0 ){
				countWhite++;
			}
		}
	}
	return countWhite;
}*/


/// input: main camera frame for marking, Thresholded HSV image, default value: minSize =200, maxSize = 10000, flushRatio = 0.1
/// return marker: (x,y,size)
Point3i GenericVision::findColouredObject(Mat &camFrame, Mat &hsvThreshold, Scalar colour, int minSize, int maxSize,
																					int minEdge, int maxEdge, double flushRatio/*0~1*/)
{
	Point3i marker;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<cv::Point> patchCorners;
	Mat canny, result;

	/// flush/crop left and right of the input image with black borders
	rectangle(hsvThreshold, Point(0, 0), Point(hsvThreshold.size().width * flushRatio, hsvThreshold.size().height), Scalar(0, 0, 0),
						-1, 1);
	rectangle(hsvThreshold, Point(hsvThreshold.size().width * (1.0 - flushRatio), 0),
						Point(hsvThreshold.size().width, hsvThreshold.size().height), Scalar(0, 0, 0), -1, 1);
	/// bottom and top
	rectangle(hsvThreshold, Point(0, 0), Point(hsvThreshold.size().width, hsvThreshold.size().height * flushRatio), Scalar(0, 0, 0),
						-1, 1);
	rectangle(hsvThreshold, Point(0, hsvThreshold.size().height * (1.0 - flushRatio)), Point(hsvThreshold.size().width, hsvThreshold.size().height), Scalar(0, 0, 0), -1, 1);

	/// closing operation to fill the black pixels
	Mat element = getStructuringElement(MORPH_RECT, Size(25, 25));
	morphologyEx(hsvThreshold, hsvThreshold, 3, element); /// Opening: MORPH_OPEN : 2 Closing: MORPH_CLOSE: 3
	Canny(hsvThreshold, canny, sbCannyLow, sbCannyHigh, 3);

	///Find all contours from canny edges
	findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	result = Mat::zeros(camFrame.size(), CV_8UC3);	 
	

	/// Get the moments
	vector<Moments> mu(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	/// Get the mass centers:
	vector<Point2f> mc(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	vector<Point> approxMarker;

	///Use contours to match polygons
	for (int i = 0; i < contours.size(); i++)
	{
		int patchSize = mu[i].m00;
		if (patchSize >= minSize && patchSize <= maxSize) /// moment area: if relatively large contour (frame) is detected
		{

			//drawContours(result, contours, i, CV_RGB(255,255,255 ), 1, 8, hierarchy, 0, cv::Point() );
			//approxPolyDP(contours[i], approxArrow, arcLength(Mat(contours[i]), true)*0.02, true);
			//convexHull( contours[i], approxArrow, false );
			//drawContours(result, approxArrow, i, CV_RGB(255,255,255 ), 1, 8, hierarchy, 0, cv::Point() );
			approxPolyDP(contours[i], approxMarker, arcLength(Mat(contours[i]), true) * 0.05, true);
			//for (int j = 0; j < approxArrow.size(); ++j)
			//cout << "Convex" << approxArrow.size() << endl;

			/// check number of corners
			if (approxMarker.size() >= minEdge && approxMarker.size() <= maxEdge)
			{

				drawContours(camFrame, contours, i, colour, 2, 8, hierarchy, 0, Point());

				marker = Point3i(mc[i].x, mc[i].y, patchSize);

			} /// IF number of edges is confirmed 

			else
			{
				marker = Point3i(-1, -1, -1);
			}

		} // IF large moments
	} // FOR contour loop
	return marker;
} /// findColourObject


int GenericVision::findNumColouredPixels(string scanDirection, bool isWalkingLeft, bool printFlag){
	int pixelsCount[8] = {0};
	int pixelsCountR[4] = {0};
	int pixelsCountEdge[2] = {0};
	int pixelsCountEdgeR[2] = {0};
	int stepStrategy = 0;
	if(printFlag)
	{
		cout <<endl;
	}
	
	for( int y = 0; y < this->threshold1Frame.rows; y++ ){
		for( int x = 0; x < this->threshold1Frame.cols; x++ ){
			if(  this->threshold1Frame.at<uchar>(y, x) != 0){
				if(printFlag&&x%10==0&&y%10==0){
					cout<<"1 ";
				}
				if( y <= 180 && x <= 160 ){
					pixelsCount[0]++;
				}
				else if( y <= 180 && x > 160 ){
					pixelsCount[1]++;
				}
				else if( y > 180 && x <= 160 ){
					pixelsCount[2]++;
				}
				else {
					pixelsCount[3]++;
				}
				//L/R edge pixels
				if( x > 0 && x <= 50 )
					pixelsCountEdge[0]++;
				else if(x > 270 && x <= 320 )
					pixelsCountEdge[1]++;
			}
			else if(  this->threshold2Frame.at<uchar>(y, x) != 0 ){
				if(printFlag&&x%10==0&&y%10==0){
					cout<<"2 ";
				}
				if( y <= 180 && x <= 160 ){
					pixelsCount[4]++;
				}
				else if( y <= 180 && x > 160 ){
					pixelsCount[5]++;
				}
				else if( y > 180 && x <= 160 ){
					pixelsCount[6]++;
				}
				else {
					pixelsCount[7]++;
				}
			}
			else if(  this->threshold3Frame.at<uchar>(y, x) != 0 ){
				if(printFlag&&x%10==0&&y%10==0){
					cout<<"3 ";
				}
				if( y <= 180 && x <= 160 ){
					pixelsCountR[0]++;
				}
				else if( y <= 180 && x > 160 ){
					pixelsCountR[1]++;
				}
				//L/R edge pixels
				if( x > 0 && x <= 50 )
					pixelsCountEdgeR[0]++;
				else if(x > 270 && x <= 320 )
					pixelsCountEdgeR[1]++;
			}
			else if(printFlag&&x%10==0&&y%10==0){
					cout<<"0 ";
			}
		}
		if(printFlag&&y%10==0){
					cout<<endl;
		}
	}
	
	if(printFlag){
		if(isWalkingLeft)
			cout << scanDirection << " walking left" << endl;
		else
			cout << scanDirection << " walking right" << endl;
		printf("pixelsCount[0]: %5d    %5d :pixelsCount[1]\n",pixelsCount[0],pixelsCount[1]);
		printf("pixelsCount[2]: %5d    %5d :pixelsCount[3]\n",pixelsCount[2],pixelsCount[3]);
		printf("edgeLeft[0]: %5d    %5d :edgeRight[1]\n",pixelsCountEdge[0],pixelsCountEdge[1]);
	}
	
	if( scanDirection == "RedScan"){
		if(pixelsCountR[0]+pixelsCountR[1] > 11000 && pixelsCountEdgeR[0] > 100 && pixelsCountEdgeR[1] >100 && pixelsCountEdge[0]<100 && pixelsCountEdge[1])
			stepStrategy = 0; //crawl
		else if((pixelsCountEdgeR[0] < 100 || pixelsCountEdge[0] >100) && pixelsCountR[1] > 5000)
			stepStrategy = 2; //right
		else if((pixelsCountEdgeR[1] < 100 || pixelsCountEdge[0] >100) && pixelsCountR[0] > 5000 )
			stepStrategy = 1; //left
		else
			stepStrategy = -1; //continue walking
	}
	else if( isWalkingLeft ){
		if( scanDirection == "BottomLeftScan" && pixelsCount[3] < 4000 ){
			//if TOO CLOSE
			if( pixelsCount[0] + pixelsCount[1] + pixelsCount[2] + pixelsCount[3] > 40000 )
				stepStrategy = 3;
				
	
			else if(pixelsCount[2] > 4000){
				stepStrategy = -2;
			}
			else{
				stepStrategy = 1;
			}
			//else if( pixelsCount[0] < 3000 && pixelsCount[1] < 3000 && pixelsCount[2] < 3000 && pixelsCount[3] < 3000 )
			//	stepStrategy = 1;
			//else if( pixelsCount[0] > 3000 && pixelsCount[2] > 3000 && pixelsCount[1] < 3000 && pixelsCount[3] < 3000 )
			//	stepStrategy = 1;
			//else if( pixelsCount[0] < 3000 && pixelsCount[2] < 3000 && pixelsCount[1] > 3000 && pixelsCount[3] > 3000 )
			//	stepStrategy = 1;
			//else
			//	stepStrategy = -2;
		} 
		else if(scanDirection == "BottomLeftScan") {
			stepStrategy = -2;
		}
	
		if( scanDirection == "BottomRightScan" && pixelsCount[2] < 4000 ){
			//if TOO CLOSE
			if( pixelsCount[0] + pixelsCount[1] + pixelsCount[2] + pixelsCount[3] > 40000 )
				stepStrategy = 3;
				
			else if(pixelsCount[3] > 4000 ){
				stepStrategy = 1;
			}
			else{
				stepStrategy = 2;
			}
			//if( pixelsCount[0] > 3000 && pixelsCount[1] > 3000 && pixelsCount[2] > 3000 && pixelsCount[3] > 3000 )
			//	stepStrategy = 3;
			//else if( pixelsCount[0] < 3000 && pixelsCount[1] < 3000 && pixelsCount[2] < 3000 && pixelsCount[3] < 3000 )
			//	stepStrategy = 2;
			//else if( pixelsCount[0] > 3000 && pixelsCount[2] > 3000 && pixelsCount[1] < 3000 && pixelsCount[3] < 3000 )
			//	stepStrategy = 2;
			//else if( pixelsCount[0] < 3000 && pixelsCount[2] < 3000 && pixelsCount[1] > 3000 && pixelsCount[3] > 3000 )
			//	stepStrategy = 2;
			//else
			//	stepStrategy = 0;
		} 
		else if( scanDirection == "BottomRightScan" ) {
			stepStrategy = 0;
		}
		
		if( scanDirection == "OnWalkingLeftScan" ){
			cout << "Pixels count: "  << pixelsCount[4] + pixelsCount[5] + pixelsCount[6] + pixelsCount[7] << endl;
			if( pixelsCount[4] + pixelsCount[5] + pixelsCount[6] + pixelsCount[7] >= 4000 )
				return 2;
			else
				return 1;
		}
	} 
	else {
		if( scanDirection == "BottomRightScan" && pixelsCount[2] < 4000 ){
			//if TOO CLOSE
			if( pixelsCount[0] + pixelsCount[1] + pixelsCount[2] + pixelsCount[3] > 40000 )
				stepStrategy = 3;
				
			else if(pixelsCount[3] > 4000){
				stepStrategy = 2;
			}
			else{
				stepStrategy = 1;
			}
			//if( pixelsCount[0] > 3000 && pixelsCount[1] > 3000 && pixelsCount[2] > 3000 && pixelsCount[3] > 3000 )
			//	stepStrategy = 3;
			//else if( pixelsCount[0] < 3000 && pixelsCount[1] < 3000 && pixelsCount[2] < 3000 && pixelsCount[3] < 3000 )
			//	stepStrategy = 2;
			//else if( pixelsCount[0] > 3000 && pixelsCount[2] > 3000 && pixelsCount[1] < 3000 && pixelsCount[3] < 3000 )
			//	stepStrategy = 2;
			//else if( pixelsCount[0] < 3000 && pixelsCount[2] < 3000 && pixelsCount[1] > 3000 && pixelsCount[3] > 3000 )
			//	stepStrategy = 2;
			//else
			//	stepStrategy = -1;
		}
		else if( scanDirection == "BottomRightScan" ) {
			stepStrategy = -1;
		}
		
		if( scanDirection == "BottomLeftScan" && pixelsCount[3] < 3000 ){
			//if TOO CLOSE
			if( pixelsCount[0] + pixelsCount[1] + pixelsCount[2] + pixelsCount[3] > 40000 )
				stepStrategy = 3;
				
	
			else if(pixelsCount[2] > 4000){
				stepStrategy = 2;
			}
			else{
				stepStrategy = 1;
			}
			
			//if( pixelsCount[0] > 3000 && pixelsCount[1] > 3000 && pixelsCount[2] > 3000 && pixelsCount[3] > 3000 )
			//	stepStrategy = 3;
			//else if( pixelsCount[0] < 3000 && pixelsCount[1] < 3000 && pixelsCount[2] < 3000 && pixelsCount[3] < 3000 )
			//	stepStrategy = 1;
			//else if( pixelsCount[0] > 3000 && pixelsCount[2] > 3000 && pixelsCount[1] < 3000 && pixelsCount[3] < 3000 )
			//	stepStrategy = 1;
			//else if( pixelsCount[0] < 3000 && pixelsCount[2] < 3000 && pixelsCount[1] > 3000 && pixelsCount[3] > 3000 )
			//	stepStrategy = 1;
			//else
			//	stepStrategy = 0;
		} 
		else if(scanDirection == "BottomLeftScan") {
			stepStrategy = 0;
		}
		if( scanDirection == "OnWalkingRightScan" ){
			cout << "Pixels count: "  << pixelsCount[4] + pixelsCount[5] + pixelsCount[6] + pixelsCount[7] << endl;
			if( pixelsCount[4] + pixelsCount[5] + pixelsCount[6] + pixelsCount[7] >= 4000 )
				return 1;
			else
				return 2;
		}
	}
	
	if( scanDirection == "BottomForwardScan" ){
		if(pixelsCount[0] + pixelsCount[1] + pixelsCount[2] + pixelsCount[3] > 40000 )
		//if( pixelsCount[0] > 5000 && pixelsCount[1] > 5000 && (pixelsCount[2] > 4000 || pixelsCount[3] > 4000) )
			stepStrategy = 3;
		else if( pixelsCountEdge[0] < 100)
			stepStrategy = 1;
		else if( pixelsCountEdge[1] < 100)
			stepStrategy = 2;
		else
			stepStrategy = 1;
	}	

	cout << "StepS: " << stepStrategy << "\n\n"<<endl;
	return stepStrategy;
}




double GenericVision::getAngle(Mat &camFrame, Mat &hsvThreshold1)
{
	double ret = 0;
	int oneLine[32] = {0};
	int numX =0;
	int temp[32] = {0};
	int numTemp;
	int zeroTohalf=0;
	int halfToAll=0;
	for( int y = 0; y < this->threshold1Frame.rows; y++ ){
		for( int x = 0; x < this->threshold1Frame.cols; x++ ){
			if(  this->threshold1Frame.at<uchar>(y, x) != 0){
				if(x%10==0&&y%10==0){
					oneLine[x]++;
					numX++;
				}
			}
		}
		if(oneLine>0){
			for(int k =0; k<32 ; k++){
				temp[k] = oneLine[k];
			}
			numTemp = numX;
			oneLine[32] = {0};
			numX = 0;
		}else{
			break;
		}
	}
	if(numTemp<29){
		for(int k =0; k<32 ; k++){
			if(temp[k]>0 && k<17)
				zeroTohalf++;
			else if(temp[k]>0)
				halfToAll++;
		}
		if(zeroTohalf>halfToAll)
			ret = 1;
		else
			ret = 2;
		
	}
	return ret;
	/*Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	findContours(hsvThreshold1 , contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
	vector<RotatedRect> minRect(contours.size());
	for (size_t i = 0; i < contours.size(); i++)
	{
		if(contours[i].size() >= 5)
			minRect[i] = fitEllipse(contours[i]);
	}
	float size = 0;
	size_t pos = -1;
	for (size_t i = 0; i< contours.size(); i++)
	{
		if (minRect[i].size.width * minRect[i].size.height > size)
		{
			size = minRect[i].size.width * minRect[i].size.height;
			pos = i;
		}
		Point2f rect[4];
		minRect[i].points(rect);
		for(int j = 0; j < 4; j++)
		{
			line(camFrame, rect[j],rect[(j+1)%4], Scalar(0,0,0),1,8);
		}
		imshow("bgred", camFrame);
		imshow("bgred123", hsvThreshold1);
	}
	
	double myContourAngle = 900;
	if (pos >= 0 && contours.size() > 0)
	{
		myContourAngle = minRect[pos].angle;
		if (minRect[pos].size.width < minRect[pos].size.height) {
			myContourAngle = myContourAngle - 90;
		}
	}
	
	return myContourAngle;*/
	
}
int GenericVision::EvalNumColouredPixels(string scanDirection, bool isWalkingLeft){
	
	return 0;
}
