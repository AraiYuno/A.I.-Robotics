#include "GenericVision.h"

// Tuning values for field extraction
//====================================================================================================
int field_h_Low = 30; int field_h_High = 84; int field_s_Low = 0; int field_s_High = 150;
int field_v_Low = 0; int field_v_High = 91;
int line_h_Low = 118; int line_h_High = 255; int line_s_Low = 114; int line_s_High = 255;
int line_v_Low = 58; int line_v_High = 255;
void updateCanny_Low(int, void* ){}
void updateCanny_High(int, void* ){}
//====================================================================================================

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
	/*namedWindow("Colour1", CV_WINDOW_AUTOSIZE);
	namedWindow("Colour2", CV_WINDOW_AUTOSIZE);
	namedWindow("Colour3", CV_WINDOW_AUTOSIZE);
	namedWindow("Debug", CV_WINDOW_AUTOSIZE);
	createTrackbar("Marker:", "Control", &marker_xml, 3, markerTrackbarCbFunc);
	createTrackbar("Canny Threshold (Low):", "Control", &sbCannyLow, 255);
	createTrackbar("Canny Threshold (High):", "Control", &sbCannyHigh, 255);
	createTrackbar("H (Gain):", "Control", &sbHSVGain[0], 60, hTrackbarCbFunc);
	createTrackbar("S (Gain):", "Control", &sbHSVGain[1], 100, sTrackbarCbFunc);
	createTrackbar("V (Gain):", "Control", &sbHSVGain[2], 100, vTrackbarCbFunc);
	markerTrackbarCbFunc(0, NULL);*/
}

void GenericVision::showGUI()
{
	imshow("Control", rawFrame);
	//imshow("Colour1", threshold1Frame);
	// imshow("Colour2", threshold2Frame);
	// imshow("Colour3", threshold3Frame);
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



int GenericVision::getMoveStrategy()
{
	int topLeft =0;
	int bottomLeft = 0;
	int topCenter = 0;
	int bottomCenter = 0;
	int topRight = 0;
	int bottomRight = 0;
	
	int step = 0;
	
	for(int y = 0; y < this->threshold1Frame.rows; y++)
	{
		for(int x = 0; x < this->threshold1Frame.cols; x++)
		{
				if(x<=100 && y <= 80)
					topLeft++;
				else if( x <=100 && y > 80)
					bottomLeft++;
				else if( (x >100 && x <=220) && y <= 80)
					topCenter++;
				else if( (x >100 && x <=220) && y > 80)
					bottomCenter++;
				else if( x > 220 && y <= 80)
					topRight++;
				else if( x > 220 && y > 80)
					bottomRight++;

		}	
	}
	
	if(bottomCenter != 0)
	{
		step = 2 ;// back
	}
	else if(topCenter!= 0)
	{
		if(topLeft !=0)
			step = 4;
		else if(topRight != 0)
			step = 3;
	}
	else if(topCenter == 0)
	{
		step = 1;
	}
	
	return step;
}

//======================================================================================
// Author: Kyle Ahn
//   From here and below, functions used for feature detection and vision part of  the
//   localisation is implemented.
//======================================================================================
vector<string> GenericVision::detectFeature( float (&visionMap)[28][19]){
	Mat field;
	original = rawFrame.clone();
	extractedImg = original.clone(); 
	resize(original, original, Size(300,300), 0,0,1);
	resize(extractedImg, extractedImg, Size(300,300), 0,0,1);
	extractField(extractedImg, field);
	//cout << "Before drawfield ExtractField" << endl;
	vector<string> detectedFeatures = drawField(field);
	//cout << "After drawField" << endl;
	imshow("Field Control",field);
	return detectedFeatures;
}


void GenericVision::calcVisionMap(float (&visionMap)[28][19]){

}

void GenericVision::calcVisionMapDegree(int (&visionMapDegree)[28][19]){
}



//==========================================================================================
// Helper functions for feature detection of A1
//==========================================================================================
void GenericVision::extractField(Mat &img, Mat &field)
{
	/// GUI with trackbar
	namedWindow( "Field Control", CV_WINDOW_AUTOSIZE );
	createTrackbar( "h (Low):", "Field Control", &field_h_Low, 255, updateCanny_Low);
	createTrackbar( "h (High):", "Field Control", &field_h_High, 255, updateCanny_High);
	createTrackbar( "s (Low):", "Field Control", &field_s_Low, 255, updateCanny_Low);
	createTrackbar( "s (High):", "Field Control", &field_s_High, 255, updateCanny_High);
	createTrackbar( "v (Low):", "Field Control", &field_v_Low, 255, updateCanny_Low);
	createTrackbar( "v (High):", "Field Control", &field_v_High, 255, updateCanny_High);

	Mat hsv_threshold;

	inRange(img, cv::Scalar(field_h_Low, field_s_Low, field_v_Low), cv::Scalar(field_h_High, field_s_High, field_v_High), hsv_threshold);
	
	Mat element = getStructuringElement(MORPH_RECT,Size(50,50), Point(1,1));
	morphologyEx(hsv_threshold, hsv_threshold, MORPH_CLOSE , element);

	Mat canny;

	Mat result = original.clone();
	Mat out;
	result.copyTo(out, hsv_threshold);

	field = out;
}


vector<string> GenericVision::drawField(Mat &hsv_img){
	namedWindow( "Line Control", CV_WINDOW_AUTOSIZE );
	createTrackbar( "h (Low):", "Line Control", &line_h_Low, 255, updateCanny_Low);
	createTrackbar( "h (High):", "Line Control", &line_h_High, 255, updateCanny_High);
	createTrackbar( "s (Low):", "Line Control", &line_s_Low, 255, updateCanny_Low);
	createTrackbar( "s (High):", "Line Control", &line_s_High, 255, updateCanny_High);
	createTrackbar( "v (Low):", "Line Control", &line_v_Low, 255, updateCanny_Low);
	createTrackbar( "v (High):", "Line Control", &line_v_High, 255, updateCanny_High);
	
	medianBlur(hsv_img, hsv_img, 5);
	Mat cann, img;
	inRange(hsv_img, cv::Scalar(line_h_Low, line_s_Low, line_v_Low), cv::Scalar(line_h_High, line_s_High, line_v_High), cann);
	bitwise_not ( cann, img );
	imshow("Line Control", img);
	resize(img, img, Size(300,300),0,0,CV_INTER_LINEAR);
	Mat mask = Mat::ones( img.size(), CV_8UC1 );
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
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
	vector<string> detectedFeatures;
	sortKeyLines(keyLines);
	cv::Mat output = img.clone();
	if( output.channels() == 1 )
		cvtColor( output, output, COLOR_GRAY2BGR );
	vector<KeyLine> mergedLines, tCornerLines, tCornerLCornerLines, normalLines, circleLineCorners, centreCircleLines;
	cout << "before cleanup " << endl;
	cleanUpLines( keyLines, mergedLines);
	cout << "after cleanup" << endl;
	detectCentreCircleLines( mergedLines, centreCircleLines );
	if( centreCircleLines.size() <= 5 ){
		string goalLineFeature = detectGoalLine( mergedLines, tCornerLines, normalLines );
		if( goalLineFeature.compare("") != 0 )
			detectedFeatures.push_back(goalLineFeature);
	}
	if( centreCircleLines.size() > 5 )
		detectCircleTCorner( mergedLines, circleLineCorners, true ); 
	else
		detectCircleTCorner( mergedLines, circleLineCorners, false ); 
	drawLines(output, mergedLines, 2);
	
	// if there exists a centre  circle in the field, we draw a centre circle in our output.
	// TODO: I need to be able to return a combination of different strings depending on the detected features.
	if( centreCircleLines.size() > 5 ){
		drawCentreCircle(output, centreCircleLines);
		detectedFeatures.push_back("centreCircle");
	}
	if( tCornerLines.size() >= 1 ){
		drawLines(output, tCornerLines, 3);
	}
	drawLines(output, circleLineCorners, 0);
	imshow("LSD", output);
	return detectedFeatures;
}



/* Summary: cleanUpLines is the main merging function which merges all the colinear and parallel lines detected by the LSD function. 
*/
void GenericVision::cleanUpLines( vector<KeyLine> &lines, vector<KeyLine> &mergedLines){
	vector<KeyLine> tempLines;
	bool mainLineIsSelected;
	// in this do_while loop, we join all the colinear lines.
	do {
		mainLineIsSelected = false;
		if( tempLines.size() > 1 ){
			lines = tempLines;
			tempLines = vector<KeyLine>();
		}
		KeyLine mainLine;
		int size = lines.size();
		for( int i = 0; i < size; i++ ){
			KeyLine line = lines[i];
			if( calcLineLength(line) > 20 ){
				if( !mainLineIsSelected ){
					mainLine = line;
					mainLineIsSelected = true;
					cout << "X: " << mainLine.startPointX << endl;
				} else {
					switchStartAndEnd(line);
					switchStartAndEnd(mainLine); // to make sure that startPointX is the smaller value than endPointX at all times.
					if( areSameLines(mainLine, line) ){ // we compare the mainLine and kl to see if we need to merge.
						mergeLines(mainLine, line);
					} else {
						tempLines.push_back(lines[i]);
					}	
				}
			} 
		}
		mergedLines.push_back(mainLine);
	} while(tempLines.size() > 1);

	// to make sure startPointX is always the smaller value than endPointX.
	for( int i = 0; i < mergedLines.size(); i++ ){
		switchStartAndEnd(mergedLines[i]);
	}
}


/* Summary: drawLines simply draws lines.
 */
void GenericVision::drawLines( Mat &output, vector<KeyLine> &keyLines, int colour){
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


void GenericVision::drawCentreCircle( Mat &output, vector<KeyLine> centreCircleLines ){
	float minX = 1000, minY = 1000, maxX = -1000, maxY = -1000;
	for( int i = 0; i < centreCircleLines.size(); i++ ){
		minX = (minX > centreCircleLines[i].startPointX) ? centreCircleLines[i].startPointX : minX;
		minY = (minY > centreCircleLines[i].startPointY) ? centreCircleLines[i].startPointY : minY;
		maxX = (maxX < centreCircleLines[i].startPointX) ? centreCircleLines[i].startPointX : maxX;
		maxY = (maxY < centreCircleLines[i].startPointX) ? centreCircleLines[i].startPointX : maxY;
	}
	int radius = (maxX - minX > maxY - minY) ? maxX - minX : maxY - minY;
	circle(output, Point((minX + maxX)/2, (minY + maxY)/2), std::abs(radius/2), Scalar(0, 255, 255), 2);
}


/* If two lines are near to one another, and have similar angles, we consider
them to be the colinear lines to be merged. */
bool GenericVision::areSameLines(KeyLine &kl1, KeyLine &kl2){
	float distance = calcDistance(&kl1, &kl2);
	float angleDifference = std::abs(calcAngleBetweenTwoLines(kl1, kl2));
	return (distance < 33.f  && angleDifference < 15.f) ? true : false;
}


float GenericVision::calcAngleBetweenTwoLines(KeyLine &kl1, KeyLine &kl2){
	float slopeKl1 = (kl1.endPointY - kl1.startPointY)/(kl1.endPointX - kl1.startPointX);
	float slopeKl2 = (kl2.endPointY - kl2.startPointY)/(kl2.endPointX - kl2.startPointX);
	float tanTheta = (slopeKl1 - slopeKl2)/(1 + slopeKl1*slopeKl2);
	return std::atan(tanTheta) * (180.0/3.141592653589793238463);
}


/* Summary: since the lines are already sorted by x in ascending order, we can simply extend mainLine
by setting its endPoint to kl's endPoint. */
void GenericVision::mergeLines(KeyLine &mainLine, KeyLine &kl){
	mainLine.endPointX = kl.endPointX;
	mainLine.endPointY = kl.endPointY;
}


/* Summary: calcDistance calculates and returns the minimum distance amongst the combinations of 4 points (2 line segments)*/
float GenericVision::calcDistance(KeyLine *kl1, KeyLine *kl2){
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
float GenericVision::calcAngle(KeyLine &kl){
	float angle;
	angle = atan2(kl.startPointY, kl.startPointX)  - atan2(kl.endPointY, kl.endPointX);
	angle = angle * 360.0f / (2*PI);
	if( angle < 0 )
		angle += 360;
	return (angle > 180.0f) ? angle - 180.0f : angle;
}


/* Summary: sortKeyLines simply sorts the given keyLines by x in ascending order using startPointX. 
If the startPointX is larger than its endPointX, we simply switch the extreme the other way.*/
void GenericVision::sortKeyLines(vector<KeyLine> &keyLines){
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
void GenericVision::switchStartAndEnd(KeyLine &kl){
	if( kl.startPointX > kl.endPointX ){
		float tempX = kl.startPointX;
		float tempY = kl.startPointY;
		kl.startPointX = kl.endPointX;
		kl.endPointX = tempX;
		kl.startPointY = kl.endPointY;
		kl.endPointY = tempY;
	}
}


void GenericVision::detectCentreCircleLines(vector<KeyLine> &lines, vector<KeyLine> &centreCircleLines){
	int linesSize = lines.size();
	for( int i = 0; i < linesSize - 1; i++ ){
		if( calcLineLength(lines[i]) <= 60 ){
			KeyLine mainLine = lines[i];
			for( int j = i + 1; j < linesSize; j++ ){
				KeyLine toCompare = lines[j];
				if( calcAngleBetweenTwoLines(mainLine, toCompare) <= 30 && calcDistance(&mainLine, &toCompare) <= 35 ){
					centreCircleLines.push_back(mainLine);
					break;					
				}
			}
		}
	}

	vector<KeyLine> newMergedLines;
	for( int i = 0; i < lines.size(); i++ ){
		bool lineExists = false;
		for( int j = 0; j < centreCircleLines.size(); j++ ){
			if( centreCircleLines[j].startPointX == lines[i].startPointX && centreCircleLines[j].startPointY == lines[i].startPointY &&
				centreCircleLines[j].endPointX == lines[i].endPointX && centreCircleLines[j].endPointX == lines[i].endPointX){
				lineExists = true;
				break;
			}
		}
		if( !lineExists ){
			newMergedLines.push_back(lines[i]);
		}
	}
	lines = newMergedLines;
}


/* Summary: detectLCorners detect all the L corners using intersection points, and distance between extremes. */
void GenericVision::detectLCorners(vector<KeyLine> lines, vector<KeyLine> &cornerLines){
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
bool GenericVision::isLCorner(KeyLine &kl1, KeyLine &kl2){
	bool isLCorner = false;
	float angleDifference = std::abs(calcAngleBetweenTwoLines(kl1, kl2));
	if( angleDifference > 50 ){
		Point kl1StartPt = Point( kl1.startPointX, kl1.startPointY);
		Point kl1EndPt = Point( kl1.endPointX, kl1.endPointY);
		Point kl2StartPt = Point( kl2.startPointX, kl2.startPointY);
		Point kl2EndPt = Point( kl2.endPointX, kl2.endPointY);
		// if two lines intersect when the angle > 50, then we check each extremes of the line 
		// to see if the lines are intersecting as T.
		if( doIntersect(kl1StartPt, kl1EndPt, kl2StartPt, kl2EndPt) ){
			float distance = calcDistance(&kl1, &kl2); 
			if( distance <= 35.0f )
				isLCorner = true;
		}
	}
	return isLCorner;
}


/* Summary: detectCircleTCorner detects the centre line T corner. The idea is that if there no L corner present 
on the field other, then we consider the T corner as centire line T corner. (can be updated using robot's position in 
the future.) */
void GenericVision::detectCircleTCorner(vector<KeyLine> &mergedLines, vector<KeyLine> &cornerLines, bool circleExists){
	if( !circleExists ){
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
	} else {
		float maxLength = calcLineLength(mergedLines[0]);
		int index = 0;
		for( int i = 1; i < mergedLines.size(); i++ ){
			int currLineLength =  calcLineLength(mergedLines[i]);
			if(currLineLength > maxLength){
				maxLength = currLineLength;
				index = i;
			}
		}
		if( maxLength > 100 ){
			cornerLines.push_back(mergedLines[index]);
		}
	}
}


/* Summary: checkTIntersection simply checks if there exists a L corner that is connected to the goal line T corner */
KeyLine GenericVision::checkTIntersection(KeyLine kl1, KeyLine kl2, Point intersectionPoint){
	float kl1StartPtDist = sqrt( pow( kl1.startPointX - intersectionPoint.x, 2 ) + pow(kl1.startPointY - intersectionPoint.y, 2));
	float kl1EndPtDist = sqrt( pow( kl1.endPointX - intersectionPoint.x, 2 ) + pow(kl1.endPointY - intersectionPoint.y, 2));
	float kl2StartPtDist = sqrt( pow( kl2.startPointX - intersectionPoint.x, 2 ) + pow(kl2.startPointY - intersectionPoint.y, 2));
	float kl2EndPtDist = sqrt( pow( kl2.endPointX - intersectionPoint.x, 2 ) + pow(kl2.endPointY - intersectionPoint.y, 2));
	float kl1Dist = (kl1StartPtDist < kl1EndPtDist) ? kl1StartPtDist : kl1EndPtDist;
	float kl2Dist = (kl2StartPtDist < kl2EndPtDist) ? kl2StartPtDist : kl2EndPtDist;
	return (kl1Dist < kl2Dist) ? kl1 : kl2;
}


/* detectGoalLine: detects T corners on the field by looking at the distance between extremes and intersection point as well as the angle
				   returns whatever feature is detected.
 */
string GenericVision::detectGoalLine(vector<KeyLine> &mergedLines, vector<KeyLine> &cornerLines, vector<KeyLine> &normalLines){
	string feature = "";
	int mergedLinesSize = mergedLines.size();
	for( int i = 0; i < mergedLinesSize-1; i++ ){
		bool twoParallelLinesExist = false;
		KeyLine mainLine = mergedLines[i];
		for( int j = i + 1; j < mergedLinesSize; j++ ){
			KeyLine toCompare = mergedLines[j];
			if( isTCorner( mainLine, toCompare ) ){
				KeyLine tCornerLine = checkTIntersection(mainLine, toCompare, getIntersectionPoint( mainLine, toCompare ));
				cornerLines.push_back(tCornerLine);

				for( int k = 0; k < mergedLinesSize; k++ ){
					if( isLCorner(tCornerLine, mergedLines[k]) && calcLineLength(mergedLines[k]) >= 30.0f )  {
						cornerLines.push_back(mergedLines[k]);
						feature = "TCornerAtGoalLine";
					}
				}
			}
		}
	}

	if( cornerLines.size() < 2 ){
		for(int i = 0; i < mergedLines.size(); i++ ){
			KeyLine mainLine = mergedLines[i];
			for( int j = i + 1; j < mergedLines.size(); j++ ){
				KeyLine toCompare = mergedLines[j];
				if( calcAngleBetweenTwoLines(mainLine, toCompare) < 20 && calcLineLength(mainLine) > 100 && calcLineLength(toCompare) > 100 ){
					cornerLines = vector<KeyLine>();
					cornerLines.push_back(mainLine);
					cornerLines.push_back(toCompare);
					feature = "parallelGoalLines";
					break;
				}
			}
		}
	}
	return feature;
}


/* Summary: isTCorner simply returns true if the two lines are T corner related, false otherwise. */ 
bool GenericVision::isTCorner(KeyLine &kl1, KeyLine &kl2){
	bool isTCorner = false;
	float angleDifference = std::abs(calcAngleBetweenTwoLines(kl1, kl2));
	if( angleDifference > 50 ){
		Point kl1StartPt = Point( kl1.startPointX, kl1.startPointY);
		Point kl1EndPt = Point( kl1.endPointX, kl1.endPointY);
		Point kl2StartPt = Point( kl2.startPointX, kl2.startPointY);
		Point kl2EndPt = Point( kl2.endPointX, kl2.endPointY);
	
		// if two lines intersect when the angle > 50, then we check each extremes of the line 
		// to see if the lines are intersecting as T.
		if( doIntersect(kl1StartPt, kl1EndPt, kl2StartPt, kl2EndPt) ){
			float distance = calcDistance(&kl1, &kl2);
			if( distance > 10.0f ){
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
int GenericVision::orientation(Point p, Point q, Point r) 
{ 
    // for details of below formula. 
    int val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 


// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool GenericVision::onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
       return true; 
  
    return false; 
} 


// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool GenericVision::doIntersect(Point p1, Point q1, Point p2, Point q2) 
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
Point GenericVision::getIntersectionPoint(KeyLine &kl1, KeyLine &kl2 ){
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


float GenericVision::calcLineLength(KeyLine &kl){
	return std::sqrt( std::pow(kl.endPointX - kl.startPointX, 2) + std::pow(kl.endPointY - kl.startPointY, 2));
}


void GenericVision::printLines(std::vector<KeyLine> lines){
	cout << "Number of Lines: " << lines.size() << endl;
	for( int i = 0; i < lines.size(); i++ ){
		cout << "Start( " << lines[i].startPointX << ", " << lines[i].startPointY <<") - End(" << lines[i].endPointX << ", " << lines[i].endPointY << ")" << endl;
		cout << "   -> Line Length: " << calcLineLength(lines[i]) << endl;
	}
	cout << endl;
}


void GenericVision::printTwoLines(KeyLine &mainLine, KeyLine &line){
	cout << "--------------------------------------------------------------------------" << endl;
	cout << "MainLine" << endl;
	cout << "	->Start( " << mainLine.startPointX << ", " << mainLine.startPointY <<") - End(" << mainLine.endPointX << ", " << mainLine.endPointY << ")" << endl;
	cout << "   -> Line Length: " << calcLineLength(mainLine) << endl;
	cout << "toCompare" << endl;
	cout << "	->Start( " << line.startPointX << ", " << line.startPointY <<") - End(" << line.endPointX << ", " << line.endPointY << ")" << endl;
	cout << "   -> Line Length: " << calcLineLength(line) << endl;
	cout << ":: Distance: " << calcDistance(&mainLine, &line) << endl;;
	cout << ":: angle Difference: " << calcAngleBetweenTwoLines(mainLine, line) << endl;
	cout << "--------------------------------------------------------------------------" << endl;
	cout << endl;
}


