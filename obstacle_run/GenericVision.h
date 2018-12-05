/*
 * GenericVision.h
 * Created on: June 20 2017
 * Author: MengCheng Lau
 */

#ifndef GENERICVISION_H_
#define GENERICVISION_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/line_descriptor.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <iomanip>      // std::setprecision
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */

using namespace std;
using namespace cv;
using namespace cv::line_descriptor;

#define INVALID_VALUE_H   180
#define INVALID_VALUE_SV   255

#define NO_ARROW 0
#define FORWARD_ARROW 1
#define LEFT_ARROW 2
#define RIGHT_ARROW 3
#define PI 3.14159265358979323846

class GenericVision
{
public:
		
		Mat rawFrame;
		Mat selectFrame;
		Mat threshold1Frame;
		Mat threshold2Frame;
		Mat threshold3Frame;
		Vec3i sbHSVGain;
		int sbCannyLow;
		int sbCannyHigh;
		Point2f mousePt1;
		Point2f mousePt2;

		GenericVision(void);

		GenericVision(Mat m_rawFrame, Mat m_hsvFrame);

		~GenericVision(void);

		static void CallBackFunc(int event, int x, int y, int flags, void *userdata);

		double getDist(Point p0, Point p1);

		

		void initGUI(void);

		void showGUI(void);

		void saveHSVAvg(const char *filename, Vec3i hsvAvg, Vec3i hsvGain);

		void readHSVAvg(const char *filename, Vec3i &hsvAvg, Vec3i &hsvGain);

		void saveCannyThresholds(const char *filename, Vec3i minHSV, Vec3i maxHSV);

		void readCannyThresholds(const char *filename, Vec3i &minHSV, Vec3i &maxHSV);

		Point3i findBall(Mat &camFrame, Mat &hsvThreshold, int minSize, int maxSize, double flushRatio/*0~1*/);

		Point3i findMixColouredObject(Mat &camFrame, Mat &hsvThreshold1, Mat &hsvThreshold2, Scalar colour, int minSize,
																	int maxSize, int minEdge, int maxEdge, double flushRatio);
		double getAngle(Mat &camFrame,Mat &hsvThreshold1);
		int findArrowPatch(Mat &camFrame, int minSize, int maxSize, double flushRatio/*0~1*/);

		void flushEdges(Mat &camFrame, double flushRatio);

		Point3i findColouredObject(Mat &camFrame, Mat &hsvThreshold, Scalar colour, int minSize, int maxSize, int minEdge,
															 int maxEdge, double flushRatio);
													
		vector<string> detectFeature( float (&visionMap)[28][19] );
		int getMoveStrategy();
		// For Feature detection from A1
private:
	Mat original, extractedImg;
	void calcVisionMap(float (&visionMap)[28][19]);
	void calcVisionMapDegree(int (&visionMapDegree)[28][19]);
	void extractField(Mat &img, Mat &field);
	vector<string> drawField(Mat &hsv_img);
	void cleanUpLines( vector<KeyLine> &lines, vector<KeyLine> &mergedLines);
	void drawLines( Mat &output, vector<KeyLine> &keyLines, int colour);
	void drawCentreCircle( Mat &output, vector<KeyLine> centreCircleLines );
	bool areSameLines(KeyLine &kl1, KeyLine &kl2);
	void mergeLines(KeyLine &mainLine, KeyLine &kl);
	void switchStartAndEnd(KeyLine &kl);
	void detectCentreCircleLines(vector<KeyLine> &lines, vector<KeyLine> &centreCircleLines);
	void detectLCorners(vector<KeyLine> lines, vector<KeyLine> &cornerLines);
	bool isLCorner(KeyLine &kl1, KeyLine &kl2);
	void detectCircleTCorner(vector<KeyLine> &mergedLines, vector<KeyLine> &cornerLines, bool circleExists);
	string detectGoalLine(vector<KeyLine> &mergedLines, vector<KeyLine> &cornerLines, vector<KeyLine> &normalLines);
	bool isTCorner(KeyLine &kl1, KeyLine &kl2);

	//Miscellaneous calculations
	float calcAngleBetweenTwoLines(KeyLine &kl1, KeyLine &kl2);
	float calcDistance(KeyLine *kl1, KeyLine *kl2);
	float calcAngle(KeyLine &kl);
	float calcLineLength(KeyLine &kl);
	void sortKeyLines(vector<KeyLine> &keyLines);
	KeyLine checkTIntersection(KeyLine kl1, KeyLine kl2, Point intersectionPoint);
	int orientation(Point p, Point q, Point r);
	bool onSegment(Point p, Point q, Point r);
	bool doIntersect(Point p1, Point q1, Point p2, Point q2);
	Point getIntersectionPoint(KeyLine &kl1, KeyLine &kl2 );
	void printLines(std::vector<KeyLine> lines);
	void printTwoLines(KeyLine &mainLine, KeyLine &line);
};

#endif /* GENERICVISION_H_ */
