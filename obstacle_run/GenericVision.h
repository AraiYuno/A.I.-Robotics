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

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <iomanip>      // std::setprecision
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */

using namespace std;
using namespace cv;

#define INVALID_VALUE_H   180
#define INVALID_VALUE_SV   255

#define NO_ARROW 0
#define FORWARD_ARROW 1
#define LEFT_ARROW 2
#define RIGHT_ARROW 3

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

		int findNumColouredPixels(string scanDirection, bool isWalkingLeft, bool printFlag);
		int EvalNumColouredPixels(string scanDirection, bool isWalkingLeft);
};

#endif /* GENERICVISION_H_ */
