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
