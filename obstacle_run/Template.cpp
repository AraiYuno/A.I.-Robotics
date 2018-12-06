#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include <cv.h>
#include <math.h>
#include <algorithm>
#include <iterator>
#include <queue>
#include <sys/time.h>
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
#include "HelperFunctions.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH      "config_forward.ini"
#define INI_FILE_PATH2      "config_backward.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

#define SCRIPT_FILE_PATH    "script.asc"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

using namespace std;
using namespace cv;

struct Tuple
{
    float pd;
    int x;
    int y;

    Tuple(float n1, int n2, int n3) : pd(n1), x(n2), y(n3)
    {
    }

    bool operator<(const struct Tuple& other) const
    {
        return pd < other.pd;
    }
};

struct PointXY
{
    int x;
    int y;

    PointXY(int n1, int n2) : x(n1), y(n2)
    {
    }

};


//----------------------------------------------------------------------------------------------------------------
// Hard coded vision map. Will be used depending on the feature detected.
//----------------------------------------------------------------------------------------------------------------
//  84 X 57??
float goalLineLCornerMap[COL][ROW] = {   
    //  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//0
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//1
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//2
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//3
    {   0,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   0},//4
    {   0,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   0},//5
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//6
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//7
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//8
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//9
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//10
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//11
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//12
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//13
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//14
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//15
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//16
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//17
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//19
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//20
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//21
    {   0,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   0},//22
    {   0,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   0},//23
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//24
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//25
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//26
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0} //27
};
int goalLineLCornerMapDegree[COL][ROW] = 
{   
    //  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//0
    {   0, -45, -45,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 225, 225,   0},//1
    {   0, -45, -45,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 225, 225,   0},//2
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 180, 180,   0},//3
    {   0,  45,  45,  90, 135, 135,   0,   0,   0,   0,   0,   0,   0,  45,  45,  90, 135, 135,   0},//4
    {   0,  45,  45,  90, 135, 135,   0,   0,   0,   0,   0,   0,   0,  45,  45,  90, 135, 135,   0},//5
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//6
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//7
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//8
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//9
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//10
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//11
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//12
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//13
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//14
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//15
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//16
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//17
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//19
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//20
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//21
    {   0, -45, -45, 270, 225, 225,   0,   0,   0,   0,   0,   0,   0, -45, -45, 270, 225, 225,   0},//22
    {   0, -45, -45, 270, 225, 225,   0,   0,   0,   0,   0,   0,   0, -45, -45, 270, 225, 225,   0},//23
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 180, 180,   0},//24
    {   0,  45,  45,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 135, 135,   0},//25
    {   0,  45,  45,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 135, 135,   0},//26
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0} //27
};

//center
float centreCircleMap[COL][ROW] = {   
    //  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//0
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//1
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//2
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//3
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//4
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//5
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//6
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//7
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//8
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//9
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//10
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//11
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//12
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//13
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//14
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//15
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//16
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//17
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0},//18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//19
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//20
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//21
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//22
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//23
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//24
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//25
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//26
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0} //27
};
int centreCircleMapDegree[COL][ROW] = 
{   
    //  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//0
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//1
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//2
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//3
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//4
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//5
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//6
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//7
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//8
    {   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   3,   3,   3,   3,   0,   0,   0,   0,   0},//9
    {   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   3,   3,   3,   3,   0,   0,   0,   0,   0},//10
    {   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   3,   3,   3,   3,   0,   0,   0,   0,   0},//11
    {   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   3,   3,   3,   3,   0,   0,   0,   0,   0},//12
    {   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   3,   3,   3,   3,   0,   0,   0,   0,   0},//13
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   4,   4,   4,   4,   0,   0,   0,   0,   0},//14
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   4,   4,   4,   4,   0,   0,   0,   0,   0},//15
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   4,   4,   4,   4,   0,   0,   0,   0,   0},//16
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   4,   4,   4,   4,   0,   0,   0,   0,   0},//17
    {   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   4,   4,   4,   4,   0,   0,   0,   0,   0},//18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//19
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//20
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//21
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//22
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//23
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//24
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//25
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//26
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0} //27
};

//single line
float singleLineMap[COL][ROW] = {   
    //  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//0
    {   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0},//1
    {   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0},//2
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//3
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//4
    {   0,   1,   1,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   1,   1,   0},//5
    {   0,   1,   1,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   1,   1,   0},//6
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//7
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//8
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//9
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//10
    {   0,   0,   0,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   0,   0,   0},//11
    {   0,   0,   0,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   0,   0,   0},//12
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//13
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//14
    {   0,   0,   0,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   0,   0,   0},//15
    {   0,   0,   0,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   0,   0,   0},//16
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//17
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//18
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//19
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//20
    {   0,   1,   1,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   1,   1,   0},//21
    {   0,   1,   1,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   1,   1,   0},//22
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//23
    {   0,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0},//24
    {   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0},//25
    {   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0},//26
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0} //27
};
int singleLineMapDegree[COL][ROW] = 
{   
    //  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//0
    {   0,   0,   0,   0,   0,   0,  90,  90,  90,  90,  90,  90,  90,   0,   0,   0,   0,   0,   0},//1
    {   0,   0,   0,   0,   0,   0,  90,  90,  90,  90,  90,  90,  90,   0,   0,   0,   0,   0,   0},//2
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//3
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//4
    {   0,  90,  90,   0,   0,   0,  90,  90,  90,  90,  90,  90,  90,   0,   0,   0,  90,  90,   0},//5
    {   0,  90,  90,   0,   0,   0,  90,  90,  90,  90,  90,  90,  90,   0,   0,   0,  90,  90,   0},//6
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//7
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//8
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//9
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//10
    {   0,   0,   0,  90,  90,  90,   0,   0,   0,   0,   0,   0,   0,  90,  90,  90,   0,   0,   0},//11
    {   0,   0,   0,  90,  90,  90,   0,   0,   0,   0,   0,   0,   0,  90,  90,  90,   0,   0,   0},//12
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//13
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//14
    {   0,   0,   0,  90,  90,  90,   0,   0,   0,   0,   0,   0,   0,  90,  90,  90,   0,   0,   0},//15
    {   0,   0,   0,  90,  90,  90,   0,   0,   0,   0,   0,   0,   0,  90,  90,  90,   0,   0,   0},//16
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//17
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//18
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//19
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//20
    {   0,  90,  90,   0,   0,   0, -90, -90, -90, -90, -90, -90, -90,   0,   0,   0,  90,  90,   0},//21
    {   0,  90,  90,   0,   0,   0, -90, -90, -90, -90, -90, -90, -90,   0,   0,   0,  90,  90,   0},//22
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//23
    {   0,  90,  90,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  90,  90,   0},//24
    {   0,   0,   0,   0,   0,   0, -90, -90, -90, -90, -90, -90, -90,   0,   0,   0,   0,   0,   0},//25
    {   0,   0,   0,   0,   0,   0, -90, -90, -90, -90, -90, -90, -90,   0,   0,   0,   0,   0,   0},//26
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0} //27
};


//T corner
float centreTCornerMap[COL][ROW] = {   
    //  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//0
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//1
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//2
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//3
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//4
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//5
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//6
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//7
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//8
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//9
    {   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1},//10
    {   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1},//11
    {   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1},//12
    {   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1},//13
    {   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1},//14
    {   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1},//15
    {   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1},//16
    {   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1},//17
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//19
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//20
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//21
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//22
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//23
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//24
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//25
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//26
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0} //27
};
int centreTCornerMapDegree[COL][ROW] = 
{   
    //  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//0
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//1
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//2
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//3
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//4
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//5
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//6
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//7
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//8
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//9
    { 225, 225, 225, 225,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, -45, -45, -45, -45},//10
    { 225, 225, 225, 225,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, -45, -45, -45, -45},//11
    { 225, 225, 225, 225,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, -45, -45, -45, -45},//12
    { 225, 225, 225, 225,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, -45, -45, -45, -45},//13
    { 135, 135, 135, 135,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  45,  45,  45,  45},//14
    { 135, 135, 135, 135,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  45,  45,  45,  45},//15
    { 135, 135, 135, 135,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  45,  45,  45,  45},//16
    { 135, 135, 135, 135,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  45,  45,  45,  45},//17
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//18
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//19
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//20
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//21
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//22
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//23
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//24
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//25
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},//26
    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0} //27
};
//----------------------------------------------------------------------------------------------------------------



////////////////////////////////////////////////////////
///////////////// VISION ///////////////////////////////
////////////////////////////////////////////////////////
// Prototype functions for vision part. Implementation at the bottom
void increaseNumFeatureDetected(vector<string> detectedFeatures);
string determineFeature();
void updateVisionMap(string feature);
void updateVisionMapDegree(string feature, float angle);
void clearFeatureCounter();
void mapCallBackFunc(int event, int x, int y, int flags, void *userdata);


#define PI 3.14159265358979323846
int testbit = 1;
int initRandom = 0;
int testobx;
int testoby;

vector<PointXY> obstacles;
MyRobot robot;

int gridMap[COL][ROW];

int particles[5][3];
float particleProb[5];
int numParticle = 0;

float motionMap[COL][ROW] = {0};
int motionMapDegree[COL][ROW] = {0};
float visionMap[COL][ROW] = {0};
int visionMapDegree[COL][ROW] = {0};
float globalMap[COL][ROW] = {0};
float globalMapDegree[COL][ROW] = {0};
int dx, dy , rx,ry;
int steps;

void drawMap();
void adjustDegree();
void addObstacle();


///////////////////////////////////////////////////////
///////////////// Motion ///////////////////////////
///////////////////////////////////////////////////////

void drawVision()
{
    //triangle coordinate offset
    int x1,x2,x3,y1,y2,y3,fx1,fx2,fx3,fy1,fy2,fy3;

    Mat field = imread("./soccer_field.png",CV_LOAD_IMAGE_COLOR);
    for(int row = 0; row < ROW; row++){
        for(int col=0; col < COL; col++){
            if(visionMap[col][row] > 0){
                int pX = col;
                int pY = row;
                int pD = visionMapDegree[col][row];
                x1 = 0;
                y1 = 10;
                x2 = -5;
                y2 = -10;
                x3 = 5;
                y3 = -10;

                //rotation by degree (add 0.5 for proper rounding)
                fx1 = x1*cos(pD*PI/180) - y1*sin(pD*PI/180) + 0.5;
                fy1 = x1*sin(pD*PI/180) + y1*cos(pD*PI/180) + 0.5;
                fx2 = x2*cos(pD*PI/180) - y2*sin(pD*PI/180) + 0.5;
                fy2 = x2*sin(pD*PI/180) + y2*cos(pD*PI/180) + 0.5;
                fx3 = x3*cos(pD*PI/180) - y3*sin(pD*PI/180) + 0.5;
                fy3 = x3*sin(pD*PI/180) + y3*cos(pD*PI/180) + 0.5;

                //add rotated offset
                Point triPoints[3];
                triPoints[0] = Point(pX*30+15 + fx1,pY*30+15 + fy1);
                triPoints[1] = Point(pX*30+15 + fx2,pY*30+15 + fy2);
                triPoints[2] = Point(pX*30+15 + fx3,pY*30+15 + fy3);
                const Point* ppt[1] = { triPoints };
                int npt[] = { 3 };
                fillPoly(field, ppt, npt, 1, Scalar(0, 0, 255), 8);
            }
        }
    }

    imshow("imageVision", field);
}

float normDist[3][3] = 
{
    {0.1,0.1,0.1},
    {0.1,0.2,0.1},
    {0.1,0.1,0.1}
};
float normDist5[5][5] = {
    {0.010000, 0.020000, 0.030000, 0.020000, 0.010000},
    {0.020000, 0.060000, 0.080000, 0.060000, 0.020000},
    {0.030000, 0.080000, 0.120000, 0.080000, 0.030000},
    {0.020000, 0.060000, 0.080000, 0.060000, 0.020000},
    {0.010000, 0.020000, 0.030000, 0.020000, 0.010000}
};

void adjustDegree()
{
    while (robot.theta < 0)
        robot.theta += 360;
    robot.theta = robot.theta % 360;
}

int globalMapSum(){
	int summ = 0;
	for(int i=0; i<ROW; i++){
        for(int j = 0; j<COL; j++)
            summ += globalMap[i][j];
    }
    return summ;
}

int motionMapSum(){
	int summ = 0;
	for(int i=0; i<ROW; i++){
        for(int j = 0; j<COL; j++)
            summ += motionMap[i][j];
    }
    return summ;
}

void printMotionMap(){
    for(int i=0; i<ROW; i++){
        for(int j = 0; j<COL; j++)
            printf("%0.3f ", motionMap[j][i]);
        printf("%s\n", "");
    }
}

void printMotionMapDegree(){
    for(int i=0; i<ROW; i++){
        for(int j = 0; j<COL; j++)
            printf("%3d ", motionMapDegree[j][i]);
        printf("%s\n", "");
    }
}
void printVisionMap(){
    for(int i=0; i<ROW; i++){
        for(int j = 0; j<COL; j++)
            printf("%0.1f ", visionMap[j][i]);
        printf("%s\n", "");
    }
}
void updateRobotCoord(){
    //first particle is the most high prob.
    robot.x = particles[0][0];
    robot.y = particles[0][1];
    robot.theta = particles[0][2];
}

void drawMap()
{
    // adjustDegree();
    //load field img and draw any obsticle
    Mat field = imread("./soccer_field.png",CV_LOAD_IMAGE_COLOR);
    for(int i = 0; i < obstacles.size() ; i++){
        circle(field, Point(obstacles.at(i).x * 30+15, obstacles.at(i).y * 30+15), 15, Scalar(255,0,255), -1);
    }
    
    //circle(field, Point(testobx * 30+15, testoby * 30+15), 15, Scalar(255,255,0), -1);

    //robot location(draw triangle)
    updateRobotCoord();
    //triangle coordinate offset
    int x1,x2,x3,y1,y2,y3,fx1,fx2,fx3,fy1,fy2,fy3;

    //vertex of triangles(when 0 degree)
    x1 = 0;
    y1 = 17;
    x2 = -10;
    y2 = -17;
    x3 = 10;
    y3 = -17;

    //rotation by degree (add 0.5 for proper rounding)
    fx1 = x1*cos( robot.theta *PI/180) - y1*sin(robot.theta *PI/180) + 0.5;
    fy1 = x1*sin(robot.theta*PI/180) + y1*cos(robot.theta*PI/180) + 0.5;
    fx2 = x2*cos(robot.theta*PI/180) - y2*sin(robot.theta*PI/180) + 0.5;
    fy2 = x2*sin(robot.theta*PI/180) + y2*cos(robot.theta*PI/180) + 0.5;
    fx3 = x3*cos(robot.theta*PI/180) - y3*sin(robot.theta*PI/180) + 0.5;
    fy3 = x3*sin(robot.theta*PI/180) + y3*cos(robot.theta*PI/180) + 0.5;

    //add rotated offset
    Point triPoints[3];
    triPoints[0] = Point(robot.x*30+15 + fx1,robot.y*30+15 + fy1);
    triPoints[1] = Point(robot.x*30+15 + fx2,robot.y*30+15 + fy2);
    triPoints[2] = Point(robot.x*30+15 + fx3,robot.y*30+15 + fy3);
    const Point* ppt[1] = { triPoints };
    int npt[] = { 3 };
    fillPoly(field, ppt, npt, 1, Scalar(0, 0, 255), 8);
	/*
    //draw other patricle smaller
    for(int i=0;i<numParticle;i++){
        int pX = particles[i][0];
        int pY = particles[i][1];
        int pD = particles[i][2];
        x1 = 0;
        y1 = 10;
        x2 = -5;
        y2 = -10;
        x3 = 5;
        y3 = -10;

        //rotation by degree (add 0.5 for proper rounding)
        fx1 = x1*cos(pD*PI/180) - y1*sin(pD*PI/180) + 0.5;
        fy1 = x1*sin(pD*PI/180) + y1*cos(pD*PI/180) + 0.5;
        fx2 = x2*cos(pD*PI/180) - y2*sin(pD*PI/180) + 0.5;
        fy2 = x2*sin(pD*PI/180) + y2*cos(pD*PI/180) + 0.5;
        fx3 = x3*cos(pD*PI/180) - y3*sin(pD*PI/180) + 0.5;
        fy3 = x3*sin(pD*PI/180) + y3*cos(pD*PI/180) + 0.5;

        //add rotated offset
        Point triPoints[3];
        triPoints[0] = Point(pX*30+15 + fx1,pY*30+15 + fy1);
        triPoints[1] = Point(pX*30+15 + fx2,pY*30+15 + fy2);
        triPoints[2] = Point(pX*30+15 + fx3,pY*30+15 + fy3);
        const Point* ppt[1] = { triPoints };
        int npt[] = { 3 };
        fillPoly(field, ppt, npt, 1, Scalar(0, 0, 255), 8);
    }
    * */
    imshow("image", field);
}

void cleanMotionMap(){
    for(int col = 0; col < COL; col++){
        for(int row = 0; row < ROW; row++){
            motionMap[col][row] = 0;
            motionMapDegree[col][row] = 0;
        }
    }
}

//create new particle
void setParticle(int i, int x, int y, int deg, float p){
    particles[i][0] = x;
    particles[i][1] = y;
    particles[i][2] = deg;
    particleProb[i] = p;
}

void normalizeParticle(){
    float sumProb = 0;
    for(int i=0;i<5;i++){
        sumProb+=particleProb[i];
    }
    for(int i=0;i<5;i++){
        particleProb[i] = particleProb[i]/sumProb;
    }
}

//update angle of all particles
void turn(int deg){
    for(int i=0;i<numParticle;i++)
        particles[i][2] += deg;
}

void motion(){

    cleanMotionMap();
    for(int i=0; i<numParticle; i++){
        float particleW = particleProb[i];
        int curX = particles[i][0];
        int curY = particles[i][1];
        int curDeg = particles[i][2];

        //calculate next coordinate base on orientation angle
        int nextX = curX - sin(curDeg*PI/180)+0.5;
        int nextY = curY + cos(curDeg*PI/180)+0.5;

        printf("nextx:%d, nexty:%d\n", nextX, nextY);

        //mark their probability on the movement Map
        for(int j = 0; j<3; j++){
            for(int k = 0; k<3; k++){
                int localX = nextX-1+j;
                int localY = nextY-1+k;
                //dont mark it if outside map
                if(localX<COL && localX>-1 && localY<ROW && localY>-1){
                    //add prob when two differnet particle meets
                    // motionMapDegree[localX][localY] = (motionMap[localX][localY]* motionMapDegree[localX][localY] + (particleW * curDeg))
                    //                 /(motionMap[localX][localY]+particleW);
                    // motionMap[localX][localY] += particleW * normDist5[j][k];

                    //pick higher probability particle
                    if(motionMap[localX][localY]<particleW * normDist5[j][k]){
                        motionMap[localX][localY] = particleW * normDist5[j][k];
                        motionMapDegree[localX][localY] = curDeg;
                    }

                }
                
            }
        }
    }
}

//pick top 5 particles
void updateParticles(){
    std::priority_queue<Tuple> pq;
    for (int i = 0; i < COL; ++i) {
        for (int j = 0; j < ROW; ++j){
            if(globalMap[i][j]>0)
                pq.push(Tuple(globalMap[i][j], i, j));
        }
    }
    int top = 5; // number of indices we need
    numParticle = 0;
    for (int i = 0; i < top && !pq.empty(); ++i) {
        float kip = pq.top().pd;
        int kix = pq.top().x;
        int kiy = pq.top().y;
        pq.pop();

        setParticle(i,kix,kiy,globalMapDegree[kix][kiy],kip);
        numParticle++;
    }
}

void updateParticlesMotion(){
    std::priority_queue<Tuple> pq;
    for (int i = 0; i < COL; ++i) {
        for (int j = 0; j < ROW; ++j){
            if(motionMap[i][j]>0)
                pq.push(Tuple(motionMap[i][j], i, j));
        }
    }
    int top = 5; // number of indices we need
    numParticle = 0;
    for (int i = 0; i < top && !pq.empty(); ++i) {
        float kip = pq.top().pd;
        int kix = pq.top().x;
        int kiy = pq.top().y;
        pq.pop();

        setParticle(i,kix,kiy,motionMapDegree[kix][kiy],kip);
        numParticle++;
    }
    normalizeParticle();
}

void printParticles(){
    for(int i=0;i<5;i++)
        printf("P: %f [%d,%d] %3d\n", particleProb[i],particles[i][0],particles[i][1], particles[i][2]);
}

void combineACTSEE(){
    for(int i = 0; i < COL; i++){
        for(int j = 0; j < ROW; j++){
            globalMap[i][j] = motionMap[i][j] * visionMap[i][j];
            globalMapDegree[i][j] = visionMapDegree[i][j];
        }
    }
}

bool findSameParticle(int x, int y){
    bool result = false;
    for(int i=0;i<5;i++){
        if(particles[i][0] == x && particles[i][1] == y){
            result = true;
            break;
        }
    }
    return result;
}

void randomParticleVision(){
    for(int i = 0; i <5; i++){
        bool added = false;
        while(!added) {
            int x = (rand() % static_cast<int>(29));
            int y = (rand() % static_cast<int>(20));
            if(visionMap[x][y] != 0 && !findSameParticle(x,y)){
                added = true;
                particles[i][0] = x;
                particles[i][1] = y;

                //OR USE DEGREE FROM VISION MAP
                particles[i][2] = visionMapDegree[x][y];

                //OR RANDOM DEGREE
                //particles[i][2] = (rand() % static_cast<int>(360));
            }
        }
        particleProb[i] = 0.2;
    }
    numParticle = 5;
}

PointXY obstacleXYScreen(int botX, int botY, int botOri, int screenX, int screenY){

    int obstacleX;
    int obstacleY;
    if(screenX<1 || screenY<1)
		return PointXY(-1,-1);
    
    if(screenX<110)
		obstacleX = 0;
	else if(screenX>=110 && screenX<=210)
		obstacleX = 1;
	else
		obstacleX = 2;
		
	if(screenY<70)
		obstacleY = 0;
	else
		obstacleY = 1;
    

    //printf("%d,%d\n",obstacleX,obstacleY );

    int diffX = abs(screenX - 160);
    int diffY = abs(screenY - 255);
    int refAngle = 90 - atan((float)diffY/diffX) * 180 / PI;
    if(screenX < 161)
        refAngle = -refAngle;

    if(obstacleX == 0)
        obstacleX = 1;
    else if(obstacleX == 1)
        obstacleX = 0;
    else if(obstacleX == 2)
        obstacleX = -1;

    if(obstacleY == 0)
        obstacleY = 2;
    else if(obstacleY == 1)
        obstacleY = 1;
  


    int x1,y1;
    float fx1,fy1;
    //vertex of triangles(when 0 degree)
    x1 = obstacleX;
    y1 = obstacleY;
    //rotation by degree (add 0.5 for proper rounding)
    fx1 = x1*cos(botOri*PI/180) - y1*sin(botOri*PI/180);
    fy1 = x1*sin(botOri*PI/180) + y1*cos(botOri*PI/180);
    if(fx1<0)
        fx1-0.5;
    else
        fx1+0.5;

    if(fy1<0)
        fy1-0.5;
    else
        fy1+0.5;

    return PointXY(fx1+botX,fy1+botY);
}

void randomParticlesInit(){
	setParticle(0,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
    setParticle(1,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
    setParticle(2,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
    setParticle(3,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
    setParticle(4,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
}
///////////////////////////////////////////////////////
///////////////// motion END ///////////////////////////
///////////////////////////////////////////////////////

// GLOBAL VARIABLES
int button = 1;
bool pressed = false;
bool ball_found; 
bool stage1 = 1; 
bool stage2 = 0; 
bool stage3 = 0; 
int numFramesForVision = 0; // to count 10 frames for feature detection
int numParallelGoalLines = 0; // to count how many times each feature was detected in these 10 frames.
int numCencirCircle = 0;
int numCentreTCorner = 0;
int numTCornerAtGoalLines = 0;
int numGoalLineLCorner = 0;
int numSingleLine = 0;
string globalFeature = "";
float globalVisionAngle = -9999;
bool needsVisionAngle = false;
Point3i marker1;
Point3i marker2;
Point3i marker3;

int distSec = 2;
int rept = 10;

bool DEBUG = false; 

Image *rgb_output; 
minIni* ini_f;
minIni* ini_b;

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


int main(void) 
{
////////////////////  Initialize Movement particles////////////////////////////
	namedWindow("image",CV_WINDOW_AUTOSIZE);
	namedWindow("imageVision",CV_WINDOW_AUTOSIZE);
    //obstacles

    
    srand(time(NULL));
    /*
    setParticle(0,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    
    numParticle++;
    setParticle(1,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
    setParticle(2,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
    setParticle(3,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
    setParticle(4,(rand()%(28)),(rand()%(19)),(rand()%(360)),0.2);
    numParticle++;
    */
    drawMap();
    if(waitKey(30)>=0){}
    
    printf("%s\n", "movement init");

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
		marker1 = vision.findColouredObject(vision.rawFrame, vision.threshold1Frame, Scalar(255, 0, 0), 30, 70000, 4, 7, 0.05);																		
		//marker2 addObstacle= vision.findColouredObject(vision.rawFrame, vision.threshold2Frame, Scalar(0, 255, 0), 50, 40000, 4, 7, 0.1);																					
		//marker3 = vision.findColouredObject(vision.rawFrame, vision.threshold3Frame, Scalar(0, 255, 0), 100, 40000, 4, 7, 0.1);
		//char strMarker1[50];
		//sprintf(strMarker1, "marker1: %d", marker1.z);
		//putText(vision.rawFrame, strMarker1, Point2f(50,50), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255));
		double center = vision.rawFrame.size().width/2;
		double right1 = center +5;
		double left1 = center -5;
		
		

		///set the callback function for any mouse event
		setMouseCallback("Control", vision.CallBackFunc, &hsvFrame);
		setMouseCallback("image", mapCallBackFunc, 0);
		vision.showGUI(); 
		
		if(waitKey(30) == 27){
				
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
				Head::GetInstance()->m_LeftLimit = 25;
				Head::GetInstance()->m_RightLimit = -25; 
				Head::GetInstance()->m_TopLimit = 40; 
				Head::GetInstance()->m_BottomLimit = -5;
            
				cout << "Initializing body complete" << endl;
			}
			/// Start button pressed
			if( StatusCheck::m_old_btn == 1 ){
				cout << "START PROGRAM" << endl;
				//Head::GetInstance()->MoveByAngle(0, -60); 
			
				pressed = true;
			
				usleep(8 * 1000);
			}
		
			/// Marker detection		
			/// If marker2 does not exist object is set to marker1
			if(marker2 == Point3i(0, 0, 0)){
			
				object = new Point2D(marker1.x , marker1.y);
			}
			/// If marker1 does not exist set object to marker2
			else if(marker1 == Point3i(0, 0, 0)){
			
				object = new Point2D(marker2.x, marker2.y);
				 
			}
			
			//If started 
			if( pressed  )
			{
				
				//ball_found = tracker.ball_position.X != 0 && tracker.ball_position.Y != 0;
                
				Walking::GetInstance()->BALANCE_ENABLE = true;
				
                if(stage1 &&!testbit){
					int xIn = marker1.x;
					int yIn = marker1.y+25;
					if(yIn > 240)
						yIn = 240;
					PointXY tempp = obstacleXYScreen(robot.x, robot.y, robot.theta, vision.getYPiller().x, vision.getYPiller().y);
					if( tempp.x >0 &&  tempp.y >0){
						testobx = tempp.x;
						testoby = tempp.y;
					}
					printf("%d,%d\n",tempp.x, tempp.y);
					drawMap();
				}
                //Enter stage 1
				else if(stage1&&!testbit){
					Head::GetInstance()->MoveByAngle(0, -60); 
					cout << "STAGE 1: " << distSec << endl; 
					timeval a, b; 
					int difference = 0; 

					gettimeofday(&a, 0);
						
					//Walk forward: 2sec:8cm, 3sec:12cm, 4sec:16.5cm, 5sec:20cm, 6sec:23.5
					//Rotate L/R: 8sec: about 45degree
					while(difference <= 8){
						gettimeofday(&b, 0); 
						difference = b.tv_sec - a.tv_sec; 
						//adjustWalk();
						if(rept ==8 || rept ==6 || rept ==4)
							adjustWalkLeft();
						else if(rept ==7 || rept ==5 || rept ==3){
							adjustWalkRight();
						}
						else if(rept ==10 || rept ==9 || rept ==2 ||  rept ==1 || rept ==0){
							adjustWalk();
							difference+=5;
						}
						Walking::GetInstance()->Start();
					}
					if(rept ==8 || rept ==6 || rept ==4){
						//adjustWalkLeft();
						turn(-45);
						drawMap();
					}
					else if(rept ==7 || rept ==5 || rept ==3){
						//adjustWalkRight();
						turn(+45);
						drawMap();
					}
					else/*(rept ==10 || rept ==9 || rept ==2 ||  rept ==1 || rept ==0)*/{
						//adjustWalk();
						//difference+=5;
						printf("HERE!!!");
						motion();
						distSec++;
						
					}
					rept--;
					stage2 = 1;
					stage1 = 0;
					
					
				}//stage1
				
				//Stage 2: Taking 20 frames and detect a feature using the vision.
				else if(stage2&&!testbit){
					Walking::GetInstance()->Stop();
					Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					cout << "STAGE 2" << endl; 
					if( numFramesForVision < 20 && !needsVisionAngle){
						numFramesForVision++;    // we use 10 frames to get the average.
						vector<string> detectedFeatures = vision.detectFeature(visionMap);
					
						if( detectedFeatures.size() > 0 ) // if any feature has been detected
							increaseNumFeatureDetected(detectedFeatures);
					}
					else if(!needsVisionAngle) {
						numFramesForVision = 0;
						globalFeature = determineFeature(); // "centreCircle" or "parallelGoalLines" or "centreLineTCorner" or "TCornerAtGoalLines", or "singleLine"
						updateVisionMap(globalFeature);
                        needsVisionAngle = (globalFeature.compare("centreCircle") == 0) ? true : false;
                        if( !needsVisionAngle ){
                            stage1 = true;
                            stage2 = false;
                        }
						combineACTSEE();
			            updateParticles();
			            drawMap();
			            
			            //printMotionMap();
			            //printVisionMap();
			            //printMotionMapDegree();
			            printParticles();
			            clearFeatureCounter();
			            cout << "Feature: " << globalFeature << endl;
					}
					
                    // when a feature is found(visionMap is found based on the feature), then we need to calculate visionMapDegree 
                    if( needsVisionAngle ){
                        float visionAngle = vision.getVisionAngle(globalFeature); // if -9999 is returned, this means that the curr frame wasn't able to detect the assigned globalFeature
                        if( visionAngle != -9999 ){
                            //updateVisionMapDegree(globalFeature, visionAngle);
                            needsVisionAngle = false;
                            stage1 = true;
                            stage2 = false;
                            globalFeature = "";
                            cout << "VisionAngle: " << visionAngle << endl;
                        }
                    }
				}
				
				//Stage 3
				else if(stage3&&!testbit){
				
					cout << "STAGE 3" << endl; 
					
					stage3 = 0;
					stage1 = 1;
					
					timeval a, b; 
					int difference = 0; 

					gettimeofday(&a, 0);
						
					//Set a timer for 2 seconds 
					while(difference <= 3){
						adjustWalkStop();
						Walking::GetInstance()->Start();
						gettimeofday(&b, 0); 
						difference = b.tv_sec - a.tv_sec; 
					}

					Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;

					if(DEBUG) cout << "Entering Stage 3" << endl;
				
				}
				
				
				//Obstacle run demo
				else if(stage1){
					Walking::GetInstance()->Stop();
					Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					cout << "STAGE 1" << endl; 
					if( numFramesForVision < 20 && !needsVisionAngle){
						numFramesForVision++;    // we use 10 frames to get the average.
						vector<string> detectedFeatures = vision.detectFeature(visionMap);
					
						if( detectedFeatures.size() > 0 ) // if any feature has been detected
							increaseNumFeatureDetected(detectedFeatures);
					}
					else if(!needsVisionAngle) {
						numFramesForVision = 0;
						globalFeature = determineFeature(); // "centreCircle" or "parallelGoalLines" or "centreLineTCorner" or "TCornerAtGoalLines", or "singleLine"
						updateVisionMap(globalFeature);
                        needsVisionAngle = (globalFeature.compare("centreCircle") == 0 || globalFeature.compare("singleLine") == 0) ? true : false;
                        if( !needsVisionAngle ){
							updateVisionMapDegree(globalFeature, 0);
                            stage1 = false;
                            stage2 = true;
                        }
						clearFeatureCounter();
					}
					
					 // when a feature is found(visionMap is found based on the feature), then we need to calculate visionMapDegree 
                    if( needsVisionAngle ){
                        globalVisionAngle = vision.getVisionAngle(globalFeature); // if -9999 is returned, this means that the curr frame wasn't able to detect the assigned globalFeature
                        if( globalVisionAngle != -9999 ){
                            updateVisionMapDegree(globalFeature, globalVisionAngle);
                            needsVisionAngle = false;
                            stage1 = false;
                            stage2 = true;
                            cout << "VisionAngle: " << globalVisionAngle << endl;
                            // Here visionMap and visionMapDegree are both ready!
                        }
                    } 
				}
				else if(stage2){
					drawVision();
					combineACTSEE();
					drawMap();
					clearFeatureCounter();
					cout << "Feature1: " << globalFeature << endl;
					//globalVisionAngle = vision.getVisionAngle(globalFeature);
					//Sees nothing and no motion prob then, create random particles
					if(initRandom==0){
						randomParticlesInit();
						initRandom = 1;
					}
					
					//if see something and no motion prob then, create particle with in possible vision area
					else if(globalFeature.compare("centreCircle")==0 && globalMapSum() ==0){
						cout << "Vision"<< endl;
						randomParticleVision();
					}
					
					// contiue with existing prob
					else if(globalMapSum() !=0){
						cout << "updateParticle"<< endl;
						updateParticles();
					}
				
					drawMap();
					
					//printMotionMap();
					//printVisionMap();
					//printMotionMapDegree();
					printParticles();
			
					PointXY newObs1 = obstacleXYScreen(robot.x, robot.y, robot.theta, 100, 70);
                    PointXY newObs2 = obstacleXYScreen(robot.x, robot.y, robot.theta, 150, 70);
                    PointXY newObs3 = obstacleXYScreen(robot.x, robot.y, robot.theta, 230, 70);
                    PointXY newObs4 = obstacleXYScreen(robot.x, robot.y, robot.theta, 100, 90);
                    PointXY newObs5 = obstacleXYScreen(robot.x, robot.y, robot.theta, 150, 90);
                    PointXY newObs6 = obstacleXYScreen(robot.x, robot.y, robot.theta, 230, 90);
					//map the obstacle if see
                    if(marker1.x > 0)
                    {

                        Point3i pos = vision.getYPiller() ;
                            
                        PointXY newObs = obstacleXYScreen(robot.x, robot.y, robot.theta, pos.x, pos.y);
                        cout << "obs " << newObs.x << " " <<newObs.y << endl;
                        bool exists = false;
                        //mark obstacle if see that
                        if(newObs.x >0 && newObs.y > 0){
                            if(obstacles.size() == 0)
                            {
                                obstacles.push_back(newObs);
                                drawMap();
							}
                            else
                            {
                                for(int i = 0; i < obstacles.size() && !exists ; i++)
                                {
                                    if(obstacles.at(i).x == newObs.x&& obstacles.at(i).y == newObs.y)
                                    {
                                        exists = true;
                                    }
                                }
                                if(!exists)
                                {
                                    obstacles.push_back(newObs);
                                    drawMap();
                                    cout << "obstacle is added \n";
                                }
                            }
                        }
                    }
					//map the obstacle if see
                   
					Point3i pos = vision.getYPiller() ;
						
					PointXY newObs = obstacleXYScreen(robot.x, robot.y, robot.theta, pos.x, pos.y);
					cout << "obs " << newObs.x << " " <<newObs.y << endl;
					bool exists = false;
					//mark obstacle if see that
					if(newObs.x >0 && newObs.y > 0){
						if(obstacles.size() == 0)
						{
							obstacles.push_back(newObs);
							drawMap();
						}
						else
						{
							for(int i = 0; i < obstacles.size() && !exists ; i++)
							{
								if(obstacles.at(i).x == newObs.x&& obstacles.at(i).y == newObs.y)
								{
									exists = true;
								}
							}
							if(!exists)
							{
								obstacles.push_back(newObs);
								drawMap();
								cout << "obstacle is added \n";
							}
						}
				   }
                    
					
					//decide motion given vision
					steps = vision.getMoveStrategy();
					
					if(globalFeature.compare("singleLine") == 0)
					{
						cout << "Single line angle : " << globalVisionAngle << endl;
						if(globalVisionAngle >-10 & globalVisionAngle < 10 && steps == 1)
						{
							steps = 4;
						}
						else if(globalVisionAngle < -10 && (steps == 1))
						{
							steps = 3;
						}
						else if(globalVisionAngle > 10 && (steps == 1))
						{
							steps = 4;
						}
					}
					else if(globalFeature.compare("TCornerAtGoalLines") == 0){
						steps = 5;
					}
					//use vision feature as well to decide
					
					//globalFeature either "centreCircle" or "singleLine". globalVisionAngle: -90 ~ 90.
					
					stage2 = false;
                    stage3 = true;
				
				}
				else if(stage3){
					cout << "stage 3 STEPS " << steps << endl;
					//forward
					if(steps ==1){
						Head::GetInstance()->MoveByAngle(0, -60); 
						timeval a, b; 
						int difference = 0; 

						gettimeofday(&a, 0);
						
						while(difference <= 3){
							gettimeofday(&b, 0); 
							difference = b.tv_sec - a.tv_sec; 
							adjustWalk();
							Walking::GetInstance()->Start();
						}
						motion();
						updateParticlesMotion();
						drawMap();
					}
                    else if(steps == 3)
                    {
                        Head::GetInstance()->MoveByAngle(0, -60);
                        timeval a, b; 
                        int difference = 0;

                        gettimeofday(&a, 0);

                        while(difference <= 8){
						    gettimeofday(&b, 0); 
						    difference = b.tv_sec - a.tv_sec; 
							adjustWalkLeft();
                            Walking::GetInstance()->Start();
						}
						cout << "before turn" << endl;
                        turn(-45);
                        cout << "after turn" << endl;
						drawMap();
						cout << "after drawmap" << endl;
						
                    }
                    else if (steps == 5)
                    {
                        Head::GetInstance()->MoveByAngle(0, -60);
                        timeval a, b; 
                        int difference = 0;

                        gettimeofday(&a, 0);

                        while(difference <= 16){
						    gettimeofday(&b, 0); 
						    difference = b.tv_sec - a.tv_sec; 
							adjustWalkRight();
                            Walking::GetInstance()->Start();
						}
                        turn(+90);
						drawMap();
						
                    }
                    else if(steps == 4)
                    {
                        Head::GetInstance()->MoveByAngle(0, -60);
                        timeval a, b; 
                        int difference = 0;

                        gettimeofday(&a, 0);

                        while(difference <= 8){
						    gettimeofday(&b, 0); 
						    difference = b.tv_sec - a.tv_sec; 
							adjustWalkRight();
                            Walking::GetInstance()->Start();
						}
                        turn(+45);
						drawMap();
						
                    }
                    stage3 = false;
                    stage1 = true;
				
				
				}
				
				
			}//pressed
		}
  }//end while Status Check for buttons
}//end main




////////
/*
    Author : Goutham

    This function will add obstacle to the list of obstacles
*/
void addObstacle(){
    
    int markerX = (marker1.x) / 106.7;
    int markerY = marker1.y / 80;
    int rangle = atan2(2 - markerY, 4 - markerX) * 180 / PI;
    rangle = (robot.theta) +(-1 * rangle);
    //cout << "RANGLE " << rangle << endl; 
    if(rangle < 0)
    {
		while (rangle < 0)
			rangle += 360;
		rangle = rangle % 360;
	}
	if(rangle > 360)
	{
		rangle = rangle % 360;
	}
	double theta = rangle * PI / 180;
    cout << "Angle << " << theta  << " " << markerX << " "<< markerY<< endl; 
    int newX = robot.x + markerX * cos(theta);
	int newY = robot.y + markerY * sin(theta);

	PointXY obs(newX, newY);
	//cout << "NEW OBSTACLE POS " << obs.x << " " << obs.y << " "<< robot.x << " " << robot.y << endl;
					
	if(obs.x < 29 && obs.y < 20)
	{
		if(obstacles.size() == 0)
		{
			//cout << "NEW OBSTACLE POS " << obs.x << " " << obs.y << " "<< robot.x << " " << robot.y << endl;
					
			obstacles.push_back(obs);
		}
		bool exists = false;
		for(int i = 0; i< obstacles.size() && !exists; i++)
		{
				if(obstacles.at(i).x == obs.x && obstacles.at(i).y == obs.y)
				{
					exists = true;
				}
		}
	}
    drawMap();
}


/////////////////////////////////////////////////////
//////////////// VISION IMPLEMENTATION //////////////
/////////////////////////////////////////////////////

//=============================================================================================
// Author: Kyle Ahn
//   This function simply increments the numFeatures if they are detected in Vision part so that
//   we can eventually determine which feature is actually detected by avering 10 frames
//=============================================================================================
void increaseNumFeatureDetected(vector<string> detectedFeatures){
    if( detectedFeatures.size() == 0 ){
        //numNoFeatures++;
    } 
    else {
        for( int i = 0; i < detectedFeatures.size(); i++ ){
            numCencirCircle = (detectedFeatures[i].compare("centreCircle") == 0 ) ? numCencirCircle + 1 : numCencirCircle;
            numCentreTCorner = (detectedFeatures[i].compare("centreTCorner") == 0 ) ? numCentreTCorner + 1 : numCentreTCorner;
            // numParallelGoalLines = (detectedFeatures[i].compare("parallelGoalLines") == 0 ) ? numParallelGoalLines + 1 : numParallelGoalLines;
            numTCornerAtGoalLines = (detectedFeatures[i].compare("TCornerAtGoalLines") == 0 ) ? numTCornerAtGoalLines + 1 : numTCornerAtGoalLines;
            //numGoalLineLCorner = (detectedFeatures[i].compare("goalLineLCorner") == 0 ) ? numGoalLineLCorner + 1 : numGoalLineLCorner;
            numSingleLine = (detectedFeatures[i].compare("singleLine") == 0 ) ? numSingleLine + 1 : numSingleLine;
            if(detectedFeatures[i].compare("singleLine") == 0 )
				cout << "numSingleLine" << numSingleLine << endl;
    	}
    }
}


//=============================================================================================
// Author: Kyle Ahn
//   This feature returns a feature that is detected as a string by determining which feature
//   is the mostly likely a correctly detected feature out of 10 frames.
//   Returns one of "centreCircle" or "parallelGoalLines" or "centreLineTCorner" or "TCornerAtGoalLines" 
//=============================================================================================
string determineFeature(){
	string feature = "No Features";
	if( numCencirCircle > 4 ){ // since circle can only be detected when there is actually a circle. (will be no confusion with other features )
		feature = "centreCircle";
	} 
	else if( numCentreTCorner > 4 && numParallelGoalLines <= 4 && numTCornerAtGoalLines <= 4 ){ // if centreTCorner( redline) is detected without parallelLines and goal T lines. ( confusion likely to happen )
		feature = "centreTCorner";
	}
	//else if( numParallelGoalLines > 4 && numTCornerAtGoalLines <= 4 && numCentreTCorner <= 4){ // if parallel lines are detected, but not goalTCorner nor centreTCorner
	// 	feature = "parallelGoalLines";
	//}
    else if( numTCornerAtGoalLines >= 4 && numCentreTCorner <=4 ){
        feature = "goalLineTCorner";
    }
    else if( numSingleLine > 6 && numCencirCircle <= 4 && numCentreTCorner <= 4 && numGoalLineLCorner <= 4 ){
        feature = "singleLine";
    }
	return feature;
}


//=============================================================================================
// Author: Kyle Ahn
//   This function simply updates the visionMap depending on the feature detected by vision so that
//   the visionMap can be multiplied with motionMap to result in better probabilities in localisation.
//=============================================================================================
void updateVisionMap(string feature){
    for( int i = 0; i < COL; i++ ) {
		for( int j = 0; j < ROW; j++ ){
			visionMap[i][j] = 0;
		}
	}
    if( feature.compare("centreCircle") == 0){
        for( int cols = 0; cols < COL; cols++ ){
            for( int rows = 0; rows < ROW; rows++ ){
                visionMap[cols][rows] = centreCircleMap[cols][rows];
            }
        }           
    } 
    else if( feature.compare("goalLineTCorner") == 0 ){
        for( int cols = 0; cols < COL; cols++ ){
            for( int rows = 0; rows < ROW; rows++ ){
                visionMap[cols][rows] = goalLineLCornerMap[cols][rows];
            }
        }      
    } 
    else if( feature.compare("singleLine") == 0 ){
        for( int cols = 0; cols < COL; cols++ ){
            for( int rows = 0; rows < ROW; rows++ ){
                visionMap[cols][rows] = singleLineMap[cols][rows];
            }
        }      
    } 
    else if( feature.compare("centreTCorner") == 0){
        for( int cols = 0; cols < COL; cols++ ){
            for( int rows = 0; rows < ROW; rows++ ){
                visionMap[cols][rows] = centreTCornerMap[cols][rows];
            }
        }      
    }
}


//=====================================================================================================
// Author: Kyle Ahn
//   updates the orientation of the robot using the vision.
//=====================================================================================================
void updateVisionMapDegree(string feature, float angle){
    for( int i = 0; i < COL; i++ ) {
		for( int j = 0; j < ROW; j++ ){
			visionMapDegree[i][j] = 0;
		}
	}
    if( feature.compare("centreCircle") == 0 ){
        for( int cols = 0; cols < COL; cols++ ){
            for( int rows = 0; rows < ROW; rows++ ){
                visionMapDegree[cols][rows] = centreCircleMapDegree[cols][rows];
                if( angle < 0 ){
                    if( centreCircleMapDegree[cols][rows] == 2 || centreCircleMapDegree[cols][rows] == 4 ){
                        visionMapDegree[cols][rows] = 0;
                        visionMap[cols][rows] = 0;
					}
                    else if( centreCircleMapDegree[cols][rows] == 1 )
                        visionMapDegree[cols][rows] = 90 + angle;
                    else if( centreCircleMapDegree[cols][rows] == 3 )
                        visionMapDegree[cols][rows] = 90 + angle + 180;
                } 
                else {
                    if( centreCircleMapDegree[cols][rows] == 1 || centreCircleMapDegree[cols][rows] == 3 ){
                        visionMapDegree[cols][rows] = 0;
                        visionMap[cols][rows] = 0;
					}
                    else if( centreCircleMapDegree[cols][rows] == 2 )
                        visionMapDegree[cols][rows] = -90 + angle;
                    else if( centreCircleMapDegree[cols][rows] == 4 )
                        visionMapDegree[cols][rows] = -90 + angle + 180;
                }
            }
        }      
    }
    else if( feature.compare("goalLineTCorner") == 0 ){
        for( int cols = 0; cols < COL; cols++ ){
            for( int rows = 0; rows < ROW; rows++ ){
                visionMapDegree[cols][rows] = goalLineLCornerMapDegree[cols][rows];
            }
        }   
    } 
    else if( feature.compare("singleLine") == 0 ){
        for( int cols = 0; cols < COL; cols++ ){
            for( int rows = 0; rows < ROW; rows++ ){
                visionMapDegree[cols][rows] = singleLineMapDegree[cols][rows];
            }
        }   
    }
    else if( feature.compare("centreTCorner") == 0 ){
        for( int cols = 0; cols < COL; cols++ ){
            for( int rows = 0; rows < ROW; rows++ ){
                visionMapDegree[cols][rows] = centreTCornerMapDegree[cols][rows];
            }
        }   
    }
}


void clearFeatureCounter(){
	numParallelGoalLines = 0; // to count how many times each feature was detected in these 10 frames.
	numCencirCircle = 0;
	numCentreTCorner = 0;
	numTCornerAtGoalLines = 0;
    numGoalLineLCorner = 0;
    numSingleLine = 0;
}



void mapCallBackFunc(int event, int x, int y, int flags, void *userdata)
{
	if(event == EVENT_LBUTTONDOWN)
	{
		dx = x;
		dy = y;
	}
	else if(event == EVENT_LBUTTONUP)
	{
		rx = x;
		ry = y;
		int rangle = (atan2(ry - dy, rx-dx) * 180/ PI) - 90;
		
		//robot.x = dx / 30;
		//robot.y = dy /30;
		//robot.theta = rangle;
		setParticle(0,dx / 30, dy /30, rangle,1);
		numParticle = 1;
		adjustDegree();
		cout << "robot pos and angle " << dx/30 << " " << dy/30 << " "<< robot.theta << endl;
		drawMap();
	}
}
