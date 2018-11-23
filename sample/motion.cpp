#include <iostream>
#include <string>
#include <sys/time.h>
#include <algorithm>
#include <iterator>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
using namespace std;
using namespace cv;
#define PI 3.14159265

//  84 X 57??
const int COL = 28;
const int ROW = 19;
int map[COL][ROW] = {0};  //global int array
int degree = 0;    //orientation of robot.
int robotX = 20;
int robotY = 2;

int particles[5][3];
float particleProb[5];
int numParticle = 0;

float motionMap[COL][ROW] = {0};

float normDist[9] = {0.1,0.1,0.1,0.1,0.2,0.1,0.1,0.1,0.1};
// float normDist5[25] = {
// 0.0100000.0200000.0300000.0200000.010000
// 0.0200000.0600000.0800000.0600000.020000
// 0.0300000.0800000.1200000.0800000.030000
// 0.0200000.0600000.0800000.0600000.020000
// 0.0100000.0200000.0300000.0200000.010000};

float nextDist[9] = {0.1,0.1,0.1,0.1,0.2,0.1,0.1,0.1,0.1};
float nextDistTemp[25] = {0};
float currDist[9] = {0.1,0.1,0.1,0.1,0.2,0.1,0.1,0.1,0.1};
float currDistTemp[25] = {0};

int threeTofive[9][9] = 
{
    {0,1,2,5,6,7,10,11,12},
    {1,2,3,6,7,8,11,12,13},
    {2,3,4,7,8,9,12,13,14},
    {5,6,7,10,11,12,15,16,17},
    {6,7,8,11,12,13,16,17,18},
    {7,8,9,12,13,14,17,18,19},
    {10,11,12,15,16,17,20,21,22},
    {11,12,13,16,17,18,21,22,23},
    {12,13,14,17,18,19,22,23,24},
};

void adjustDegree()
{
    while (degree < 0)
        degree += 360;
    degree = degree%360;
}


void drawField(int rX, int rY)
{
    // adjustDegree();
    Mat field = imread("../soccer_field.png",CV_LOAD_IMAGE_COLOR);
    for(int row = 0; row < ROW; row++){
        for(int col=0; col < COL; col++){
            if(map[col][row] < 0){
                //obstacle.
                circle(field, Point(col*30+15,row*30+15), 15, Scalar(255,0,255), -1);
            }
        }
    }

    //robot location
    //triangle coordinate offset
    int x1,x2,x3,y1,y2,y3,fx1,fx2,fx3,fy1,fy2,fy3;
    x1 = 0;
    y1 = 17;
    x2 = -10;
    y2 = -17;
    x3 = 10;
    y3 = -17;

    //rotation by degree (add 0.5 for proper rounding)
    fx1 = x1*cos(degree*PI/180) - y1*sin(degree*PI/180) + 0.5;
    fy1 = x1*sin(degree*PI/180) + y1*cos(degree*PI/180) + 0.5;
    fx2 = x2*cos(degree*PI/180) - y2*sin(degree*PI/180) + 0.5;
    fy2 = x2*sin(degree*PI/180) + y2*cos(degree*PI/180) + 0.5;
    fx3 = x3*cos(degree*PI/180) - y3*sin(degree*PI/180) + 0.5;
    fy3 = x3*sin(degree*PI/180) + y3*cos(degree*PI/180) + 0.5;

    //add rotated offset
    Point triPoints[3];
    triPoints[0] = Point(rX*3+15 + fx1,rY*3+15 + fy1);
    triPoints[1] = Point(rX*3+15 + fx2,rY*3+15 + fy2);
    triPoints[2] = Point(rX*3+15 + fx3,rY*3+15 + fy3);
    const Point* ppt[1] = { triPoints };
    int npt[] = { 3 };
    fillPoly(field, ppt, npt, 1, Scalar(0, 0, 255), 8);

    
    imshow("image", field);
}

void drawField()
{
    // adjustDegree();
    Mat field = imread("../soccer_field.png",CV_LOAD_IMAGE_COLOR);
    for(int row = 0; row < ROW; row++){
        for(int col=0; col < COL; col++){
            if(map[col][row] < 0){
                //obstacle.
                circle(field, Point(col*30+15,row*30+15), 15, Scalar(255,0,255), -1);
            }
        }
    }

    //robot location
    //triangle coordinate offset
    int x1,x2,x3,y1,y2,y3,fx1,fx2,fx3,fy1,fy2,fy3;
    x1 = 0;
    y1 = 17;
    x2 = -10;
    y2 = -17;
    x3 = 10;
    y3 = -17;

    //rotation by degree (add 0.5 for proper rounding)
    fx1 = x1*cos(degree*PI/180) - y1*sin(degree*PI/180) + 0.5;
    fy1 = x1*sin(degree*PI/180) + y1*cos(degree*PI/180) + 0.5;
    fx2 = x2*cos(degree*PI/180) - y2*sin(degree*PI/180) + 0.5;
    fy2 = x2*sin(degree*PI/180) + y2*cos(degree*PI/180) + 0.5;
    fx3 = x3*cos(degree*PI/180) - y3*sin(degree*PI/180) + 0.5;
    fy3 = x3*sin(degree*PI/180) + y3*cos(degree*PI/180) + 0.5;

    //add rotated offset
    Point triPoints[3];
    triPoints[0] = Point(robotX*30+15 + fx1,robotY*30+15 + fy1);
    triPoints[1] = Point(robotX*30+15 + fx2,robotY*30+15 + fy2);
    triPoints[2] = Point(robotX*30+15 + fx3,robotY*30+15 + fy3);
    const Point* ppt[1] = { triPoints };
    int npt[] = { 3 };
    fillPoly(field, ppt, npt, 1, Scalar(0, 0, 255), 8);

    
    imshow("image", field);
}

void motion(){
}

void setParticle(int particleNum, int x, int y, int deg, float p){
    particles[particleNum][0] = x;
    particles[particleNum][1] = y;
    particles[particleNum][2] = deg;
    particleProb[particleNum] = p;
}

int main(int argc, char** argv)
{
    int action[20] = {0,0,1,0,1,0,2,0,0,2,0,1,0,2,0,0,1,0,0,1};
    namedWindow("image",CV_WINDOW_AUTOSIZE);
    //obstacles
    map[10][12] = -1;
    map[8][1] = -1;
    map[3][9] = -1;
    map[14][5] = -1;
    map[22][7] = -1;
    map[5][17] = -1;

    setParticle(0,robotX,robotY,degree,1);
    numParticle++;

    timeval a, b; 
    float difference = 0; 

    
    int first = 0;
    int second = 0;
    int third = 0;
    drawField();
    if(waitKey(30) >= 0){

    }
    printf("R(%d, %d)\n",robotX,robotY );
    //Set a timer for 2 seconds 
/*    for (int k = 0; k< 20; k++){
        std::copy(std::begin(currDist), std::end(currDist), std::begin(prevDist));
        if(action[k] == 0){
            first = 0;
            second = 0;
            third = 0;
            difference = 0;
            gettimeofday(&a, 0);
            float fx = -10/3*sin(degree*PI/180);
            float fy = 10/3*cos(degree*PI/180);
            int robotXtemp = robotX*10;
            int robotYtemp = robotY*10;
            while(difference <= 3){
                drawField(robotXtemp,robotYtemp);
                if(difference == 0 && first == 0){
                    robotXtemp += fx;
                    robotYtemp += fy;
                    first = 1;
                }
                if(difference == 1 && second == 0){
                    robotXtemp += fx;
                    robotYtemp += fy;
                    second = 1;
                }
                if(difference == 2 && third == 0){
                    robotXtemp += fx;
                    robotYtemp += fy;
                    third = 1;
                }
                gettimeofday(&b, 0); 
                difference = b.tv_sec - a.tv_sec;
                if(waitKey(30) >= 0) break; 
            }
            robotX += -sin(degree*PI/180)+0.5;
            robotY += cos(degree*PI/180)+0.5;
            printf("R(%d, %d)\n",robotX,robotY );
            drawField();
            
        }
        else if(action[k] == 1){
            first = 0;
            second = 0;
            third = 0;
            difference = 0;
            gettimeofday(&a, 0);
            while(difference <= 3){
                drawField();
                if(difference == 0 && first == 0){
                    degree += 15;
                    first = 1;
                }
                if(difference == 1 && second == 0){
                    degree += 15;
                    second = 1;
                }
                if(difference == 2 && third == 0){
                    degree += 15;
                    third = 1;
                }
                gettimeofday(&b, 0); 
                difference = b.tv_sec - a.tv_sec; 
                if(waitKey(30) >= 0) break;
            }
            
        }
        else if(action[k] == 2){
            first = 0;
            second = 0;
            third = 0;
            difference = 0;
            gettimeofday(&a, 0);
            while(difference <= 3){
                drawField();
                if(difference == 0 && first == 0){
                    degree -= 15;
                    first = 1;
                }
                if(difference == 1 && second == 0){
                    degree -= 15;
                    second = 1;
                }
                if(difference == 2 && third == 0){
                    degree -= 15;
                    third = 1;
                }
                gettimeofday(&b, 0); 
                difference = b.tv_sec - a.tv_sec; 
                if(waitKey(30) >= 0) break;
            }
        }
    }*/
   

    waitKey();
    return 0;
}