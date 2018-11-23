#include <iostream>
#include <string>
#include <sys/time.h>
#include <algorithm>
#include <iterator>
#include <queue>
#include <vector>
#include <tuple>
#include <stdexcept>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>

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
#define PI 3.14159265

//  84 X 57??
const int COL = 28;
const int ROW = 19;
int map[COL][ROW] = {0};  //global int array
int degree = 90;    //orientation of robot.
int robotX = 20;
int robotY = 2;

int particles[5][3];
float particleProb[5];
int numParticle = 0;

float motionMap[COL][ROW] = {0};
int motionMapDegree[COL][ROW] = {0};

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
    while (degree < 0)
        degree += 360;
    degree = degree%360;
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
void updateRobotCoord(){
    //first particle is the most high prob.
    robotX = particles[0][0];
    robotY = particles[0][1];
    degree = particles[0][2];
}

void drawField()
{
    // adjustDegree();
    //load field img and draw any obsticle
    Mat field = imread("../soccer_field.png",CV_LOAD_IMAGE_COLOR);
    for(int row = 0; row < ROW; row++){
        for(int col=0; col < COL; col++){
            if(map[col][row] < 0){
                //obstacle.
                circle(field, Point(col*30+15,row*30+15), 15, Scalar(255,0,255), -1);
            }
        }
    }

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
            if(motionMap[i][j]>0)
                pq.push(Tuple(motionMap[i][j], i, j));
        }
    }
    int top = 5; // number of indices we need
    numParticle = 0;
    for (int i = 0; i < top/*not empty*/; ++i) {
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

int main(int argc, char** argv)
{
    int action[20] = {0,0,1,0,1,0,2,0,0,2,0,1,0,2,0,0,1,0,0,1};
    namedWindow("image",CV_WINDOW_AUTOSIZE);
    //set obstacles
    map[10][12] = -1;
    map[8][1] = -1;
    map[3][9] = -1;
    map[14][5] = -1;
    map[22][7] = -1;
    map[5][17] = -1;


    //add 5 random particle
    srand( time(NULL) );
    setParticle(0,(std::rand()%( 28 )),(std::rand()%( 19 )),(std::rand()%( 360 )),0.2);
    numParticle++;

    setParticle(1,(std::rand()%( 28 )),(std::rand()%( 19 )),(std::rand()%( 360 )),0.2);
    numParticle++;

    setParticle(2,(std::rand()%( 28 )),(std::rand()%( 19 )),(std::rand()%( 360 )),0.2);
    numParticle++;

    setParticle(3,(std::rand()%( 28 )),(std::rand()%( 19 )),(std::rand()%( 360 )),0.2);
    numParticle++;

    setParticle(4,(std::rand()%( 28 )),(std::rand()%( 19 )),(std::rand()%( 360 )),0.2);
    numParticle++;


    timeval a, b; 
    float difference = 0;     
    drawField();
    if(waitKey(100) >= 0){

    }

    //Set a timer for 2 seconds 
    for (int k = 0; k< 20; k++){
        if(action[k] == 0){ //move forward
            gettimeofday(&a, 0);
            difference = 0;
            while(difference <= 3){
                gettimeofday(&b, 0); 
                difference = b.tv_sec - a.tv_sec;
                if(waitKey(30) >= 0) break; 
            }
            motion();
            updateParticles();
            drawField();
            printMotionMap();
            printMotionMapDegree();
            printParticles();
        }
        else if(action[k] == 1){//left
            gettimeofday(&a, 0);
            difference = 0;
            while(difference <= 3){
                gettimeofday(&b, 0); 
                difference = b.tv_sec - a.tv_sec; 
            }
            turn(45);

            
        }
        else if(action[k] == 2){//right
            gettimeofday(&a, 0);
            difference = 0;
            while(difference <= 3){
                gettimeofday(&b, 0); 
                difference = b.tv_sec - a.tv_sec; 
            }
            turn(-45);
        }
        if(waitKey(30) >= 0) break;
    }
    // motion();
    // printMotionMap();
    // printMotionMapDegree();
    // updateParticles();
    // printParticles();

    // motion();
    // printMotionMap();
    // printMotionMapDegree();
    // updateParticles();
    // printParticles();

    // motion();
    // printMotionMap();
    // printMotionMapDegree();
    // updateParticles();
    // printParticles();
    

    waitKey();
    return 0;
}