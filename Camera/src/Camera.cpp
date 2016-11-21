#include <cv.h>
#include <opencv2/core/core.hpp>
#include <highgui.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include "CompressiveTracker.h"

using namespace cv;
using namespace std;

int main()
{
    VideoCapture capture;
    capture.open(0);
    if(!capture.isOpened())
    {
        return 1;
    }
    double rate = capture.get(CV_CAP_PROP_FPS);
    int delay = 1000/rate;
    namedWindow("Camera",CV_WINDOW_AUTOSIZE);
    Mat frame;
    int key;
    while(capture.read(frame))
    {
        imshow("Camera",frame);
        key = cvWaitKey(33);
        if(key=='q')
        {
            capture.release();
            return 0;
            break;
        }
    }
    return 0;
}
