#ifndef COLORIDENTIFY_H
#define COLORIDENTIFY_H

#include <cv.h>
#include <highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
using namespace cv;
using namespace std;
#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)

#define BLUE_RedLow 0
#define BLUE_RedUp 60
#define BLUE_GreenLow 0
#define BLUE_GreenUp 60
#define BLUE_BlueLow 210
#define BLUE_BlueUp 255

#define RED_RedLow 210
#define RED_RedUp 255
#define RED_GreenLow 0
#define RED_GreenUp 60
#define RED_BlueLow 0
#define RED_BlueUp 60

#define RED_BLUE_LOW 8
#define RED_BLUE_UP 12
#define RED_GREEN_LOW 8
#define RED_GREEN_UP 12

#define BLUE_RED_LOW 80
#define BLUE_RED_UP 12
#define BLUE_GREEN_LOW 8
#define BLUE_GREEN_UP 12

#define RED_BLUEandGREEN_LOW 1
#define RED_BLUEandGREEN_UP 13
#define BLUE_REDandGREEN_LOW 1
#define BLUE_REDEandGREEN_UP 13


#define REDCAR_HUE_UP 180
#define REDCAR_HUE_LOW 165
#define REDCAR_SATURATION_UP 220
#define REDCAR_SATURATION_LOW 185

#define BLUECAR_HUE_UP 107
#define BLUECAR_HUE_LOW 100
#define BLUECAR_SATURATION_UP 255
#define BLUECAR_SATURATION_LOW 190

//#define REDCAR_HUE_UP 180
//#define REDCAR_HUE_LOW 130
//#define REDCAR_SATURATION_UP 230
//#define REDCAR_SATURATION_LOW 110

//#define BLUECAR_HUE_UP 110
//#define BLUECAR_HUE_LOW 80
//#define BLUECAR_SATURATION_UP 255
//#define BLUECAR_SATURATION_LOW 180

#define REDCAR_HUE_VAR 15
#define BLUECAR_HUE_VAR 15

#define IS_PROPERTION_1 0.03
#define NOT_PROPERTION_1 0.005
#define BOTH_PROPERTION_1 0.015

#define IS_PROPERTION_2 0.03
#define NOT_PROPERTION_2 0.005
#define BOTH_PROPERTION_2 0.015

#define IS_PROPERTION_3 0.04
#define NOT_PROPERTION_3 0.008
#define BOTH_PROPERTION_3 0.025

#define IS_PROPERTION_4 0.013
#define NOT_PROPERTION_4 0.002
#define BOTH_PROPERTION_4 0.007

#define IS_PROPERTION_5 0.04
#define NOT_PROPERTION_5 0.003
#define BOTH_PROPERTION_5 0.02
class ColorIdentify
{
public:
    /*Construct&Deconstruct Function*/
    ColorIdentify();
    ColorIdentify(uchar identifier);
    ~ColorIdentify();
    /*Pubilc Function*/
    int Identify(Mat img,Rect rect);
    int Identify(Mat img);
    void setColor(uchar identifier);
    void setMethod(int Method);
    vector<Rect> ConnectedComponents(Mat mask_process, int poly1_hull0, float perimScale, int number, Point contour_centers);
    /*Public Membership*/
    float Us_rgb_low[3];
    float Us_rgb_up[3];
    float Us_hsv_low[3];
    float Us_hsv_up[3];
    float Opposite_rgb_low[3];
    float Opposite_rgb_up[3];
    float Opposite_hsv_low[3];
    float Opposite_hsv_up[3];
private:
    /*Private Function*/
    int CarIdentify(Mat img);
    float ColorHSV(Mat h,Mat s,Mat v,int flag);
    /*Private Membership*/
    int MethodFlag;
    uchar Us_flag;
};
#endif // COLORIDENTIFY_H
