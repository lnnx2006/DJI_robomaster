#include <cv.h>
#include <highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include "ColorIdentify.h"

#define wnd_size_width 64
#define wnd_size_height 48
#define block_size_width 48
#define block_size_height 48
#define block_stride 16
#define cell_size_width 16
#define cell_size_height 16
#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 448
using namespace cv;
using namespace std;
class detect: public HOGDescriptor, public ColorIdentify
{
public:
    detect():HOGDescriptor(cvSize(wnd_size_width,wnd_size_height),cvSize(block_size_width,block_size_height),cvSize(block_stride,block_stride),cvSize(cell_size_width,cell_size_height),9)
    {

    }
    ~detect();
    /*public function*/
    vector<Rect> getTargetRect();
    vector<Point> getTargetPosition();
    vector<Point2f> getTargetAngular();
    vector<int> getTargetClassifier();
    void detectCar_Hog(vector<Mat> Images);
    void setCenter(int center_x, int center_y);
    void setImgAttributes(int center_x, int center_y, int Width, int Height);
    void setImg(Mat img);
    void detectCar(vector<Rect> rect);
    void detectCarForColor(vector<Rect> rect);

    //functions for colordetection
    void colordetection(Mat img, int xmin_box, int ymin_box);
    void ColordetectionBingBoxes(vector<Rect> rect);
    vector<Rect> ColorFoundSuspectedCar();
    Rect ColorFoundSuspectedRegion();
    vector<Rect> ConnectedComponents(Mat mask_process, int poly1_hull0, float perimScale, int number, Point contour_centers);

    /*public membership*/
    vector<Point> found_position;
    vector<Rect> found_rect;
    vector<Point2f> found_angular;
    vector<int> found_classifier;

    //variables for colordetection
    vector<Rect> FoundSuspectedCar;
    Rect FoundSuspectedRegion;

private:
    /*private function*/
    void CaculateTargetAngular();
    /*private membership*/
    Mat image;
    vector<Rect> selectRect;
    Point center;
    int ImgWidth;
    int ImgHeight;
};

//vector<Point> detect(HOGDescriptor Hog,vector<Mat> Images);
