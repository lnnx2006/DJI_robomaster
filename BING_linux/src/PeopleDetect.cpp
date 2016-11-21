#include <cv.h>
#include <highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include "detect.h"
#include "ColorIdentify.h"
#include "descriptor.h"
//#define wnd_size_width 64
//#define wnd_size_height 48
//#define block_size 16
//#define block_stride 8
//#define cell_size 8
using namespace cv;
using namespace std;
int main()
{
    //Mat img[3];
    //Mat img = imread("/catkin_ws/src/PeopleDetect/DSC_0127.JPG",1);
    Mat img_origin = imread("/home/exbot/catkin_ws/src/PeopleDetect/159.jpg");
    int width = int(img_origin.cols);
    int height = int(img_origin.rows);
    Mat img(height,width,CV_8UC3);
    //img.at<0,0,0> = 0;
    //cvResize(&img_origin,&img,CV_INTER_LINEAR);
    cv::resize(img_origin,img,img.size(),0,0,1);
    //vector<Point> found_position;
    //vector<Rect> found_rect;
    vector<Mat> ImageSequence;
    vector<Rect> rect;
    rect.push_back(Rect_<int>(460,270,130,100));
    rect.push_back(Rect_<int>(270,70,300,210));
    ImageSequence.push_back(img);
    //ImageSequence.push_back(img);
    //HOGDescriptor defaultHog;
    //Rect f;
    detect DetectCar;
    DetectCar.setColor('b');
    DetectCar.setMethod(4);
    DetectCar.setCenter(int(width/2),int(height/2));
    DetectCar.setImgAttributes(int(width/2),int(height/2),width,height);
    DetectCar.setImg(img);
//    DetectCar.HOGDescriptor(cvSize(wnd_size_width,wnd_size_height),cvSize(block_size,block_size),cvSize(block_stride,block_stride),cvSize(cell_size,cell_size),9);
    vector<float> detector = vector<float>(Descriptor, Descriptor + sizeof(Descriptor)/sizeof(Descriptor[0]));
    DetectCar.setSVMDetector(detector);
    //HOGDescriptor defaultHog = HOGDescriptor(cvSize(wnd_size,wnd_size),cvSize(block_size,block_size),cvSize(block_stride,block_stride),cvSize(cell_size,cell_size),9);
    //defaultHog.setSVMDetector(Descriptor);
    //defaultHog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    //found = detect(defaultHog,ImageSequence);
    double t = (double)cvGetTickCount();
//    DetectCar.detectCar_Hog(ImageSequence);
    DetectCar.detectCar(rect);
    t = (double)cvGetTickCount() - t;
    t =  t/(cvGetTickFrequency()*1000);   //ms
    printf("Runing Time is:/home/exbot/catkin_ws/src/PeopleDetect %f\r\n",t);
    vector<Point> found_position = DetectCar.getTargetPosition();
    vector<Rect> found_rect = DetectCar.getTargetRect();
    vector<Point2f> found_angular = DetectCar.getTargetAngular();
    vector<int> found_classifier = DetectCar.getTargetClassifier();
    /*for(int i=0;i<1;i++)
        defaultHog.detectMultiScale(img, found);*/
    for(int i=0;i<found_rect.size();i++)
    {
        Rect r = found_rect[i];
        rectangle(img, r.tl(), r.br(), Scalar(0, 255, 0), 3);
    }
    namedWindow("Image", CV_WINDOW_AUTOSIZE);
    imshow("Image", img);
    imwrite("/home/exbot/catkin_ws/src/PeopleDetect/jiajia.jpg",img);
    int key = waitKey();
    if(key>0)
        return 0;
}	
