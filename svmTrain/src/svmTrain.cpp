#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ml.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <opencv2/objdetect/objdetect.hpp>
#include "mySVM.h"
using namespace cv;
using namespace std;
#define wnd_size_width 64
#define wnd_size_height 48
#define block_size_width 48
#define block_size_height 48
#define block_stride 16
#define cell_size_width 16
#define cell_size_height 16

//#define PositiveSampleNum 3326
#define PositiveSampleNum 7000
#define NegativeSampleNum 91300
int main()
{
    ////////////////////////////////////
    //////////HOG initialize////////////
    ////////////////////////////////////
    //window size,block size,block stride,cell size,directions
    HOGDescriptor *hog=new HOGDescriptor(cvSize(wnd_size_width,wnd_size_height),cvSize(block_size_width,block_size_height),cvSize(block_stride,block_stride),cvSize(cell_size_width,cell_size_height),9);
    vector<float> descriptors;
    /**/
    IplImage* image_test = cvLoadImage("/home/exbot/catkin_ws/src/svmTrain/1.jpg",1);
    IplImage* image_sub = cvCreateImage(cvSize(wnd_size_width,wnd_size_height),8,3);
    Mat image_temp;
//    CvPoint2D32f center;
//    center.x = 200;
//    center.y = 200;
//    cvGetRectSubPix(image_test,image_sub,center);
//    /*œ«œØÈ¡µÄÍŒÏñ×ª»»³ÉMatÀàÐÍ*/
    CvMat *image_mat;
    image_mat = cvCreateMat(wnd_size_width,wnd_size_height,CV_8UC3);
    cvGetMat(image_test,image_mat);
    image_temp = imread("/home/exbot/catkin_ws/src/svmTrain/1.jpg",1);
    //ÍŒÏñ£¬ÌØÕ÷ÃèÊö×Ó£šÊä³ö£©£¬Ž°¿ÚÒÆ¶¯²œ³€£¬À©³äÏñËØÊý
    hog->compute(image_temp,descriptors,Size(0,0),Size(0,0));
    size_t descriptor_length = hog->getDescriptorSize();

    ////////////////////////////////////
    //////////SVM Parameters////////////
    ////////////////////////////////////
    //ÉèÖÃSVMµÄ²ÎÊý
    CvSVMParams params;
    params.svm_type    = 100;     //nÀà·Ö×é
    params.kernel_type = 0;       //ºËº¯ÊýÀàÐÍ
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_EPS, 100, 1e-6);  //CV_TERMCRIT_EPS,CV_TERMCRIT_ITER

    //////////////////////////////////////////////
    //////Training Samples Descriptors&Class//////
    //////////////////////////////////////////////
    CvMat* FeatureVectorMat = cvCreateMat(PositiveSampleNum+NegativeSampleNum,descriptor_length,CV_32FC1);
    CvMat* ResultsMat = cvCreateMat(PositiveSampleNum+NegativeSampleNum,1,CV_32FC1);
    //Mat FeatureVectorMat1(PositiveSampleNum+NegativeSampleNum,descriptor_length,CV_32FC1,0);
    //Mat ResultsMat1(PositiveSampleNum+NegativeSampleNum,1,CV_32FC1,0);
    //FeatureVectorMat1.at<double>(1,1) = 0;
    //ResultsMat1.at<double>(1,1) = 0;
    cvSetZero(FeatureVectorMat);
    cvSetZero(ResultsMat);

    //////////////////////////////////
    //////////Positive Sample/////////
    //////////////////////////////////
    int i,j;
    char filename[100];
    for(i=0;i<PositiveSampleNum;i++)
    {
        sprintf(filename,"/home/exbot/catkin_ws/src/svmTrain/Positive1/%d.jpg",i+1);
        image_temp = imread(filename,1);
        hog->compute(image_temp,descriptors,Size(10,10),Size(0,0));
        for(j=0;j<descriptor_length;j++)
        {
            //CV_MAT_ELEM(*FeatureVectorMat,float,i,j) = descriptors[j];
            cvmSet(FeatureVectorMat,i,j,descriptors[j]);
        }

        cvmSet(ResultsMat,i,0,1);
    }

    //////////////////////////////////
    //////////Negative Sample/////////
    //////////////////////////////////
    for(i=PositiveSampleNum;i<PositiveSampleNum+NegativeSampleNum;i++)
    {
        sprintf(filename,"/home/exbot/catkin_ws/src/svmTrain/Negative3/%d.jpg",i-PositiveSampleNum+1);
        image_temp = imread(filename,1);
        hog->compute(image_temp,descriptors,Size(10,10),Size(0,0));
        for(j=0;j<descriptor_length;j++)
        {
            //CV_MAT_ELEM(*FeatureVectorMat,float,i,j) = descriptors[j];
            cvmSet(FeatureVectorMat,i,j,descriptors[j]);
        }
        cvmSet(ResultsMat,i,0,-1);
    }

    ////////////////////////////////////
    ///////////SVM Training/////////////
    ////////////////////////////////////
    Mysvm SVM;
    SVM.train(FeatureVectorMat,ResultsMat, Mat(), Mat(),params);
    SVM.save("/home/exbot/catkin_ws/src/svmTrain/result.xml");
    int supportVectorSize = SVM.get_support_vector_count();

    //////////////////////////////
    //////Calculate Detector//////
    //////////////////////////////
    CvMat *sv,*alp,*re;
    sv  = cvCreateMat(supportVectorSize , descriptor_length, CV_32FC1);
    alp = cvCreateMat(1 , supportVectorSize, CV_32FC1);
    re  = cvCreateMat(1 , descriptor_length, CV_32FC1);
    CvMat *res  = cvCreateMat(1 , 1, CV_32FC1);
    cvSetZero(sv);
    cvSetZero(re);
    for(int i=0; i<supportVectorSize; i++)
    {
        memcpy( (float*)(sv->data.fl+i*descriptor_length), SVM.get_support_vector(i), descriptor_length*sizeof(float));
    }
    double* alphaArr = SVM.get_alpha();
    int alphaCount = SVM.get_alpha_count();
    for(int i=0; i<supportVectorSize; i++)
    {
        alp->data.fl[i] = -alphaArr[i];
//        alp->data.fl[i] = alphaArr[i];
    }
    cvMatMul(alp,sv,re);
    float rho = SVM.get_rho();
    //cvmSet(re,1,descriptor_length,SVM.get_rho());

    ///////////////////////////
    //////Detector to Txt//////
    ///////////////////////////
    FILE* fp = fopen("/home/exbot/catkin_ws/src/svmTrain/descriptors.txt","wb");
    fprintf(fp,"#define wnd_size_width %d\r\n",wnd_size_width);
    fprintf(fp,"#define wnd_size_height %d\r\n",wnd_size_height);
    fprintf(fp,"#define block_size_width %d\r\n",block_size_width);
    fprintf(fp,"#define block_size_height %d\r\n",block_size_height);
    fprintf(fp,"#define block_stride %d\r\n",block_stride);
    fprintf(fp,"#define cell_size_width %d\r\n",cell_size_width);
    fprintf(fp,"#define cell_size_height %d\r\n",cell_size_height);
    fprintf(fp,"#define PositiveSampleNum %d\r\n",PositiveSampleNum);
    fprintf(fp,"#define NegativeSampleNum %d\r\n",NegativeSampleNum);
    fprintf(fp,"Descriptor = {");
    for(i=0;i<descriptor_length;i++)
    {
        int o = i%10;
        if(o==0)
            fprintf(fp,"\r\n  ");
        fprintf(fp,"%f,  ",(float)cvmGet(re,0,i));
    }
    fprintf(fp,"\r\n};");
    fprintf(fp,"\r\n rho = %f;",rho);
    fclose(fp);
	return 0;
}
