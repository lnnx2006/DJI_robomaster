/************************************************************************
* File:	CompressiveTracker.h
* Brief: C++ demo for paper: Kaihua Zhang, Lei Zhang, Ming-Hsuan Yang,"Real-Time Compressive Tracking," ECCV 2012.
* Version: 1.0
* Author: Yang Xian
* Email: yang_xian521@163.com
* Date:	2012/08/03
* History:
* Revised by Kaihua Zhang on 14/8/2012
* Email: zhkhua@gmail.com
* Homepage: http://www4.comp.polyu.edu.hk/~cskhzhang/
************************************************************************/
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <stdio.h>
#define DELTATIME 1
#define GAP 3.0
using std::vector;
using namespace cv;
using namespace std;
//---------------------------------------------------
class CompressiveTracker
{
public:
	CompressiveTracker(void);
	~CompressiveTracker(void);

private:
	int featureMinNumRect;
	int featureMaxNumRect;
	int featureNum;
	vector<vector<Rect> > features;
	vector<vector<float> > featuresWeight;
	int rOuterPositive;
	vector<Rect> samplePositiveBox;
	vector<Rect> sampleNegativeBox;
	int rSearchWindow;
	Mat imageIntegral;
	Mat samplePositiveFeatureValue;
	Mat sampleNegativeFeatureValue;
	vector<float> muPositive;
	vector<float> sigmaPositive;
	vector<float> muNegative;
	vector<float> sigmaNegative;
	float learnRate;
	vector<Rect> detectBox;
	Mat detectFeatureValue;
	RNG rng;
    bool Tracking;
    int radioMaxIndex;
    float radioMax;
    float radioMax_last;
    float radioMax_init;
    bool first_flag;
    float radioMax_max;
    float radioMax_min;
    float relative1_max;
    float relative1_min;
    float radioMax_diff1[2];
    float radioMax_diff2;
    float radioMax_storage[3];
private:
	void HaarFeature(Rect& _objectBox, int _numFeature);
	void sampleRect(Mat& _image, Rect& _objectBox, float _rInner, float _rOuter, int _maxSampleNum, vector<Rect>& _sampleBox);
	void sampleRect(Mat& _image, Rect& _objectBox, float _srw, vector<Rect>& _sampleBox);
	void getFeatureValue(Mat& _imageIntegral, vector<Rect>& _sampleBox, Mat& _sampleFeatureValue);
	void classifierUpdate(Mat& _sampleFeatureValue, vector<float>& _mu, vector<float>& _sigma, float _learnRate);
	void radioClassifier(vector<float>& _muPos, vector<float>& _sigmaPos, vector<float>& _muNeg, vector<float>& _sigmaNeg,
						Mat& _sampleFeatureValue, float& _radioMax, int& _radioMaxIndex);
    void TrackOnLock();
    float CalculateDiff(float x1,float x2,float dT);
public:
	void processFrame(Mat& _frame, Rect& _objectBox);
    void init(Mat& _frame, Rect& _objectBox);
    float getRadioMax();
    bool isTracking();
    void printHistory();
};
