#include "CompressiveTracker.h"
#include <math.h>
#include <iostream>
using namespace cv;
using namespace std;

//------------------------------------------------
CompressiveTracker::CompressiveTracker(void)
{
    featureMinNumRect = 2;
    featureMaxNumRect = 4;	// number of rectangle from 2 to 4
    featureNum = 60;	// number of all weaker classifiers, i.e,feature pool
    rOuterPositive = 5;	// radical scope of positive samples
    rSearchWindow = 27; // size of search window
	muPositive = vector<float>(featureNum, 0.0f);
	muNegative = vector<float>(featureNum, 0.0f);
	sigmaPositive = vector<float>(featureNum, 1.0f);
	sigmaNegative = vector<float>(featureNum, 1.0f);
    learnRate = 0.33f;	// Learning rate parameter
}

CompressiveTracker::~CompressiveTracker(void)
{

}


void CompressiveTracker::HaarFeature(Rect& _objectBox, int _numFeature)
/*Description: compute Haar features
  Arguments:
  -_objectBox: [x y width height] object rectangle
  -_numFeature: total number of features.The default is 50.
*/
{
	features = vector<vector<Rect> >(_numFeature, vector<Rect>());
	featuresWeight = vector<vector<float> >(_numFeature, vector<float>());
	
	int numRect;
	Rect rectTemp;
	float weightTemp;
      
	for (int i=0; i<_numFeature; i++)
	{
		numRect = cvFloor(rng.uniform((double)featureMinNumRect, (double)featureMaxNumRect));
	    
		//int c = 1;
		for (int j=0; j<numRect; j++)
		{
			
			rectTemp.x = cvFloor(rng.uniform(0.0, (double)(_objectBox.width - 3)));
			rectTemp.y = cvFloor(rng.uniform(0.0, (double)(_objectBox.height - 3)));
			rectTemp.width = cvCeil(rng.uniform(0.0, (double)(_objectBox.width - rectTemp.x - 2)));
			rectTemp.height = cvCeil(rng.uniform(0.0, (double)(_objectBox.height - rectTemp.y - 2)));
			features[i].push_back(rectTemp);

			weightTemp = (float)pow(-1.0, cvFloor(rng.uniform(0.0, 2.0))) / sqrt(float(numRect));
            //weightTemp = (float)pow(-1.0, c);
			
			featuresWeight[i].push_back(weightTemp);
           
		}
	}
}


void CompressiveTracker::sampleRect(Mat& _image, Rect& _objectBox, float _rInner, float _rOuter, int _maxSampleNum, vector<Rect>& _sampleBox)
/* Description: compute the coordinate of positive and negative sample image templates
   Arguments:
   -_image:        processing frame
   -_objectBox:    recent object position 
   -_rInner:       inner sampling radius
   -_rOuter:       Outer sampling radius
   -_maxSampleNum: maximal number of sampled images
   -_sampleBox:    Storing the rectangle coordinates of the sampled images.
*/
{
	int rowsz = _image.rows - _objectBox.height - 1;
	int colsz = _image.cols - _objectBox.width - 1;
	float inradsq = _rInner*_rInner;
	float outradsq = _rOuter*_rOuter;

  	
	int dist;

	int minrow = max(0,(int)_objectBox.y-(int)_rInner);
	int maxrow = min((int)rowsz-1,(int)_objectBox.y+(int)_rInner);
	int mincol = max(0,(int)_objectBox.x-(int)_rInner);
	int maxcol = min((int)colsz-1,(int)_objectBox.x+(int)_rInner);
    
	
	
	int i = 0;

	float prob = ((float)(_maxSampleNum))/(maxrow-minrow+1)/(maxcol-mincol+1);

	int r;
	int c;
    
    _sampleBox.clear();//important
    Rect rec(0,0,0,0);

	for( r=minrow; r<=(int)maxrow; r++ )
		for( c=mincol; c<=(int)maxcol; c++ ){
			dist = (_objectBox.y-r)*(_objectBox.y-r) + (_objectBox.x-c)*(_objectBox.x-c);

			if( rng.uniform(0.,1.)<prob && dist < inradsq && dist >= outradsq ){

                rec.x = c;
				rec.y = r;
				rec.width = _objectBox.width;
				rec.height= _objectBox.height;
				
                _sampleBox.push_back(rec);				
				
				i++;
			}
		}
	
		_sampleBox.resize(i);
		
}

void CompressiveTracker::sampleRect(Mat& _image, Rect& _objectBox, float _srw, vector<Rect>& _sampleBox)
/* Description: Compute the coordinate of samples when detecting the object.*/
{
	int rowsz = _image.rows - _objectBox.height - 1;
	int colsz = _image.cols - _objectBox.width - 1;
	float inradsq = _srw*_srw;	
	

	int dist;

	int minrow = max(0,(int)_objectBox.y-(int)_srw);
	int maxrow = min((int)rowsz-1,(int)_objectBox.y+(int)_srw);
	int mincol = max(0,(int)_objectBox.x-(int)_srw);
	int maxcol = min((int)colsz-1,(int)_objectBox.x+(int)_srw);

	int i = 0;

	int r;
	int c;

	Rect rec(0,0,0,0);
    _sampleBox.clear();//important

	for( r=minrow; r<=(int)maxrow; r++ )
		for( c=mincol; c<=(int)maxcol; c++ ){
			dist = (_objectBox.y-r)*(_objectBox.y-r) + (_objectBox.x-c)*(_objectBox.x-c);

			if( dist < inradsq ){

				rec.x = c;
				rec.y = r;
				rec.width = _objectBox.width;
				rec.height= _objectBox.height;

				_sampleBox.push_back(rec);				

				i++;
			}
		}
	
		_sampleBox.resize(i);

}
// Compute the features of samples
void CompressiveTracker::getFeatureValue(Mat& _imageIntegral, vector<Rect>& _sampleBox, Mat& _sampleFeatureValue)
{
	int sampleBoxSize = _sampleBox.size();
	_sampleFeatureValue.create(featureNum, sampleBoxSize, CV_32F);
	float tempValue;
	int xMin;
	int xMax;
	int yMin;
	int yMax;

	for (int i=0; i<featureNum; i++)
	{
		for (int j=0; j<sampleBoxSize; j++)
		{
			tempValue = 0.0f;
			for (size_t k=0; k<features[i].size(); k++)
			{
				xMin = _sampleBox[j].x + features[i][k].x;
				xMax = _sampleBox[j].x + features[i][k].x + features[i][k].width;
				yMin = _sampleBox[j].y + features[i][k].y;
				yMax = _sampleBox[j].y + features[i][k].y + features[i][k].height;
				tempValue += featuresWeight[i][k] * 
					(_imageIntegral.at<float>(yMin, xMin) +
					_imageIntegral.at<float>(yMax, xMax) -
					_imageIntegral.at<float>(yMin, xMax) -
					_imageIntegral.at<float>(yMax, xMin));
			}
			_sampleFeatureValue.at<float>(i,j) = tempValue;
		}
	}
}

// Update the mean and variance of the gaussian classifier
void CompressiveTracker::classifierUpdate(Mat& _sampleFeatureValue, vector<float>& _mu, vector<float>& _sigma, float _learnRate)
{
	Scalar muTemp;
	Scalar sigmaTemp;
    
	for (int i=0; i<featureNum; i++)
	{
		meanStdDev(_sampleFeatureValue.row(i), muTemp, sigmaTemp);
	   
		_sigma[i] = (float)sqrt( _learnRate*_sigma[i]*_sigma[i]	+ (1.0f-_learnRate)*sigmaTemp.val[0]*sigmaTemp.val[0] 
		+ _learnRate*(1.0f-_learnRate)*(_mu[i]-muTemp.val[0])*(_mu[i]-muTemp.val[0]));	// equation 6 in paper

		_mu[i] = _mu[i]*_learnRate + (1.0f-_learnRate)*muTemp.val[0];	// equation 6 in paper
	}
}

// Compute the ratio classifier 
void CompressiveTracker::radioClassifier(vector<float>& _muPos, vector<float>& _sigmaPos, vector<float>& _muNeg, vector<float>& _sigmaNeg,
										 Mat& _sampleFeatureValue, float& _radioMax, int& _radioMaxIndex)
{
	float sumRadio;
	_radioMax = -FLT_MAX;
	_radioMaxIndex = 0;
	float pPos;
	float pNeg;
	int sampleBoxNum = _sampleFeatureValue.cols;

	for (int j=0; j<sampleBoxNum; j++)
	{
		sumRadio = 0.0f;
		for (int i=0; i<featureNum; i++)
		{
			pPos = exp( (_sampleFeatureValue.at<float>(i,j)-_muPos[i])*(_sampleFeatureValue.at<float>(i,j)-_muPos[i]) / -(2.0f*_sigmaPos[i]*_sigmaPos[i]+1e-30) ) / (_sigmaPos[i]+1e-30);
			pNeg = exp( (_sampleFeatureValue.at<float>(i,j)-_muNeg[i])*(_sampleFeatureValue.at<float>(i,j)-_muNeg[i]) / -(2.0f*_sigmaNeg[i]*_sigmaNeg[i]+1e-30) ) / (_sigmaNeg[i]+1e-30);
			sumRadio += log(pPos+1e-30) - log(pNeg+1e-30);	// equation 4
		}
		if (_radioMax < sumRadio)
		{
			_radioMax = sumRadio;
			_radioMaxIndex = j;
		}
	}
}
void CompressiveTracker::init(Mat& _frame, Rect& _objectBox)
{
	// compute feature template
	HaarFeature(_objectBox, featureNum);

	// compute sample templates
	sampleRect(_frame, _objectBox, rOuterPositive, 0, 1000000, samplePositiveBox);
    sampleRect(_frame, _objectBox, rSearchWindow*1.5, rOuterPositive+GAP, 200, sampleNegativeBox);

	integral(_frame, imageIntegral, CV_32F);

	getFeatureValue(imageIntegral, samplePositiveBox, samplePositiveFeatureValue);
	getFeatureValue(imageIntegral, sampleNegativeBox, sampleNegativeFeatureValue);
	classifierUpdate(samplePositiveFeatureValue, muPositive, sigmaPositive, learnRate);
	classifierUpdate(sampleNegativeFeatureValue, muNegative, sigmaNegative, learnRate);
    Tracking = true;
    first_flag = true;
    radioMax_max = 0;
    radioMax_min = 0;
    radioMax_storage[0] = 0;
    radioMax_storage[1] = 0;
    radioMax_storage[2] = 0;
}
void CompressiveTracker::processFrame(Mat& _frame, Rect& _objectBox)
{
	// predict
	sampleRect(_frame, _objectBox, rSearchWindow,detectBox);
	integral(_frame, imageIntegral, CV_32F);
	getFeatureValue(imageIntegral, detectBox, detectFeatureValue);
//	int radioMaxIndex;
//	float radioMax;
	radioClassifier(muPositive, sigmaPositive, muNegative, sigmaNegative, detectFeatureValue, radioMax, radioMaxIndex);
    //printf("radioMax = %f\r\n",radioMax);
    //printf("first_flag : %s\r\n",first_flag==false?"FALSE":"TRUE");
    if(first_flag)  //TRUE stands for the first tracking iteration , FALSE: none operation
    {
//        printf("11111111");
        radioMax_last = radioMax;
        radioMax_init = radioMax;
        radioMax_storage[0] = radioMax;
        radioMax_storage[1] = radioMax;
        radioMax_storage[2] = radioMax;
        first_flag = false;
//        printf("first_flag : %s\r\n",first_flag==false?"FALSE":"TRUE");
    }
    if(radioMax>radioMax_max)
        radioMax_max = radioMax;
    if(radioMax<radioMax_min)
        radioMax_min = radioMax;

    _objectBox = detectBox[radioMaxIndex];

	// update
	sampleRect(_frame, _objectBox, rOuterPositive, 0.0, 1000000, samplePositiveBox);
    sampleRect(_frame, _objectBox, rSearchWindow*1.5, rOuterPositive+GAP, 200, sampleNegativeBox);
	
	getFeatureValue(imageIntegral, samplePositiveBox, samplePositiveFeatureValue);
	getFeatureValue(imageIntegral, sampleNegativeBox, sampleNegativeFeatureValue);
	classifierUpdate(samplePositiveFeatureValue, muPositive, sigmaPositive, learnRate);
	classifierUpdate(sampleNegativeFeatureValue, muNegative, sigmaNegative, learnRate);
    TrackOnLock();
}
void CompressiveTracker::TrackOnLock()
{
    float relative1 = fabs((radioMax - radioMax_last)/radioMax_last);
    float relative2 = fabs((radioMax - radioMax_init)/radioMax_init);
    //printf("relative1=%f relative2=%f radioMax=%f radioMax_last=%f radioMax_init=%f\r\n",relative1,relative2,radioMax,radioMax_last,radioMax_init);

    radioMax_storage[2] = radioMax_storage[1];
    radioMax_storage[1] = radioMax_storage[0];
    radioMax_storage[0] = radioMax;
    radioMax_diff1[0] = CalculateDiff(radioMax_storage[0],radioMax_storage[1],DELTATIME);
    radioMax_diff1[1] = CalculateDiff(radioMax_storage[1],radioMax_storage[2],DELTATIME);
    radioMax_diff2 = CalculateDiff(radioMax_diff1[0],radioMax_diff1[1],DELTATIME);
    printf("radioMax[0]:%f  radioMax[1]:%f  radioMax[2]:%f\r\n",radioMax_storage[0],radioMax_storage[1],radioMax_storage[2]);
    printf("Diff1[0]:%f   Diff1[1]:%f\r\n",radioMax_diff1[0],radioMax_diff1[1]);
    printf("Diff2:%f\r\n",radioMax_diff2);
//    if(radioMax<0)
//        Tracking = false;
//    else if(relative>0.02)
//        Tracking = false;
//    if(radioMax<-20 && relative1>0.3 && relative2>0.6)
    if(relative1>relative1_max)
        relative1_max = relative1;
    if(relative1<relative1_min)
        relative1_min = relative1;
//    if(radioMax<-100 && relative1>0.5)
//        Tracking = false;
    if(radioMax_diff1[0]<-0.6 && radioMax_diff1[1]<-0.38 && radioMax_diff2<-0.02)
        Tracking = false;
    //Tracking = true;
    radioMax_last = radioMax;
}

bool CompressiveTracker::isTracking()
{
    return Tracking;
}
float CompressiveTracker::getRadioMax()
{
    return radioMax;
}
void CompressiveTracker::printHistory()
{
    printf("radioMax minimum value:%f  radioMax maximum value:%f\r\nrelative1 minimum value:%f  relative1 maximum value:%f",radioMax_min,radioMax_max,relative1_min,relative1_max);
}
float CompressiveTracker::CalculateDiff(float x1,float x2,float dT)
{
    //x1 for the value at present,x2 for the valuve of last time
    return (x1-x2)/dT;
}
