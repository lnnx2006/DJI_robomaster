#include "ColorIdentify.h"
ColorIdentify::ColorIdentify()
{
    MethodFlag = 3;
}
ColorIdentify::ColorIdentify(uchar identifier)
{
    this->setColor(identifier);
    MethodFlag = 3;
}
ColorIdentify::~ColorIdentify()
{

}
void ColorIdentify::setColor(uchar identifier)
{
    Us_flag = identifier;
    //If identifier is 'r',mean our team is red;'b' stands for blue//
    if(identifier=='r')
    {
        Us_rgb_low[0] = RED_RedLow;
        Us_rgb_up[0] = RED_RedUp;

        Us_rgb_low[1] = RED_GreenLow;
        Us_rgb_up[1] = RED_GreenUp;

        Us_rgb_low[2] = RED_BlueLow;
        Us_rgb_up[2] = RED_BlueUp;

        Us_hsv_low[0] = REDCAR_HUE_LOW;
        Us_hsv_up[0] = REDCAR_HUE_UP;
        Us_hsv_low[1] = REDCAR_SATURATION_LOW;
        Us_hsv_up[1] = REDCAR_SATURATION_UP;

        Opposite_hsv_low[0] = BLUECAR_HUE_LOW;
        Opposite_hsv_up[0] = BLUECAR_HUE_UP;
        Opposite_hsv_low[1] = BLUECAR_SATURATION_LOW;
        Opposite_hsv_up[1] = BLUECAR_SATURATION_UP;
    }
    else if(identifier=='b')
    {
        Opposite_rgb_low[0] = BLUE_RedLow;
        Opposite_rgb_up[0] = BLUE_RedUp;

        Opposite_rgb_low[1] = BLUE_GreenLow;
        Opposite_rgb_up[1] = BLUE_GreenUp;

        Opposite_rgb_low[2] = BLUE_BlueLow;
        Opposite_rgb_up[2] = BLUE_BlueUp;

        Us_hsv_low[0] = BLUECAR_HUE_LOW;
        Us_hsv_up[0] = BLUECAR_HUE_UP;
        Us_hsv_low[1] = BLUECAR_SATURATION_LOW;
        Us_hsv_up[1] = BLUECAR_SATURATION_UP;

        Opposite_hsv_low[0] = REDCAR_HUE_LOW;
        Opposite_hsv_up[0] = REDCAR_HUE_UP;
        Opposite_hsv_low[1] = REDCAR_SATURATION_LOW;
        Opposite_hsv_up[1] = REDCAR_SATURATION_UP;
    }
}
void ColorIdentify::setMethod(int Method)
{
    MethodFlag = Method;
}

int ColorIdentify::Identify(Mat img)
{
    return CarIdentify(img);
}
int ColorIdentify::Identify(Mat img,Rect rect)
{
    Mat SubImg = img.rowRange(rect.y,rect.y+rect.height);
    SubImg = SubImg.colRange(rect.x,rect.x+rect.width);
    return CarIdentify(SubImg);
}
int ColorIdentify::CarIdentify(Mat img)
{
    vector<Mat> img_bgr(img.channels());
    split(img,img_bgr);
    ////////////////
    ///Method 1st///
    ////////////////
    //Calculate whether is us or not//
    if(MethodFlag==1)
    {
        Mat r_cmp = (img_bgr[2] > Us_rgb_low[0]) & (img_bgr[2] < Us_rgb_up[0]);
        Mat g_cmp = (img_bgr[1] > Us_rgb_low[1]) & (img_bgr[1] < Us_rgb_up[1]);
        Mat b_cmp = (img_bgr[0] > Us_rgb_low[2]) & (img_bgr[0] < Us_rgb_up[2]);
        Mat compare = r_cmp & b_cmp & g_cmp;
        /*Mat r_cmp_l = img_bgr[2] > Us_rgb_low[0];
        Mat r_cmp_u = img_bgr[2] < Us_rgb_up[0];
        Mat g_cmp_l = img_bgr[1] > Us_rgb_low[1];
        Mat g_cmp_u = img_bgr[1] < Us_rgb_up[1];
        Mat b_cmp_l = img_bgr[0] > Us_rgb_low[2];
        Mat b_cmp_u = img_bgr[0] < Us_rgb_up[2];
        Mat r_cmp = r_cmp_l==255 & r_cmp_u==255;
        Mat b_cmp = b_cmp_l==255 & b_cmp_u==255;
        Mat g_cmp = g_cmp_l==255 & g_cmp_u==255;
        Mat compare = r_cmp & b_cmp & g_cmp;*/
        int count = countNonZero(compare);
        float Percentage_Us = float(count)/(float)(img.rows*img.cols);
        r_cmp = (img_bgr[2] > Opposite_rgb_low[0]) & (img_bgr[2] < Opposite_rgb_up[0]);
        g_cmp = (img_bgr[1] > Opposite_rgb_low[1]) & (img_bgr[1] < Opposite_rgb_up[1]);
        b_cmp = (img_bgr[0] > Opposite_rgb_low[2]) & (img_bgr[0] < Opposite_rgb_up[2]);
        compare = r_cmp & b_cmp & g_cmp;

        //Calculate whether is opposite or not//
        /*r_cmp_l = img_bgr[2] > Opposite_rgb_low[0];
        r_cmp_u = img_bgr[2] < Opposite_rgb_up[0];
        g_cmp_l = img_bgr[1] > Opposite_rgb_low[1];
        g_cmp_u = img_bgr[1] < Opposite_rgb_up[1];
        b_cmp_l = img_bgr[0] > Opposite_rgb_low[2];
        b_cmp_u = img_bgr[0] < Opposite_rgb_up[2];
        r_cmp = r_cmp_l==255 & r_cmp_u==255;
        b_cmp = b_cmp_l==255 & b_cmp_u==255;
        g_cmp = g_cmp_l==255 & g_cmp_u==255;
        compare = r_cmp & b_cmp & g_cmp;*/
        count = countNonZero(compare);
        float Percentage_Opposite = float(count)/(float)(img.rows*img.cols);
        //Return  1 means the car is federate/ally
        //Return -1 means the car is enemy/foe
        //Return  0 means the car is not found
        //Return  2 means containing both side of car, while federate is bigger
        //Return -2 means containing both side of car, while enemy    is bigger
        if(Percentage_Us>=IS_PROPERTION_1 && Percentage_Opposite<BOTH_PROPERTION_1)
            return 1;
        else if(Percentage_Us<BOTH_PROPERTION_1 && Percentage_Opposite>=IS_PROPERTION_1)
            return -1;
        else if(Percentage_Us<NOT_PROPERTION_1 && Percentage_Opposite<NOT_PROPERTION_1)
            return 0;
        else if(Percentage_Us>BOTH_PROPERTION_1 && Percentage_Opposite>BOTH_PROPERTION_1 && Percentage_Us>Percentage_Opposite)
            return 2;
        else if(Percentage_Us>BOTH_PROPERTION_1 && Percentage_Opposite>BOTH_PROPERTION_1 && Percentage_Us<Percentage_Opposite)
            return -2;
        else if(Percentage_Us>IS_PROPERTION_1)
            return 2;
        else if(Percentage_Opposite>IS_PROPERTION_1)
            return -2;
        else if(Percentage_Us>BOTH_PROPERTION_1)
            return 1;
        else if(Percentage_Opposite>BOTH_PROPERTION_1)
            return -1;
        else
            return 100;
    }
    ////////////////
    ///Method 2nd///
    ////////////////
    else if(MethodFlag==2)
    {
        Mat r_b = img_bgr[2]/img_bgr[0];
        Mat r_g = img_bgr[2]/img_bgr[1];
        Mat b_r = img_bgr[0]/img_bgr[2];
        Mat b_g = img_bgr[0]/img_bgr[1];
        Mat r_cmp = r_b > RED_BLUE_LOW & r_g > RED_GREEN_LOW;
        Mat b_cmp = b_r > BLUE_RED_LOW & b_g > BLUE_GREEN_LOW;
        int count = countNonZero(r_cmp);
        float Percentage_Us = float(count)/(float)(img.rows*img.cols);
        count = countNonZero(b_cmp);
        float Percentage_Opposite = float(count)/(float)(img.rows*img.cols);
        if(Percentage_Us>=IS_PROPERTION_2 && Percentage_Opposite<BOTH_PROPERTION_2)
            return 1;
        else if(Percentage_Us<BOTH_PROPERTION_2 && Percentage_Opposite>=IS_PROPERTION_2)
            return -1;
        else if(Percentage_Us<NOT_PROPERTION_2 && Percentage_Opposite<NOT_PROPERTION_2)
            return 0;
        else if(Percentage_Us>BOTH_PROPERTION_2 && Percentage_Opposite>BOTH_PROPERTION_2 && Percentage_Us>Percentage_Opposite)
            return 2;
        else if(Percentage_Us>BOTH_PROPERTION_2 && Percentage_Opposite>BOTH_PROPERTION_2 && Percentage_Us<Percentage_Opposite)
            return -2;
        else if(Percentage_Us>IS_PROPERTION_2)
            return 2;
        else if(Percentage_Opposite>IS_PROPERTION_2)
            return -2;
        else if(Percentage_Us>BOTH_PROPERTION_2)
            return 1;
        else if(Percentage_Opposite>BOTH_PROPERTION_2)
            return -1;
        else
            return 100;
    }
    ////////////////
    ///Method 3rd///
    ////////////////
    else if(MethodFlag==3)
    {
        Mat r_bg = img_bgr[2]/(img_bgr[0]+img_bgr[1]);
        Mat b_gr = img_bgr[0]/(img_bgr[2]+img_bgr[1]);
        Mat r_cmp = r_bg > RED_BLUEandGREEN_LOW;
        //imshow("r_bg",r_bg);
        Mat b_cmp = b_gr > BLUE_REDandGREEN_LOW;
        float Percentage_Us;
        float Percentage_Opposite;
        if(Us_flag=='r')
        {
            int count = countNonZero(r_cmp);
            Percentage_Us = float(count)/(float)(img.rows*img.cols);
            count = countNonZero(b_cmp);
//            imshow("enemy",b_cmp);
            Percentage_Opposite = float(count)/(float)(img.rows*img.cols);
        }
        else if(Us_flag=='b')
        {
            int count = countNonZero(b_cmp);
            Percentage_Us = float(count)/(float)(img.rows*img.cols);
            count = countNonZero(r_cmp);
//            imshow("enemy",r_cmp);
            Percentage_Opposite = float(count)/(float)(img.rows*img.cols);
        }
        if(Percentage_Us>=IS_PROPERTION_3 && Percentage_Opposite<BOTH_PROPERTION_3)
            return 1;
        else if(Percentage_Us<BOTH_PROPERTION_3 && Percentage_Opposite>=IS_PROPERTION_3)
            return -1;
        else if(Percentage_Us<NOT_PROPERTION_3 && Percentage_Opposite<NOT_PROPERTION_3)
            return 0;
        else if(Percentage_Us>=BOTH_PROPERTION_3 && Percentage_Opposite>=BOTH_PROPERTION_3 && Percentage_Us>Percentage_Opposite)
            return 2;
        else if(Percentage_Us>=BOTH_PROPERTION_3 && Percentage_Opposite>=BOTH_PROPERTION_3 && Percentage_Us<Percentage_Opposite)
            return -2;
        else if(Percentage_Us>=IS_PROPERTION_3)
            return 2;
        else if(Percentage_Opposite>=IS_PROPERTION_3)
            return -2;
        else if(Percentage_Us>=BOTH_PROPERTION_3 && Percentage_Opposite<NOT_PROPERTION_3)
            return 1;
        else if(Percentage_Us<NOT_PROPERTION_3 && Percentage_Opposite>=BOTH_PROPERTION_3)
            return -1;
        else if(Percentage_Us<BOTH_PROPERTION_3 && Percentage_Opposite<BOTH_PROPERTION_3)
            return 0;
        else if(Percentage_Us<BOTH_PROPERTION_3 && Percentage_Opposite<BOTH_PROPERTION_3)
            return 0;
        else
            return 100;
    }
    ////////////////
    ///Method 4th///
    ////////////////
    else if(MethodFlag==4)
    {
        Mat img_;
        vector<Mat> img_hsv(img.channels());
        cvtColor(img,img_,COLOR_BGR2HSV);
        split(img_,img_hsv);
        int temp = img_hsv[0].at<unsigned char>(75,15);
        float Percentage_Us,Percentage_Opposite;
        Mat h_cmp = (img_hsv[0] > Us_hsv_low[0]) & (img_hsv[0] < Us_hsv_up[0]);
        Mat s_cmp = (img_hsv[1] > Us_hsv_low[1]) & (img_hsv[1] < Us_hsv_up[1]);
        Mat compare = h_cmp & s_cmp;
        Point point_cc;
        point_cc.x = -1;
        point_cc.y = -1;
        //image; polyfit; perimScale; num; countour center
//        vector<Rect> bbs_cc = ConnectedComponents(compare, 1, 1.0, 0, point_cc);
        int count = countNonZero(compare);
        Percentage_Us = float(count)/(float)(img.rows*img.cols);

        h_cmp = (img_hsv[0] > Opposite_hsv_low[0]) & (img_hsv[0] < Opposite_hsv_up[0]);
        s_cmp = (img_hsv[1] > Opposite_hsv_low[1]) & (img_hsv[1] < Opposite_hsv_up[1]);
//        imshow("Opposite h",h_cmp);
        compare = h_cmp & s_cmp;
//        imshow("in", compare);
//        bbs_cc = ConnectedComponents(compare, 1, 1000.0, 3, point_cc);    // 0: 采用多边形拟合处理
//        for (int i=0;i<bbs_cc.size();i++)
//            rectangle(compare, bbs_cc[i].tl(), bbs_cc[i].br(), Scalar(255, 255, 255), 1);
//        imshow("out", compare);
        count = countNonZero(compare);
        Percentage_Opposite = float(count)/(float)(img.rows*img.cols);

        if(Percentage_Us>=IS_PROPERTION_4 && Percentage_Opposite<BOTH_PROPERTION_4)
            return 1;
        else if(Percentage_Us<BOTH_PROPERTION_4 && Percentage_Opposite>=IS_PROPERTION_4)
            return -1;
        else if(Percentage_Us<NOT_PROPERTION_4 && Percentage_Opposite<NOT_PROPERTION_4)
            return 0;
        else if(Percentage_Us>=BOTH_PROPERTION_4 && Percentage_Opposite>=BOTH_PROPERTION_4 && Percentage_Us>Percentage_Opposite)
            return 2;
        else if(Percentage_Us>=BOTH_PROPERTION_4 && Percentage_Opposite>=BOTH_PROPERTION_4 && Percentage_Us<Percentage_Opposite)
            return -2;
        else if(Percentage_Us>=IS_PROPERTION_4)
            return 2;
        else if(Percentage_Opposite>=IS_PROPERTION_4)
            return -2;
        else if(Percentage_Us>=BOTH_PROPERTION_4 && Percentage_Opposite<NOT_PROPERTION_4)
            return 1;
        else if(Percentage_Us<NOT_PROPERTION_4 && Percentage_Opposite>=BOTH_PROPERTION_4)
            return -1;
        else if(Percentage_Us<BOTH_PROPERTION_4 && Percentage_Opposite<BOTH_PROPERTION_4)
            return 0;
        else if(Percentage_Us<BOTH_PROPERTION_4 && Percentage_Opposite<BOTH_PROPERTION_4)
            return 0;
        else
            return 100;
    }
    ////////////////
    ///Method 5th///
    ////////////////
    else if(MethodFlag==5)
    {
        Mat img_;
        vector<Mat> img_hsv(img.channels());
        cvtColor(img,img_,COLOR_BGR2HSV);
        split(img_,img_hsv);
        float Percentage_Us,Percentage_Opposite;
        Percentage_Us = ColorHSV(img_hsv[0],img_hsv[1],img_hsv[1],0);
        Percentage_Opposite = ColorHSV(img_hsv[0],img_hsv[1],img_hsv[1],1);

        if(Percentage_Us>=IS_PROPERTION_5 && Percentage_Opposite<BOTH_PROPERTION_5)
            return 1;
        else if(Percentage_Us<BOTH_PROPERTION_5 && Percentage_Opposite>=IS_PROPERTION_5)
            return -1;
        else if(Percentage_Us<NOT_PROPERTION_5 && Percentage_Opposite<NOT_PROPERTION_5)
            return 0;
        else if(Percentage_Us>=BOTH_PROPERTION_5 && Percentage_Opposite>=BOTH_PROPERTION_5 && Percentage_Us>Percentage_Opposite)
            return 2;
        else if(Percentage_Us>=BOTH_PROPERTION_5 && Percentage_Opposite>=BOTH_PROPERTION_5 && Percentage_Us<Percentage_Opposite)
            return -2;
        else if(Percentage_Us>=IS_PROPERTION_5)
            return 2;
        else if(Percentage_Opposite>=IS_PROPERTION_5)
            return -2;
        else if(Percentage_Us>=BOTH_PROPERTION_5 && Percentage_Opposite<NOT_PROPERTION_5)
            return 1;
        else if(Percentage_Us<NOT_PROPERTION_5 && Percentage_Opposite>=BOTH_PROPERTION_5)
            return -1;
        else if(Percentage_Us<BOTH_PROPERTION_5 && Percentage_Opposite<BOTH_PROPERTION_5)
            return 0;
        else if(Percentage_Us<BOTH_PROPERTION_5 && Percentage_Opposite<BOTH_PROPERTION_5)
            return 0;
        else
            return 100;
    }
}

//vector<Rect> ColorIdentify::ConnectedComponents(Mat mask_process, int poly1_hull0, float perimScale, int number, Point contour_centers)
//{
//    /*下面4句代码是为了兼容原函数接口，即内部使用的是c风格，但是其接口是c++风格的*/
////    IplImage *mask = mask_process.operator IplImage();
//    IplImage mask = mask_process;
//    int *num = &number;
////    CvRect *bbs = bounding_box.operator CvRect();
//    CvRect bbs;
//    vector<Rect> bbs_return;
////    CvPoint *centers = contour_centers.operator CvPoint();
//    CvPoint centers = contour_centers;
//    static CvMemStorage*    mem_storage    = NULL;
//    static CvSeq*            contours    = NULL;
//    //CLEAN UP RAW MASK
//        //开运算作用：平滑轮廓，去掉细节,断开缺口
//        cvMorphologyEx( &mask, &mask, NULL, NULL, CV_MOP_OPEN, 1 );//对输入mask进行开操作，CVCLOSE_ITR为开操作的次数，输出为mask图像
//        //闭运算作用：平滑轮廓，连接缺口
//        cvMorphologyEx( &mask, &mask, NULL, NULL, CV_MOP_CLOSE, 1 );//对输入mask进行闭操作，CVCLOSE_ITR为闭操作的次数，输出为mask图像
//    //FIND CONTOURS AROUND ONLY BIGGER REGIONS
//        if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
//            else cvClearMemStorage(mem_storage);
//        //CV_RETR_EXTERNAL=0是在types_c.h中定义的，CV_CHAIN_APPROX_SIMPLE=2也是在该文件中定义的
//        CvContourScanner scanner = cvStartFindContours(&mask,mem_storage,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
//        CvSeq* c;
//        int numCont = 0;
//        //该while内部只针对比较大的轮廓曲线进行替换处理
//        while( (c = cvFindNextContour( scanner )) != NULL )
//        {
//            double len = cvContourPerimeter( c );
//            double q = (mask.height + mask.width) /perimScale;   //calculate perimeter len threshold
//            if( len < q ) //Get rid of blob if it's perimeter is too small
//            {
//                cvSubstituteContour( scanner, NULL );    //用NULL代替原来的那个轮廓
//            }
//            else //Smooth it's edges if it's large enough
//            {
//                CvSeq* c_new;
//                if(poly1_hull0) //Polygonal approximation of the segmentation
//                    c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, 2,0);
//                else //Convex Hull of the segmentation
//                    c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
//                cvSubstituteContour( scanner, c_new ); //最开始的轮廓用凸包或者多项式拟合曲线替换
//                numCont++;
//            }
//        }
//        contours = cvEndFindContours( &scanner );    //结束轮廓查找操作
////        printf("elem_size: %d  %d\n",contours->elem_size,contours->header_size);

//        // PAINT THE FOUND REGIONS BACK INTO THE IMAGE
//        cvZero( &mask );
//        IplImage *maskTemp;
//        //CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
//        if(*num != 0)
//        {
//            int N = *num, numFilled = 0, i=0;
//            CvMoments moments;
//            double M00, M01, M10;
//            maskTemp = cvCloneImage(&mask);
//            for(i=0, c=contours; c != NULL; c = c->h_next,i++ )        //h_next为轮廓序列中的下一个轮廓
//            {
////                if(i < N) //Only process up to *num of them
//                {
//                    //CV_CVX_WHITE在本程序中是白色的意思
//                    cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
//                    //Find the center of each contour
//                    if (( centers.x != -1 )&&( centers.y != -1 ))
//                    {
//                        cvMoments(maskTemp,&moments,1);    //计算mask图像的最高达3阶的矩
//                        M00 = cvGetSpatialMoment(&moments,0,0); //提取x的0次和y的0次矩
//                        M10 = cvGetSpatialMoment(&moments,1,0); //提取x的1次和y的0次矩
//                        M01 = cvGetSpatialMoment(&moments,0,1); //提取x的0次和y的1次矩
//                        centers.x = (int)(M10/M00);    //利用矩的结果求出轮廓的中心点坐标
//                        centers.y = (int)(M01/M00);
//                    }
//                    //Bounding rectangles around blobs
////                      if ( (bbs.x != 0) && (bbs.y != 0) &&  (bbs.width != 0) &&  (bbs.height != 0) )
////                    {
//                        bbs = cvBoundingRect(c); //算出轮廓c的外接矩形
////                    }
//                    cvZero(maskTemp);
//                    numFilled++;
//                    Rect temp = Rect_<int> (bbs.x,bbs.y,bbs.width,bbs.height);
//                    bbs_return.push_back(temp);
//                }
//                //Draw filled contours into mask
//                cvDrawContours(&mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
//                printf("I:::::%d\n",i);
//            } //end looping over contours

//            *num = numFilled;
//            cvReleaseImage( &maskTemp);
//            printf("BBS: %d  %d  %d  %d\n",bbs.x,bbs.y,bbs.width,bbs.height);
////            Rect bbs_return = Rect_<int>(bbs.x,bbs.y,bbs.width,bbs.height);
//        }
//        //ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
//        else
//        {
//            for( c=contours; c != NULL; c = c->h_next )
//            {
//                cvDrawContours(&mask,c,CV_CVX_WHITE, CV_CVX_BLACK,-1,CV_FILLED,8);
//            }
//        }
//        return bbs_return;
//}

float ColorIdentify::ColorHSV(Mat h,Mat s,Mat v,int flag)
{
    Mat h_float,s_float,v_float;
    h.convertTo(h_float,CV_32F,1,0);
    s.convertTo(s_float,CV_32F,1,0);
    v.convertTo(v_float,CV_32F,1,0);
    float percentage;
    int count;
    if(flag==0)  //Us
    {
        Mat compare,h_estimate;
        if(Us_flag=='r')
        {
            h_estimate = 177.4 + 0.003104*v_float - 0.006403*s_float;
            compare = (h_estimate-h_float)>=-REDCAR_HUE_VAR & (h_estimate-h_float)<=REDCAR_HUE_VAR;
            count = countNonZero(compare);
            percentage = float(count)/(float)(h.rows*h.cols);
        }
        else if(Us_flag=='b')
        {
            h_estimate = 45.88 - 0.05265*v_float + 0.0417*s_float;
            compare = (h_estimate-h_float)>=-BLUECAR_HUE_VAR & (h_estimate-h_float)<=BLUECAR_HUE_VAR;
            count = countNonZero(compare);
            percentage = float(count)/(float)(h.rows*h.cols);
        }
        return percentage;
    }
    else if(flag==1)  //Opposite
    {
        Mat compare,h_estimate;
        if(Us_flag=='r')
        {
            h_estimate = 45.88 - 0.05265*v_float + 0.0417*s_float;
            compare = (h_estimate-h_float)>=-BLUECAR_HUE_VAR & (h_estimate-h_float)<=BLUECAR_HUE_VAR;
            count = countNonZero(compare);
            percentage = float(count)/(float)(h.rows*h.cols);
        }
        else if(Us_flag=='b')
        {
            h_estimate = 177.4 + 0.003104*v_float - 0.006403*s_float;
            compare = (h_estimate-h_float)>=-REDCAR_HUE_VAR & (h_estimate-h_float)<=REDCAR_HUE_VAR;
            count = countNonZero(compare);
            percentage = float(count)/(float)(h.rows*h.cols);
        }
        return percentage;
    }
}
