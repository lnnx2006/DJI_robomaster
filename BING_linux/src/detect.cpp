#include "detect.h"
detect::~detect()
{

}
//vector<Rect> detect::getTargetRect()
vector<Rect> detect::getTargetRect()
{
//    printf("LIHAOYU 00000");
    return found_rect;
}
//vector<Point> detect::getTargetPosition()
vector<Point> detect::getTargetPosition()
{
    return found_position;
}
//vector<Point2f> detect::getTargetAngular()
vector<Point2f> detect::getTargetAngular()
{
    return found_angular;
}
vector<int> detect::getTargetClassifier()
{
    return found_classifier;
}

void detect::detectCar_Hog(vector<Mat> Images)
{
    if(found_position.size()>0)
        found_position.clear();
    if(found_rect.size()>0)
        found_rect.clear();
    if(found_classifier.size()>0)
        found_classifier.clear();
    int img_num = Images.size();
    vector<Rect> found_;
    Point point;
    //int count=0;
    for(int i=0;i<img_num;i++)
    {
        this->detectMultiScale(Images[i],found_);
        /*for(int j=0;j<found_.size();j++)
        {
            point.x = found_[j].x + (int)found_[j].width/2 - center.x;
            point.y = found_[j].y + (int)found_[j].height/2 - center.y;
            //found_position.at<float>(count,0) = found_[j].x - center.x;
            //found_position.at<float>(count,1) = found_[j].y - center.y;
            found_position.push_back(point);
            //point.x = found_[j].tl;
            //point.y = found_[j].br;
            found_rect.push_back(found_[j]);
        }*/
        for(int p=0;p<found_.size();p++)
        {
            Rect a = found_[p];
//            if(p==14)
                rectangle(Images[i], a.tl(), a.br(), Scalar(0, 255, 255), 3);
        }
        imshow("Image_found",Images[i]);
        for(int j=0;j<found_.size();j++)
        {
            Rect r = found_[j];
            int k;
            for(k=0;k<found_.size();k++)
            {
                if(k!=j && (r & found_[k])==r)
                    break;
            }
            if(k==found_.size())
            {
                int classifier = this->Identify(Images[i],r);
                if(classifier<0)
                {
                    found_classifier.push_back(classifier);
                    found_rect.push_back(r);
                    point.x = r.x + (int)r.width/2 - center.x;
                    point.y = r.y + (int)r.height/2 - center.y;
                    found_position.push_back(point);
                }
            }
        }
    }
    CaculateTargetAngular();
}

void detect::CaculateTargetAngular()
{
    if(found_angular.size()>0)
        found_angular.clear();
    int num = found_position.size();
    float scale_x = 90/(float)ImgWidth;
    float scale_y = 90/(float)ImgHeight;
    Point2f p;
    for(int i=0;i<num;i++)
    {
        p.x = scale_x * found_position[i].x;
        p.y = scale_y * found_position[i].y;
        found_angular.push_back(p);
        //found_angular[i].x = scale_x * found_position[i].x;
        //found_angular[i].y = scale_y * found_position[i].y;
    }
}
void detect::setCenter(int  center_x,int center_y)
{
    center.x = center_x;
    center.y = center_y;
}
void detect::setImgAttributes(int center_x, int center_y, int Width, int Height)
{
    center.x = center_x;
    center.y = center_y;
    ImgWidth = Width;
    ImgHeight = Height;
}

void detect::setImg(Mat img)
{
    img.copyTo(image);
}

void detect::detectCar(vector<Rect> rect)
{
    if(found_position.size()>0)
        found_position.clear();
    if(found_rect.size()>0)
        found_rect.clear();
    if(found_classifier.size()>0)
        found_classifier.clear();
    selectRect = rect;
    int selectRect_num = selectRect.size();
    Mat img_temp;
    vector<Rect> found_,found_temp;
    Point point;
    for(int i=0;i<selectRect_num;i++)
    {
//        printf("STEP 3.4.1 OK!  %d %d %d %d %d\n",i,selectRect[i].x,selectRect[i].y,selectRect[i].width,selectRect[i].height);
        img_temp = image.rowRange(selectRect[i].y,selectRect[i].y+selectRect[i].height);
//        printf("STEP 3.4.2 OK!  %d %d %d %d %d\n",i,selectRect[i].x,selectRect[i].y,selectRect[i].width,selectRect[i].height);
        img_temp = img_temp.colRange(selectRect[i].x,selectRect[i].x+selectRect[i].width);
        this->detectMultiScale(img_temp,found_,0,cvSize(block_stride,block_stride),cvSize(0,0),1.1);
        for(int p=0;p<found_.size();p++)
        {
            Rect a = found_[p];
            Point tl,br;
            tl.x = selectRect[i].x+found_[p].x;
            tl.y = selectRect[i].y+found_[p].y;
            br.x = selectRect[i].x+found_[p].x+found_[p].width;
            br.y = selectRect[i].y+found_[p].y+found_[p].height;
            rectangle(image, tl, br, Scalar(0, 255, 255), 3);
        }
//        imshow("Image_found",image);
        for(int j=0;j<found_.size();j++)
        {

            Rect r = found_[j];
//            int k;
//            for(k=0;k<found_.size();k++)
//            {
//                if(k!=j && (r & found_[k])==r)
//                    break;
//            }
//            if(k==found_.size())
//            {
                int classifier = this->Identify(img_temp,r);
                if(classifier<0)
                {
                    r.x = selectRect[i].x + r.x;
                    r.y = selectRect[i].y + r.y;
                    found_temp.push_back(r);
                }
//            }
        }
        found_.clear();
    }
    for(int j=0;j<found_temp.size();j++)
    {
        Rect r = found_temp[j];
        int k;
        for(k=0;k<found_temp.size();k++)
        {
            if(k!=j &&(r & found_temp[k])==r)
            {
                break;
                k++;
            }
        }
        if(k==found_temp.size())
        {
            int classifier = this->Identify(image,r);
            if(classifier<0)
            {
                found_classifier.push_back(classifier);
                found_rect.push_back(r);
                point.x = r.x + (int)r.width/2 - center.x;
                point.y = r.y + (int)r.height/2 - center.y;
                found_position.push_back(point);
            }
        }
    }
    CaculateTargetAngular();
}

void detect::detectCarForColor(vector<Rect> rect)
{
    if(found_position.size()>0)
        found_position.clear();
    if(found_rect.size()>0)
        found_rect.clear();
    selectRect = rect;
    int selectRect_num = selectRect.size();
    Mat img_temp;
    vector<Rect> found_;
    Point point;
    for(int i=0;i<selectRect_num;i++)
    {
        img_temp = image.rowRange(selectRect[i].y,selectRect[i].y+selectRect[i].height);
        img_temp = img_temp.colRange(selectRect[i].x,selectRect[i].x+selectRect[i].width);
        this->detectMultiScale(img_temp,found_,0,cvSize(block_stride,block_stride),cvSize(0,0),1.1);
        for(int p=0;p<found_.size();p++)
        {
            Rect a = found_[p];
            Point tl,br;
            tl.x = selectRect[i].x+found_[p].x;
            tl.y = selectRect[i].y+found_[p].y;
            br.x = selectRect[i].x+found_[p].x+found_[p].width;
            br.y = selectRect[i].y+found_[p].y+found_[p].height;
            rectangle(image, tl, br, Scalar(0, 255, 255), 3);
        }
//        imshow("Image_found",image);
//        found_.clear();
    }
    for(int j=0;j<found_.size();j++)
    {
        Rect r = found_[j];
        int k;
        for(k=0;k<found_.size();k++)
        {
            if(k!=j &&(r & found_[k])==r)
            {
                break;
                k++;
            }
        }
        if(k==found_.size())
        {
//            if(float(r.area()/float(rect[0].area()))>=0.14)
//            {
                found_rect.push_back(r);
                point.x = r.x + (int)r.width/2 - center.x;
                point.y = r.y + (int)r.height/2 - center.y;
                found_position.push_back(point);
//            }
        }
    }
}

void detect::colordetection(Mat img, int xmin_box, int ymin_box)
{
    FoundSuspectedCar.clear();
//    Mat hsv3u;
//    cvtColor(img, hsv3u, CV_BGR2HSV);
//    const int H = img.rows, W = img.cols;

//    Mat h_cmp(H,W,CV_8UC1), s_cmp(H,W,CV_8UC1);
//    Mat compare(H,W,CV_8UC1);
//    for( size_t nrow = 0; nrow < H; nrow++)
//    {
//        for(size_t ncol = 0; ncol < W; ncol++)
//        {
//             Vec3i hsv_temp = hsv3u.at<Vec3b>(nrow,ncol);
//             if  ( (( hsv_temp.val[0] >= 165) &&  (hsv_temp.val[0] <= 180 )) && (( hsv_temp.val[1] >= 165) &&  (hsv_temp.val[1] <= 220 )) )
//                     compare.at<uchar>(nrow,ncol) = 255;
//             else
//                 compare.at<uchar>(nrow,ncol) = 0;
//        }
//     }

  // ================================
    Mat img_;
    vector<Mat> img_hsv(3);

    cvtColor( img, img_, COLOR_BGR2HSV );

    split(img_, img_hsv);

    const int H = img.rows, W = img.cols;

    Mat h_cmp(H,W,CV_8UC1), s_cmp(H,W,CV_8UC1);
    Mat compare(H,W,CV_8UC1);

    // Print the whole Image
//    for( size_t nrow = 0; nrow < img_.rows; nrow++)
//     {
//        for(size_t ncol = 0; ncol < img_.cols; ncol++)
//        {
//            Vec3i hsv = img_.at<Vec3b>(nrow,ncol);//用Vec3b也行
//            cout   << "("<<hsv.val[0]<<","
//                    <<hsv.val[1]<<","
//                    <<hsv.val[2]<<")";
//        }
//        cout << endl;
//     }


//    Mat h_cmp; Mat s_cmp;
//    h_cmp = (img_hsv[0] > Us_hsv_low[0]) & (img_hsv[0] < Us_hsv_up[0]);
//    s_cmp = (img_hsv[1] > Us_hsv_low[1]) & (img_hsv[1] < Us_hsv_up[1]);
//    Mat compare = h_cmp & s_cmp;
//    int count = countNonZero(compare);
//    Percentage_Us = float(count)/(float)(img.rows*img.cols);

//    h_cmp = ( (img_hsv[0] > Opposite_hsv_low[0]) & (img_hsv[0] < Opposite_hsv_up[0]) ) ;
//    | ( (img_hsv[0] > Opposite_hsv_low[3]) & (img_hsv[0] < Opposite_hsv_up[3]) );
//    s_cmp = ( img_hsv[1] >=  Opposite_hsv_low[1] ) & (img_hsv[1] <= Opposite_hsv_up[1]);

//    Mat cmp1, cmp2, cmp3;
//    cmp1 = ( (img_hsv[0] >= 171) & (img_hsv[0] <= 180 ) ) & ( (img_hsv[1] >= 50) & (img_hsv[1] <= 200 ) );
//    cmp2 = ( (img_hsv[0] >= 173) & (img_hsv[0] <= 179 ) ) & ( (img_hsv[1] >= 200) & (img_hsv[1] <= 225 ) );
//    cmp3 = ( (img_hsv[0] >= 175) & (img_hsv[0] <= 178 ) ) & ( (img_hsv[1] >= 225) & (img_hsv[1] <= 255 ) );

//    cmp1 = ( (img_hsv[0] >= 155) & (img_hsv[0] <= 180 ) ) & ( (img_hsv[1] >= 12) & (img_hsv[1] <= 255 ) ) & ( (img_hsv[2] >= 55) & (img_hsv[2] <= 255 ) ) ;
//    cmp1 = ( (img_hsv[0] >= 155) & (img_hsv[0] <= 180 ) ) ;
//    cmp2 = ( (img_hsv[1] >= 130) & (img_hsv[1] <= 210 ) ) ;
//    & ( (img_hsv[2] >= 120) & (img_hsv[2] <= 240 ) ) ;


    h_cmp = (img_hsv[0] > Opposite_hsv_low[0]) & (img_hsv[0] < Opposite_hsv_up[0]);
    s_cmp = (img_hsv[1] > Opposite_hsv_low[1]) & (img_hsv[1] < Opposite_hsv_up[1]);
    compare = h_cmp & s_cmp;



//    Mat compare = h_cmp & s_cmp;
//    Mat compare = h_cmp;
//    imshow("in", compare);

//    for( size_t nrow = 0; nrow < compare.rows; nrow++)
//     {
//        for(size_t ncol = 0; ncol < compare.cols; ncol++)
//        {
//            int hsv = compare.at<uchar>(nrow,ncol);//用Vec3b也行
//            cout   << " "<< hsv <<" ";
//        }
//        cout << endl;
//     }


    Point point_cc;
    point_cc.x = -1;
    point_cc.y = -1;

    //    int  pos_min = *min_element(bbs_cc.begin(),bbs_cc[].end());  //不要省略 “*” 求最大值

    int pos_xmin = 1000;
    int pos_xmax = 0;
    int pos_ymin = 1000;
    int pos_ymax = 0;

    vector<Rect> bbs_cc = ConnectedComponents(compare, 0, 10.0, 1, point_cc);    // 0: 采用多边形拟合处理

//            for (int i=0;i<bbs_cc.size();i++)
//                rectangle(compare, bbs_cc[i].tl(), bbs_cc[i].br(), Scalar(255, 255, 255), 1);

//    imshow("out_cc", compare);

//    cvWaitKey(1000);

    if (bbs_cc.size())
    {
//        printf("*********%d*********\n",bbs_cc.size());
        for (int i=0;i< bbs_cc.size();i++)
        {
            if( bbs_cc[i].x < pos_xmin)
                pos_xmin = bbs_cc[i].x ;
            if( bbs_cc[i].y < pos_ymin)
                pos_ymin = bbs_cc[i].y ;
            if( (bbs_cc[i].x + bbs_cc[i].width) > pos_xmax )
                pos_xmax =  bbs_cc[i].x + bbs_cc[i].width ;
            if( (bbs_cc[i].y + bbs_cc[i].height ) > pos_ymax )
                pos_ymax =  bbs_cc[i].y + bbs_cc[i].height ;
        }
//        rectangle(image, cvPoint(pos_xmin + xmin_box, pos_ymin + ymin_box), cvPoint(pos_xmax + xmin_box, pos_ymax + ymin_box), Scalar(255, 255, 255), 1);
//          rectangle(image, cvPoint(xmin_box, ymin_box), cvPoint(xmin_box + img_.cols, ymin_box + img_.rows), Scalar(255, 255, 255), 1);
        FoundSuspectedCar = bbs_cc;
        FoundSuspectedRegion = Rect_<int> ( pos_xmin, pos_ymin, pos_xmax - pos_xmin, pos_ymax - pos_ymin);
    }

    bbs_cc.clear();

//    for (int i=0;i<bbs_cc.size();i++)
//        rectangle(compare, bbs_cc[i].tl(), bbs_cc[i].br(), Scalar(255, 255, 255), 1);
//    imshow("out", image);

}

void detect::ColordetectionBingBoxes(vector<Rect> rect)
{
    selectRect = rect;
    int selectRect_num = selectRect.size();
    Mat img_temp;

    for(int i=0;i<selectRect_num;i++)
    {
        img_temp = image.rowRange( selectRect[i].y, selectRect[i].y + selectRect[i].height);
        img_temp = img_temp.colRange( selectRect[i].x, selectRect[i].x + selectRect[i].width );
        colordetection(img_temp,selectRect[i].x,selectRect[i].y);
    }

}

vector<Rect> detect::ColorFoundSuspectedCar()
{
    return FoundSuspectedCar;
}

Rect detect::ColorFoundSuspectedRegion()
{
    return FoundSuspectedRegion;
}

vector<Rect> detect::ConnectedComponents(Mat mask_process, int poly1_hull0, float perimScale, int number, Point contour_centers)
{
    /*下面4句代码是为了兼容原函数接口，即内部使用的是c风格，但是其接口是c++风格的*/
//    IplImage *mask = mask_process.operator IplImage();
    IplImage mask = mask_process;
    int *num = &number;
//    CvRect *bbs = bounding_box.operator CvRect();
    CvRect bbs;
    vector<Rect> bbs_return;
//    CvPoint *centers = contour_centers.operator CvPoint();
    CvPoint centers = contour_centers;
    static CvMemStorage*    mem_storage    = NULL;
    static CvSeq*            contours    = NULL;
    //CLEAN UP RAW MASK
        //开运算作用：平滑轮廓，去掉细节,断开缺口
//        cvMorphologyEx( &mask, &mask, NULL, NULL, CV_MOP_OPEN, 1 );//对输入mask进行开操作，CVCLOSE_ITR为开操作的次数，输出为mask图像
        //闭运算作用：平滑轮廓，连接缺口
        cvMorphologyEx( &mask, &mask, NULL, NULL, CV_MOP_CLOSE, 1 );//对输入mask进行闭操作，CVCLOSE_ITR为闭操作的次数，输出为mask图像
    //FIND CONTOURS AROUND ONLY BIGGER REGIONS
        if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
            else cvClearMemStorage(mem_storage);
        //CV_RETR_EXTERNAL=0是在types_c.h中定义的，CV_CHAIN_APPROX_SIMPLE=2也是在该文件中定义的
        CvContourScanner scanner = cvStartFindContours(&mask,mem_storage,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        CvSeq* c;
        int numCont = 0;
        //该while内部只针对比较大的轮廓曲线进行替换处理
        while( (c = cvFindNextContour( scanner )) != NULL )
        {
            double len = cvContourPerimeter( c );
//            double q = (mask.height + mask.width) / perimScale;   //calculate perimeter len threshold
            double q = ( IMAGE_WIDTH + IMAGE_HEIGHT ) / perimScale;
            if( len < q ) //Get rid of blob if it's perimeter is too small
            {
                cvSubstituteContour( scanner, NULL );    //用NULL代替原来的那个轮廓
            }
            else //Smooth it's edges if it's large enough
            {
                CvSeq* c_new;
                if(poly1_hull0) //Polygonal approximation of the segmentation
                    c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, 2,0);
                else //Convex Hull of the segmentation
                    c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
                cvSubstituteContour( scanner, c_new ); //最开始的轮廓用凸包或者多项式拟合曲线替换
                numCont++;
            }
        }
        contours = cvEndFindContours( &scanner );    //结束轮廓查找操作
//        printf("elem_size: %d  %d\n",contours->elem_size,contours->header_size);

        // PAINT THE FOUND REGIONS BACK INTO THE IMAGE
        cvZero( &mask );
        IplImage *maskTemp;
        //CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
        if(*num != 0)
        {
            int N = *num, numFilled = 0, i=0;
            CvMoments moments;
            double M00, M01, M10;
            maskTemp = cvCloneImage(&mask);
            for(i=0, c=contours; c != NULL; c = c->h_next,i++ )        //h_next为轮廓序列中的下一个轮廓
            {
//              if(i < N) //Only process up to *num of them
//                {
                    //CV_CVX_WHITE在本程序中是白色的意思
                    cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
                    //Find the center of each contour
                    if (( centers.x != -1 )&&( centers.y != -1 ))
                    {
                        cvMoments(maskTemp,&moments,1);    //计算mask图像的最高达3阶的矩
                        M00 = cvGetSpatialMoment(&moments,0,0); //提取x的0次和y的0次矩
                        M10 = cvGetSpatialMoment(&moments,1,0); //提取x的1次和y的0次矩
                        M01 = cvGetSpatialMoment(&moments,0,1); //提取x的0次和y的1次矩
                        centers.x = (int)(M10/M00);    //利用矩的结果求出轮廓的中心点坐标
                        centers.y = (int)(M01/M00);
                    }
                    //Bounding rectangles around blobs
//                      if ( (bbs.x != 0) && (bbs.y != 0) &&  (bbs.width != 0) &&  (bbs.height != 0) )
//                    {
                        bbs = cvBoundingRect(c); //算出轮廓c的外接矩形
//                    }
                    cvZero(maskTemp);
                    numFilled++;
                    Rect temp = Rect_<int> (bbs.x,bbs.y,bbs.width,bbs.height);


                    // Height/Width Rate
                    if ( float(float(bbs.height)/float(bbs.width)) <= 1.1 )
                    {
//                        printf("Height/Width Rate: %f\n", float(float(bbs.height)/float(bbs.width)));
                        bbs_return.push_back(temp);
                        //Draw filled contours into mask
                        cvDrawContours(&mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
                    }
//                printf("I:::::%d\n",i);
            } //end looping over contours

            *num = numFilled;
            cvReleaseImage( &maskTemp);
//            printf("BBS: %d  %d  %d  %d\n",bbs.x,bbs.y,bbs.width,bbs.height);
//            Rect bbs_return = Rect_<int>(bbs.x,bbs.y,bbs.width,bbs.height);
        }
//        //ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
//        else
//        {
//            for( c=contours; c != NULL; c = c->h_next )
//            {
//                cvDrawContours(&mask,c,CV_CVX_WHITE, CV_CVX_BLACK,-1,CV_FILLED,8);
//            }
//        }
        return bbs_return;
}
