#include "vision_control/ConfidentMapConvex.h"


#include <QDebug>

namespace ImFusion {
namespace vision_control {

ConfidentMapConvex::ConfidentMapConvex()
: m_image(nullptr)
, m_output(nullptr)
{
//    configureDefaults();
}

ConfidentMapConvex::~ConfidentMapConvex()
{
    delete m_output;
}

bool ConfidentMapConvex::createCompatible(const DataList &data, Algorithm **a)
{
    if (1 == data.size())
        {
        if (1 == data[0]->kind())
            {
            SharedImageSet* img = dynamic_cast<SharedImageSet*>(data[0]);
            if (img)
                {
                if (a)
//                    *a = new ConfidentMapConvex();
                return true;
            }
        }
    }
    return false;
}

void ConfidentMapConvex::compute()
{
    delete m_output;
    m_output = new SharedImageSet();

}

void ConfidentMapConvex::output(DataList &dataOut)
{
    dataOut.add(m_output);
    m_output = nullptr;
}

void ConfidentMapConvex::configure(const Properties *p)
{
    if (!p)
        return;
    p->param("paramVec", m_paramVec);
    p->param("firstParameter", m_paramDouble);
}

void ConfidentMapConvex::configure(Properties *p) const
{
    if(!p)
        return;
    p->setParam("firstParameter", m_paramDouble);
    p->setParam("paramVec", m_paramVec, vec2(1, 0));
}

DataList ConfidentMapConvex::readImage_IMF(const QString& szFilePath, const QString& szFileName)
{
    QString szFile;
    if("/" == szFilePath.right(1))
        szFile = szFilePath + szFileName;
    else
        szFile = szFilePath + "/" + szFileName;
    std::string szFile_STD = szFile.toUtf8().constData();
    ImFusionFile ImageFileReader(szFile_STD);
    DataList USImageData;
    if (ImageFileReader.load(USImageData))
    {
        LOG_INFO("successful");
    } else {
        LOG_INFO("false");
    }
    return USImageData;

}

////////////////// openCV Part beginning

//use opencv function to read jpeg image (png version has problem with libpng)
void ConfidentMapConvex::readImage()
{
    std::cerr << cv::getBuildInformation() << std::endl;
    cv::Mat image=cv::imread("/home/zhongliang/Project/Pro_OrientationAdj/Data/sample.jpeg", 0);
    if (!image.data) {std::cerr << "Could not open or find the image" << std::endl;}

    //  2.1 Display a Picture
    qDebug()<<"enter into readImage";
    cv::Mat img = cv::imread("/home/zhongliang/Project/Pro_OrientationAdj/Data/sweep_28_10_18_39_01.png",  cv::IMREAD_COLOR);
//    cv::Mat img = cv::imread("/home/zhongliang/Project/Pro_OrientationAdj/Data/sample.jpeg",  cv::IMREAD_COLOR);
    qDebug()<<"111";
    if(img.empty()) {
        qDebug()<<"222";
        return;
    }
    qDebug()<<"333";
    cv::namedWindow("Eample1", cv::WINDOW_AUTOSIZE);
    qDebug()<<"222";
    cv::imshow("Eample1", img);
    qDebug()<<"333";
    cv::waitKey(0);
    cv::destroyWindow("Eample1");
}

void ConfidentMapConvex::saveImageToDisc(MemImage *memImage)
{
    int nImageHeight = memImage->height();
    int nImageWidth = memImage->width();
    qDebug()<<nImageHeight<<"       "<<nImageWidth;
    cv::Mat cv_imgIn;
    cv_imgIn = cv::Mat(nImageHeight, nImageWidth, CV_8UC1);

    if (memImage->byteSize() == cv_imgIn.total() * cv_imgIn.elemSize())
        {
        memcpy(cv_imgIn.data, memImage->data(), memImage->byteSize());
    }
    else
        {
        LOG_ERROR("onSaveImageClick: byteSize is different");
        qDebug()<<"memImage->byteSize():"<<memImage->byteSize();
        qDebug()<<"cv_imgIn->byteSize():"<<cv_imgIn.total() * cv_imgIn.elemSize();
    }
    cv::imwrite("/home/zhongliang/yuangao/download/test_saved.jpeg", cv_imgIn);   //good, the croped one
//    BackgroundExporter* sweepExporter = new BackgroundExporter();
//    sweepExporter->save(sharedImage_out, "/home/zhongliang/Project/Pro_OrientationAdj/test2.jpeg");

}



std::vector<cv::Point2f> ConfidentMapConvex::refineFan(cv::Point centerPoint, int nshortRadius, int nlongRadius)
{

    qDebug()<<"---------------------------------------------";
    qDebug()<<"enteronRefineFanClick";
    std::vector<int> vecRadius;   //float
    for (int i = nshortRadius; i<=nlongRadius; i++)
        {
        vecRadius.push_back(i);
    }

    //rotate the line to construct the matrix
    std::vector<cv::Point2f> vecWholePosition = std::vector<cv::Point2f>();
    cv::Point2f point_temp;

    int ncols = floor(fConvexAngle/fAngleInterval);
    float fStartAng = PI/2.0 - fConvexAngle/2.0;

    for (int i = 0; i<=ncols; i++)
        {
        for (unsigned j = 0; j < vecRadius.size(); j++)
        {
            point_temp.x = centerPoint.x + vecRadius.at(j) * cos(fStartAng + i*fAngleInterval);
            point_temp.y = 616-centerPoint.y + vecRadius.at(j) * sin(fStartAng + i*fAngleInterval);
            vecWholePosition.push_back(point_temp);
        }
    }

    std::cout << ".........................." << std::endl;
    std::cout<<vecWholePosition.at(100).x<<"        "<<vecWholePosition.at(100).y<<std::endl;
    std::cout<<vecWholePosition.at(101).x<<"        "<<vecWholePosition.at(101).y<<std::endl;
    std::cout<<vecWholePosition.at(102).x<<"        "<<vecWholePosition.at(102).y<<std::endl;
    std::cout<<vecWholePosition.at(103).x<<"        "<<vecWholePosition.at(103).y<<std::endl;
    std::cout<<vecWholePosition.at(104).x<<"        "<<vecWholePosition.at(104).y<<std::endl;
    std::cout<<vecWholePosition.at(105).x<<"        "<<vecWholePosition.at(105).y<<std::endl;
    std::cout<<vecWholePosition.at(106).x<<"        "<<vecWholePosition.at(106).y<<std::endl;

    //make it to be Integer
    for (unsigned i = 0; i < vecWholePosition.size(); i++)
    {
        vecWholePosition.at(i).x = round( vecWholePosition.at(i).x);
        vecWholePosition.at(i).y = round( vecWholePosition.at(i).y);
    }

    std::cout<<vecWholePosition.at(100).x<<"        "<<vecWholePosition.at(100).y<<std::endl;
    std::cout<<vecWholePosition.at(101).x<<"        "<<vecWholePosition.at(101).y<<std::endl;
    std::cout<<vecWholePosition.at(102).x<<"        "<<vecWholePosition.at(102).y<<std::endl;
    std::cout<<vecWholePosition.at(103).x<<"        "<<vecWholePosition.at(103).y<<std::endl;
    std::cout<<vecWholePosition.at(104).x<<"        "<<vecWholePosition.at(104).y<<std::endl;
    std::cout<<vecWholePosition.at(105).x<<"        "<<vecWholePosition.at(105).y<<std::endl;
    std::cout<<vecWholePosition.at(106).x<<"        "<<vecWholePosition.at(106).y<<std::endl;

    m_vecPointPosition = vecWholePosition;

    std::cout << "type: " << getCvMatRaw().type() << "," << "channels:"<<getCvMatRaw().channels() << std::endl;
    std::cout << "Size: " << getCvMatRaw().rows << ", " << getCvMatRaw().cols << std::endl;

    ///////////////////////////mark the area as white
//    //mark center line in white
//    int nStartLength = getHeightRaw()-centerPoint.y + nshortRadius;   //616-853+324
//    int nEndLength = getHeightRaw()-centerPoint.y + nlongRadius;   //616-853+823
//    for(int i = nStartLength; i< nEndLength; i++)
//    {
//        getCvMatRaw().at<uchar>(i, centerPoint.x) = 255;    //(i, 420)
//    }

//    //mark fan area as white area
//    for (unsigned i = 0; i < vecWholePosition.size(); i++)
//    {
////        std::cout << "[" << vecWholePosition.at(i).y << ", " << vecWholePosition.at(i).x << "]" << std::endl;
//        getCvMatRaw().at<uchar>(vecWholePosition.at(i).y, vecWholePosition.at(i).x) = 255;

//    }
    ///////////////////////////////

    /////////////refine the fan area to a new image
    cv::Mat matCatesian(getHeightRaw(), getWidthRaw(), CV_8UC1, cv::Scalar(255));
    for (unsigned i = 0; i < vecWholePosition.size(); i++)
    {
        matCatesian.at<uchar> (vecWholePosition.at(i).y, vecWholePosition.at(i).x) =
                getCvMatRaw().at<uchar>(vecWholePosition.at(i).y, vecWholePosition.at(i).x);

    }

    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_refined.jpeg", matCatesian);

    return vecWholePosition;

}

cv::Mat ConfidentMapConvex::polarToCatesian(cv::Point centerPoint, int nshortRadius, int nlongRadius)
{
    int nHeightCar = nLongRadius - nShortRadius +1;
    int nWidthCar = floor(fConvexAngle/fAngleInterval);
    cv::Mat matCatesian(nHeightCar, nWidthCar, CV_8UC1, cv::Scalar(255));   //rows, cols

//    cv::flip(getCvMatRaw(), getCvMatRaw(), 0);
    std::vector<cv::Point2f> vecWholePosition = refineFan(centerPoint, nshortRadius, nlongRadius);

    for (int i = 0; i < nWidthCar; i++)
    {
        for (int j = 0; j<nHeightCar; j++)
            matCatesian.at<uchar>(j, i) =
                    getCvMatRaw().at<uchar>(vecWholePosition.at(j + i*(nlongRadius-nShortRadius+1)).y, vecWholePosition.at(j+i*(nlongRadius-nShortRadius+1)).x);
    }
    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_cartesian.jpeg", matCatesian);
    return matCatesian;

}

DataList* ConfidentMapConvex::computeConfiMap(SharedImageSet* sharedImageSet)
{
    USConfidenceMapAlgorithm* confidenceAlg = new USConfidenceMapAlgorithm (sharedImageSet);

//    confidenceAlg->configure();
    Properties* properties = new Properties("confiMapProperties");

    confidenceAlg->configuration(properties);
//    std::cout<<"name of prop:"<<properties->name()<<std::endl;
//    std::cout<<"type of prop:"<<properties->type()<<std::endl;
    double dWorkingResolution(0.0);
    double UseSuperpixels(0.0);
    bool FastMode(true);
    double Adaptiveness(0.0);

    //Querying default parameters
    properties->param("Working Resolution", dWorkingResolution);
    properties->param("Use Superpixels", UseSuperpixels);
    properties->param("Fast Mode", FastMode);
    properties->param("Adaptiveness", Adaptiveness);

//    qDebug()<<"dWorkingResolution:"<<dWorkingResolution;
//    qDebug()<<"UseSuperpixels:"<<UseSuperpixels;
//    qDebug()<<"FastMode:"<<FastMode;
//    qDebug()<<"Adaptiveness:"<<Adaptiveness;

    //setting parameters
    properties->setParam("Working Resolution", 1.5);
    properties->setParam("Fast Mode", true);
    properties->setParam("Adaptiveness", 100.0);

    confidenceAlg->configure(properties);
    confidenceAlg->configuration(properties);

    //Querying default parameters
    properties->param("Working Resolution", dWorkingResolution);
    properties->param("Use Superpixels", UseSuperpixels);
    properties->param("Fast Mode", FastMode);
    properties->param("Adaptiveness", Adaptiveness);

//    qDebug()<<"dWorkingResolution:"<<dWorkingResolution;
//    qDebug()<<"UseSuperpixels:"<<UseSuperpixels;
//    qDebug()<<"FastMode:"<<FastMode;
//    qDebug()<<"Adaptiveness:"<<Adaptiveness;


    if (properties->empty())
        {
        qDebug()<<"the properties is empty";
    }

    std::cout<<"start to compute the confidence map"<<std::endl;

//    //set  progress start
//    Progress* progress = confidenceAlg->progress();
//    qDebug()<<"crate progress";
//    confidenceAlg->setProgress(confidenceAlg->progress());
//    qDebug()<<"set progress";
//    confidenceAlg->progress()->setProgressUpdateDisplay(true);
//    qDebug()<<"display it";
//    confidenceAlg->progress()->progressUpdateDisplay();
//    qDebug()<<"display it";
//    //set progress end

//    SharedImage* sharedImage = sharedImageSet->get();
//    SharedImage* sharedImageConfi = confidenceAlg->computeConfidenceMap(sharedImage);


    confidenceAlg->compute();

    DataList* dataListTemp = new DataList();

    confidenceAlg->output(*dataListTemp);
    std::cout << "Conficente output size:" <<dataListTemp->size() <<std::endl;
    return dataListTemp;
}


//transfer from catesian to polar
cv::Mat ConfidentMapConvex::catesianToPolar(cv::Point centerPoint, int nshortRadius, int nlongRadius)
{
    //problem about the readed image is 4 times than one
//    QString szFilePath = "/home/zhongliang/Project/Pro_OrientationAdj/";
//    QString szFileName = "test_confidencemap.imf";
//    DataList USImageData = readImage_IMF(szFilePath, szFileName);
//    SharedImageSet* sharedImage = USImageData.getImage(Data::UNKNOWN);
//    qDebug()<<"sharedImage kind:"<<sharedImage->kind();
//    qDebug()<<"sharedImage Modality:"<<sharedImage->modality();
//    MemImage* memImage = sharedImage->mem()->clone();

//    int nImageHeight, nImageWidth;
//    nImageHeight = sharedImage->mem()->height();
//    nImageWidth = sharedImage->mem()->width();
//    qDebug()<<nImageHeight<<"       "<<nImageWidth;
//    cv::Mat cv_imgIn, cv_imgRef, cv_imgOut;
//    cv_imgIn = cv::Mat(nImageHeight+100, nImageWidth+100, CV_8UC1);
////    memcpy(cv_imgIn.data, memImage->data(), memImage->byteSize());

    cv::Mat cvImageCar=cv::imread("/home/zhongliang/Project/Pro_OrientationAdj/test_confiMap.jpeg", 0);
//    cv::flip(cvImageCar, cvImageCar, 0);
    qDebug()<<"cvImageCatesian->byteSize():"<<cvImageCar.total() * cvImageCar.elemSize();

    std::vector<cv::Point2f> vecWholePosition = refineFan(centerPoint, nShortRadius, nlongRadius);

    int nHeightRaw = getHeightRaw();
    int nWidthRaw = getWidthRaw();
    cv::Mat cvImagePolar = cv::Mat(nHeightRaw, nWidthRaw, CV_8UC1, cv::Scalar(0));   //destination

    for(int i = 0; i < cvImageCar.cols; i++)
        {
        for(int j = 0; j< cvImageCar.rows; j++)
            {
            cvImagePolar.at<uchar>(vecWholePosition.at(j+i*(nlongRadius-nshortRadius+1)).y, vecWholePosition.at(j + i*(nlongRadius-nShortRadius+1)).x) =
                    cvImageCar.at<uchar>(j, i);
        }
    }

    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_backPolar.jpeg", cvImagePolar);

    return cvImagePolar;
}





////////////////// openCV Part ending


////////////////////////// control part
void ConfidentMapConvex::computeBaryCenter(cv::Mat& cvImage)
{
    //transer the image data from 0-255 to 0-1
    int nHeight = cvImage.rows;
    int nWidth = cvImage.cols;
    cv::Mat cvImage32(nHeight, nWidth, CV_32FC1);
    cvImage.convertTo(cvImage32, CV_32FC1, 1.0/255.0);

    std::cout<<"cvImage32.at<float>(j, i):"<<cvImage32.at<float>(300, 100)<<std::endl;
    cvImage32.at<float>(300, 100) = 1.0;
    //start to compute the data

    double dNumerator(0.0);
    double dDenominator(0.0);
    for (int i =0; i < nWidth; i++)  //theta
        {
        for (int j = 0; j < nHeight/5; j++)   //radiu
            {
            double m = ((i+1)*1.0/nWidth);
            dDenominator += cvImage32.at<float>(j, i)* 1/nHeight * 1/nWidth;
//            dDenominator += 1.0* 1.0/nHeight * 1.0/nWidth;
        }
    }

    for (int i = 0; i < nWidth; i++)  //theta, cols
        {
        for (int j = 0; j < nHeight/5; j++)   //radiu, rows,
            {
//            dNumerator += (i-round(nWidth/2.0))/nWidth * cvImage32.at<float>(j, i)* 1/nHeight * 1/nWidth;
            dNumerator += (i-round(nWidth/2.0))/nWidth * cvImage32.at<float>(j, i)* 1.0/nHeight * 1.0/nWidth;
        }
    }

    double dFeature = dNumerator/dDenominator *2;   //feature = i -round(nWidth/2)

    std::cout<<"dFeature:"<<dFeature<<std::endl;
    std::cout<<"dNumerator:"<<dNumerator<<std::endl;
    std::cout<<"dDenominator:"<<dDenominator<<std::endl;

    double dAngle = dFeature * fConvexAngle/PI * 180;


    m_dConfidenceAng = dAngle;

    std::cout<<"dAngle:"<<dAngle<<std::endl;

    //draw the centrel line of probe and the barycenter
    //////this works for retangle figure
//    int n_baryWidth(0);
//    cv::Mat cvColorImage;

//    cvtColor(cvImage,cvColorImage,CV_GRAY2RGB);
//    n_baryWidth = round((dFeature + 0.5) * nWidth);
//    for (int j = 0; j < nHeight; j++)   //radiu, rows,
//        {
////        cvColorImage.at<int>(j, n_baryWidth) = 0;
//        cvColorImage.at<cv::Vec3b>(j, n_baryWidth)[0] = 0;
//        cvColorImage.at<cv::Vec3b>(j, n_baryWidth)[1] = 0;
//        cvColorImage.at<cv::Vec3b>(j, n_baryWidth)[2] = 255;
//    }

//    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_barycenter.jpeg", cvColorImage);
    //////this works for retangle figure

    ////this directly works for plor figure
    cvImage=cv::imread("/home/zhongliang/Project/Pro_OrientationAdj/test_backPolar.jpeg", 0);
    int n_baryWidth(0);
    cv::Mat cvColorImage;

    cvtColor(cvImage,cvColorImage,cv::COLOR_BGR2RGB);
    n_baryWidth = round((dFeature + 0.5) * nWidth);

    for(int j = 0; j< nHeight; j++)
    {
        cvColorImage.at<cv::Vec3b>(m_vecPointPosition.at(j+n_baryWidth *(nLongRadius-nShortRadius+1)).y, m_vecPointPosition.at(j + n_baryWidth *(nLongRadius-nShortRadius+1)).x)[0] = 0;
        cvColorImage.at<cv::Vec3b>(m_vecPointPosition.at(j+n_baryWidth *(nLongRadius-nShortRadius+1)).y, m_vecPointPosition.at(j + n_baryWidth *(nLongRadius-nShortRadius+1)).x)[1] = 0;
        cvColorImage.at<cv::Vec3b>(m_vecPointPosition.at(j+n_baryWidth *(nLongRadius-nShortRadius+1)).y, m_vecPointPosition.at(j + n_baryWidth *(nLongRadius-nShortRadius+1)).x)[2] = 255;

    }

    for(int j = 0; j< nHeight; j++)
    {
        cvColorImage.at<cv::Vec3b>(m_vecPointPosition.at(j+round(nWidth/2.0) *(nLongRadius-nShortRadius+1)).y, m_vecPointPosition.at(j + round(nWidth/2.0) *(nLongRadius-nShortRadius+1)).x)[0] = 0;
        cvColorImage.at<cv::Vec3b>(m_vecPointPosition.at(j+round(nWidth/2.0) *(nLongRadius-nShortRadius+1)).y, m_vecPointPosition.at(j + round(nWidth/2.0) *(nLongRadius-nShortRadius+1)).x)[1] = 255;
        cvColorImage.at<cv::Vec3b>(m_vecPointPosition.at(j+round(nWidth/2.0) *(nLongRadius-nShortRadius+1)).y, m_vecPointPosition.at(j + round(nWidth/2.0) *(nLongRadius-nShortRadius+1)).x)[2] = 0;

    }




    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_barycenter.jpeg", cvColorImage);

}


void ConfidentMapConvex::calFigureDerivation(cv::Mat &cvImage)
{
    int nHeight = cvImage.rows;
    int nWidth = cvImage.cols;
    cv::Mat cvImage32(nHeight, nWidth, CV_32FC1);
    cvImage.convertTo(cvImage32, CV_32FC1, 1.0/255.0);   //transfer to the cv_imgae32


    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y, dst;



    cv::Sobel(cvImage, grad_x, CV_16S, 1, 0, 3, 1, 1, cv::BORDER_DEFAULT);
    convertScaleAbs( grad_x, abs_grad_x );
    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_grad_x.jpeg", abs_grad_x);
    cv::Sobel(cvImage, grad_y, CV_16S, 0, 1, 3, 1, 1, cv::BORDER_DEFAULT);
    convertScaleAbs( grad_y, abs_grad_y );
    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_grad_y.jpeg", abs_grad_y);
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst);
    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_grad_dst.jpeg", dst);

    //threshold the gradient image so that low intensity gets mapped to 255 and 0 everything else
    //you will need to fine tune the threshold a bit
    //for this thing, the gradient image (cvImage) needs to be of type CV_8UC1

    cv::Mat cvImageBlack(nHeight, nWidth, CV_8UC1, cv::Scalar(0));
    cv::Mat thresholdedGradient;
    double threshold = 80;
    cv::threshold(dst, thresholdedGradient, threshold, 255, cv::THRESH_BINARY);

    //find contours in the thresholded gradient image
    std::vector<std::vector<cv::Point> > contours; //the output of the find contours method will be here. It is a vector of contours. The contours are encoded as vector<Point>
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(thresholdedGradient, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
    //cycle through the contours. Each contour with low contour area gets discarded
    for(int i = 0; i < contours.size(); i++)
    {
        cv::Scalar color(255);


        float sizeThreshold(10);
        float size = cv::contourArea(contours[i]);
        if(size > sizeThreshold)    {
            //delete stuff
            qDebug()<<"size:"<<size;
            cv::drawContours(cvImageBlack, contours, i, color, cv::FILLED);
        }
    }

    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_counter.jpeg", cvImageBlack);

//    cv::convertScaleAbs()


}

////////////////////////// control part ending


}
}














