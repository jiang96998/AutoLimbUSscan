#ifndef CONFIDENTMAPCONVEX_H
#define CONFIDENTMAPCONVEX_H

#include <QtCore/QObject>
#include <QString>
#include <QDebug>

//imfusion part
#include <ImFusion/Base/Algorithm.h>  // for IoAlgorithm
#include <ImFusion/Base/DataList.h>   // for DataList
#include <ImFusion/Stream/OpenIGTLinkImageStream.h>
#include <ImFusion/Base/AlgorithmListener.h>  // for AlgorithmListener
#include <ImFusion/Base/Log.h>                // for LOG_INFO
#include <ImFusion/Base/Properties.h>         // for Properties

#include <ImFusion/Base/MemImage.h>
#include <ImFusion/Base/SharedImage.h>
#include <ImFusion/Base/SharedImageSet.h>



#include <ImFusion/Base/ImFusionFile.h>
#include <ImFusion/GUI/AnnotationModel.h>
#include <ImFusion/GUI/DataWidget.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>
#include <ImFusion/GUI/ImageView2D.h>
#include <ImFusion/GUI/ImageView3D.h>
#include <ImFusion/GUI/Interactive.h>
#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/IO/BackgroundExporter.h>



#include <ImFusion/Base/TypedImage.h>
#include <ImFusion/Stream/OpenIGTLinkStreamData.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <ImFusion/US/USConfidenceMapAlgorithm.h>
#include <ImFusion/Base/Properties.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/US/USSweepRecorderAlgorithm.h>
#include <ImFusion/Base/Progress.h>
#include <ImFusion/Base/OpenCV.h>




#include <vector>
#include <math.h>
#include <stdio.h>

#include <Eigen/Dense>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

const double PI = 3.14159265358979323846;
//get from imfusion
const cv::Point pCenterPoint = cv::Point(410, 853);
const int nShortRadius = 300;  //324
const int nLongRadius = 823;

//get from geometric from US probe
const float fConvexAngle = 56.0/180.0 * PI;     //C5-2/60 GPS Convex  56
const float fAngleInterval = 0.05/180.0 * PI;


namespace ImFusion {
namespace vision_control {

class ConfidentMapConvex : public QObject
{
    Q_OBJECT

public:
    ConfidentMapConvex();
    ~ConfidentMapConvex();

    static bool createCompatible(const DataList& data, Algorithm** a = 0);
    void compute();
    void output(DataList& dataOut);

    void configure(const Properties* p);
    void configure(Properties* p) const;


private:
    SharedImageSet* m_image;
    SharedImageSet* m_output;

    double m_paramDouble;
    vec2 m_paramVec;





    //openCV Part beginning
public:
    void readImage();
    DataList readImage_IMF(const QString& szFilePath, const QString& szFileName);
    void saveImageToDisc(MemImage* memImage);

    //openCV Part ending
    std::vector<cv::Point2f> refineFan(cv::Point centerPoint, int nShortRadius, int nlongRadius);   //obtain the fan from US image

    cv::Mat polarToCatesian(cv::Point centerPoint, int nshortRadius, int nlongRadius);
    cv::Mat catesianToPolar(cv::Point centerPoint, int nshortRadius, int nlongRadius);
    DataList* computeConfiMap(SharedImageSet* sharedImageSet);



    void setCvMatRaw(cv::Mat cvMat) {m_CvImagRaw= cvMat;}
    cv::Mat getCvMatRaw() {return m_CvImagRaw;}
    void setHeightRaw(int nHeight) {m_nHeightRaw= nHeight;}
    void setWidthRaw(int nWidth) {m_nWidthRaw= nWidth;}
    int getHeightRaw() {return m_nHeightRaw;}
    int getWidthRaw() {return m_nWidthRaw;}

    //for control part
    void computeBaryCenter(cv::Mat& cvImage);   //give the angular adjustment
    void calFigureDerivation(cv::Mat& cvImage);   //get the derivation of 2D image (cv_8UC1b)



public:
    cv::Mat m_CvImagRaw;   //used to save raw image data
    bool m_bFlagImgRaw{false};   //check is there any raw data for comfidence map

    int m_nHeightRaw;
    int m_nWidthRaw;
    double m_dConfidenceAng {0};


    std::vector<cv::Point2f> m_vecPointPosition;


};

}
}



#endif // CONFIDENTMAPCONVEX_H
