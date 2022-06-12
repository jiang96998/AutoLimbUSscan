/*******************************************************************
* STD INCLUDES
*******************************************************************/
#include <iostream>
#include <fstream>

/*******************************************************************
* ROS INCLUDES
*******************************************************************/
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>

/*******************************************************************
* OPENCV INCLUDES
*******************************************************************/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>

/*******************************************************************
* SENSOR_FUSION INCLUDES
*******************************************************************/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*******************************************************************
* QT UI INCLUDES
*******************************************************************/
//#include <QMainWindow>
//#include "ui_mainwindowarucotesting.h"

//namespace Ui {
//class MainWindowArucoTesting;
//}

//class MainWindowArucoTesting : public QMainWindow
//{
//  Q_OBJECT

//public:
//  explicit MainWindowArucoTesting(QWidget *parent = nullptr);
//  ~MainWindowArucoTesting();

//private slots:
//  void on_pushButton_clicked();

//private:
//  Ui::MainWindowArucoTesting *ui;
//};

//MainWindowArucoTesting::MainWindowArucoTesting(QWidget *parent) :
//  QMainWindow(parent),
//  ui(new Ui::MainWindowArucoTesting)
//{
//  ui->setupUi(this);
//}

//MainWindowArucoTesting::~MainWindowArucoTesting()
//{
//  delete ui;
//}

//void MainWindowArucoTesting::on_pushButton_clicked()
//{

//}

/**
 * @brief The ArucoChecker class
 * This class finds the coordinates of all the corners in the chessboard in the camera coordinates and later on
 * the transformation between camera coordinates and the robot base is read from the /home/nehil/.ros/easy_handeye/iiwa_azure_kinect_eye_on_base.yaml
 * calibration file. Then the points will be transformed to the robot base, and this way we will know whether our handeye calibration works well or not.
 */

class ArucoChecker
{
public:
    ArucoChecker();
    void callback(const sensor_msgs::ImageConstPtr &depth_image, const sensor_msgs::ImageConstPtr &color_image,
                  const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

private:
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped transformStamped;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;
    typedef message_filters::Synchronizer<sync_pol> Sync;
    boost::shared_ptr<Sync> sync;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Subscriber<sensor_msgs::Image> color_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;
    int count = 0;
};

ArucoChecker::ArucoChecker()
{
    ROS_INFO("111111111111111");
    depth_sub.subscribe(nh_, "/depth_to_rgb_image", 1);
    color_sub.subscribe(nh_, "/rgb_image", 1);
    cam_info_sub.subscribe(nh_, "/camera_info", 1);
    sync.reset(new Sync(sync_pol(5),depth_sub, color_sub, cam_info_sub));

    ROS_INFO("22222222222222");
    sync->registerCallback(boost::bind(&ArucoChecker::callback,this,_1,_2,_3));

    ROS_INFO("33333333333333");
    cv::namedWindow("Extractcorner",0);
    cv::resizeWindow("Extractcorner", 1440, 2560);
}

void ArucoChecker::callback(const sensor_msgs::ImageConstPtr &depth_image, const sensor_msgs::ImageConstPtr &color_image,
                                  const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{

    ROS_INFO("444444444444");
    float cx = static_cast<float>(cam_info_msg->K.at(2));
    float cy = static_cast<float>(cam_info_msg->K.at(5));
    float fx = static_cast<float>(cam_info_msg->K.at(0));
    float fy = static_cast<float>(cam_info_msg->K.at(4));
    cv_bridge::CvImageConstPtr cv_ptr_depth;
    cv_bridge::CvImageConstPtr cv_ptr_color;
    cv::Mat imageGray;

    ROS_INFO("444444444444");
    try
    {
        cv_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr_color = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
        cv::Mat color_image_cv = cv_ptr_color->image;
        cv::Mat depth_image_cv = cv_ptr_depth->image;
        cv::cvtColor(color_image_cv, imageGray, CV_RGB2GRAY);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_INFO("444444444444");
    cv::Mat marker_img = cv_ptr_color->image;
    if(this->count == 10) {
      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f> > corners;
      cv::aruco::detectMarkers(imageGray, dictionary, corners, ids);
      ROS_INFO("444444444444");
      // if at least one marker detected
      if (ids.size() > 0) {
          cv::aruco::drawDetectedMarkers(marker_img, corners, ids);
          for (unsigned int i = 0; i < corners.size(); i++) {
            if(ids[i] == 584){
              for (unsigned int j = 0; j < corners[i].size(); j++) {
                geometry_msgs::Point32 tempPoint;
                tempPoint.z = cv_ptr_depth->image.at<float>(static_cast<int>(corners[i][j].y), static_cast<int>(corners[i][j].x));
                tempPoint.x = (corners[i][j].x - cx) * tempPoint.z / fx;
                tempPoint.y = (corners[i][j].y - cy) * tempPoint.z / fy;
                std::ofstream file;

                file.open("/home/y/RosCode/catkin_ws/src/dataset/corners_pos.txt", std::ios_base::app); // append instead of overwrite
                file << std::to_string(1000 * tempPoint.x) << " " << std::to_string(1000 * tempPoint.y) << " " << std::to_string(1000 * tempPoint.z) << "\n";
                ROS_INFO("8888888888888888");
              }
            }
          }
      }
    }

    ROS_INFO("55555555555");
    // cv::imshow("Extractcorner", marker_img);
    // cv::waitKey(1);
    this->count += 1;
}



int main(int argc, char** argv)
{
    //ROS initialization
    ros::init(argc, argv, "calibration_checker");
    ROS_INFO("[AZURE KINECT] calibration checker started!");
    ArucoChecker FP;
    ros::spin();
    return 0;
}
