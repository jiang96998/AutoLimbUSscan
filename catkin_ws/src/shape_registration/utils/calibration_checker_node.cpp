/*******************************************************************
* STD INCLUDES
*******************************************************************/
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

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

/*******************************************************************
* SENSOR_FUSION INCLUDES
*******************************************************************/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/**
 * @brief The Fusion_and_Publish class
 * This class finds the coordinates of all the corners in the chessboard in the camera coordinates and later on
 * the transformation between camera coordinates and the robot base is read from the /home/nehil/.ros/easy_handeye/iiwa_azure_kinect_eye_on_base.yaml
 * calibration file. Then the points will be transformed to the robot base, and this way we will know whether our handeye calibration works well or not.
 */

class Fusion_and_Publish
{
public:
    Fusion_and_Publish();
    ~Fusion_and_Publish() { sync.reset(); }
    void callback(const sensor_msgs::ImageConstPtr &depth_image, const sensor_msgs::ImageConstPtr &color_image,
                  const sensor_msgs::CameraInfoConstPtr &cam_info_msg);

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
    cv::Size board_size;
};

Fusion_and_Publish::Fusion_and_Publish()
{
    int board_x;
    int board_y;
    std::string calibration_file_path;
    // nh_.getParam("chessboard_calibration/calibration_file_path", calibration_file_path);
    nh_.getParam("chessboard_calibration/board_x", board_x);
    nh_.getParam("chessboard_calibration/board_y", board_y);
    board_size = cv::Size(board_x, board_y);
    // read the calibration data from yaml file

    // YAML::Node config = YAML::LoadFile(calibration_file_path);
    // YAML::Node attributes = config["transformation"];

    // transformStamped.transform.rotation.x = attributes["qx"].as<double>();
    // transformStamped.transform.rotation.y = attributes["qy"].as<double>();
    // transformStamped.transform.rotation.z = attributes["qz"].as<double>();
    // transformStamped.transform.rotation.w = attributes["qw"].as<double>();

    // transformStamped.transform.translation.x = attributes["x"].as<double>();
    // transformStamped.transform.translation.y = attributes["y"].as<double>();
    // transformStamped.transform.translation.z = attributes["z"].as<double>();

    depth_sub.subscribe(nh_, "/depth_to_rgb_image", 20);
    color_sub.subscribe(nh_, "/rgb_image", 20);
    cam_info_sub.subscribe(nh_, "/camera_info", 20);
    sync.reset(new Sync(sync_pol(5), depth_sub, color_sub, cam_info_sub));
    sync->registerCallback(boost::bind(&Fusion_and_Publish::callback, this, _1, _2, _3));
    cv::namedWindow("Extractcorner", 0);
    cv::resizeWindow("Extractcorner", 1440, 2560);
}

void Fusion_and_Publish::callback(const sensor_msgs::ImageConstPtr &depth_image, const sensor_msgs::ImageConstPtr &color_image,
                                  const sensor_msgs::CameraInfoConstPtr &cam_info_msg)
{

    float cx = static_cast<float>(cam_info_msg->K.at(2));
    float cy = static_cast<float>(cam_info_msg->K.at(5));
    float fx = static_cast<float>(cam_info_msg->K.at(0));
    float fy = static_cast<float>(cam_info_msg->K.at(4));
    cv_bridge::CvImageConstPtr cv_ptr_depth;
    cv_bridge::CvImageConstPtr cv_ptr_color;
    try
    {
        cv_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat data;
        data = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Detect corners of chessboard
    cv_ptr_color = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
    cv::Mat chessboard = cv_ptr_color->image;
    cv::Mat Extractcorner = chessboard.clone();
    std::vector<cv::Point2f> corners;
    cv::Mat imageGray;
    cv::cvtColor(Extractcorner, imageGray, CV_RGB2GRAY);

    bool patternfound = cv::findChessboardCorners(Extractcorner, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!patternfound)
    {
        std::cout << "can not find chessboard corners!" << std::endl;
        exit(1);
    }
    else
    {
        ROS_INFO_STREAM("FOUND");
        cv::cornerSubPix(imageGray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001));
    }

    // std::cout << transformStamped << std::endl;
    std::ofstream file;
    file.open("/home/y/RosCode/catkin_ws/src/dataset/corners_pos.txt");
    int gap1=0;
    int gap2=0;
    ROS_INFO(std::to_string(corners.size()).c_str());
    for (std::size_t i = 0; i < corners.size(); i++)
    {
        if(gap2%2==0)
        {
        cv::circle(chessboard, corners[i], 1, cv::Scalar(255, 0, 255), 1, 8);
        cv::putText(chessboard, std::to_string(i + 1), corners[i], cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(0, 0, 255), 1, 8);
        }
        gap2++;
        geometry_msgs::Point32 tempPoint;
        tempPoint.z = cv_ptr_depth->image.at<float>(static_cast<int>(corners[i].y), static_cast<int>(corners[i].x));
        tempPoint.x = (corners[i].x - cx) * tempPoint.z / fx;
        //below line can be changed.
        //tempPoint.y = (corners[i].y - cy) * tempPoint.z / fx;
        tempPoint.y = (corners[i].y - cy) * tempPoint.z / fy;

        geometry_msgs::PointStamped transformed_pt;
        geometry_msgs::PointStamped initial_pt;
        initial_pt.point.x = tempPoint.x;
        initial_pt.point.y = tempPoint.y;
        initial_pt.point.z = tempPoint.z;

        /*tf2::doTransform(initial_pt, transformed_pt, transformStamped);

         tempPoint.x = transformed_pt.point.x;
         tempPoint.y = transformed_pt.point.y;
         tempPoint.z = transformed_pt.point.z;*/

        //std::cout << i + 1 << ":  [" << 1000 * tempPoint.x << " " << 1000 * tempPoint.y << " " << 1000 * tempPoint.z << "]" << std::endl;

        if (this->count == 10)
        {
            // std::ofstream file;
            //    file.open("/home/y/RosCode/catkin_ws/src/dataset/corners_pos.txt", std::ios_base::app); // append instead of overwrite
            // file.open("/home/y/RosCode/catkin_ws/src/dataset/corners_pos.txt");
            if(gap1%2==0)
            file << std::to_string(tempPoint.x/1000) << " " << std::to_string(tempPoint.y/1000) << " " << std::to_string(tempPoint.z/1000) << "\n";
            gap1++;
        }
    }

    this->count += 1;

    if (count > 10)
    {
        cv::imshow("Extractcorner", chessboard);
        cv::imwrite("/home/zhongliang/yuangao/roscode/catkin_ws/src/dataset/chess.jpg", chessboard);
        cv::waitKey(10);
        ros::shutdown();
    }
}

int main(int argc, char **argv)
{
    //ROS initialization
    ros::init(argc, argv, "calibration_checker");
    ROS_INFO("[AZURE KINECT] calibration checker started!");
    Fusion_and_Publish FP;
    ros::spin();
    return 0;
}
