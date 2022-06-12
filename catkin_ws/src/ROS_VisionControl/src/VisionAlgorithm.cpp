#include "vision_control/VisionAlgorithm.h"
#include <ImFusion/Base/Log.h>
#include <ImFusion/Stream/OpenIGTLinkConnection.h>
#include <ImFusion/Stream/TrackingStreamData.h>
#include <QDebug>

#include <fstream>
#include <QVector3D>
#include <ctime> 



namespace ImFusion
{
  namespace vision_control
  {

    PluginAlgorithm::PluginAlgorithm()
    {
      onInitROS(); //初始化ros


      //超声探头的位置
      probe_rotation_.block<4, 4>(0, 0) << 0, 0, 1, 0,
          1, 0, 0, 0,
          0, 1, 0, 38.6,
          0, 0, 0, 1;
          // 38.6

      m_scanPointPoses.resize(0);
      m_confiMapConvex = new ConfidentMapConvex();
      m_sharedImageSetCar = new SharedImageSet();
      m_dataListCar = new DataList();
      m_usListener = new StreamOptimizer();

      // these windows are to show the US image and its confidence map
      // cv::namedWindow("memImage");
      // cv::namedWindow("confi_map");
    }

    void PluginAlgorithm::receive_us_results(sensor_msgs::ImageConstPtr img_msg)
    {
      cv_bridge::CvImagePtr cv_mask_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
      m_segmented_images.push_back(cv_mask_ptr->image);
    }

    cv::Point PluginAlgorithm::find_closest_centroid(const std::vector<std::vector<cv::Point>> &contours, const cv::Point &gt_centroid)
    {
      auto distance = 0.0;
      cv::Point result;
      unsigned int idx = 0;
      for (auto contour : contours)
      {
        cv::Moments m = moments(contour, true);
        cv::Point centroid(m.m10 / m.m00, m.m01 / m.m00);
        auto curr_dist = std::sqrt(std::pow(gt_centroid.x - centroid.x, 2) + std::pow(gt_centroid.y - centroid.y, 2));
        if (idx == 0)
        {
          distance = curr_dist;
          result = centroid;
        }
        else
        {
          if (curr_dist < distance)
          {
            distance = curr_dist;
            result = centroid;
          }
        }
        idx++;
      }
      return result;
    }

    //initialize the ros related node.
    void PluginAlgorithm::onInitROS()
    {
      std::map<std::string, std::string> emptyArgs;
      if (!ros::isInitialized())
      {
        ros::init(emptyArgs, "iiwaRos");
      }
      ros_spinner_ = std::make_unique<ros::AsyncSpinner>(1);
      ros_spinner_->start();

      // create a ROS handle to connect the neural network and the movement tracking parts
      ros::NodeHandle nh;
      // subscribers
      // this->m_check_movement_sub = nh.subscribe("movement_occured", 1, &PluginAlgorithm::check_movement, this);
      // this->m_unet_segmentation_sub = nh.subscribe("segmentedImg", 2, &PluginAlgorithm::receive_us_results, this);

      //subscribe vessel's position
       this->sub = nh.subscribe("/vessel/position", 10, &PluginAlgorithm::receiveVesselPos, this);

      // publishers
      // this->m_calc_transformation_pub = nh.advertise<std_msgs::Bool>("/button_pressed_from_app", 1);
      // this->m_unet_segmentation_pub = nh.advertise<sensor_msgs::Image>("/ultrasound_img", 1);
      this->m_ros_initialized = true;
      LOG_INFO(this->m_ros_initialized);
      // ros::spin();
      // return;
    }

    void PluginAlgorithm::setUnetParam(bool isPureUnet)
    {
      // std::map<std::string, std::string> emptyArgs;
      // if (!ros::isInitialized()) { ros::init(emptyArgs, "iiwaRos"); }
      // nh.setParam("p_bool",isPureUnet);
    }

    void PluginAlgorithm::unet_segmentation(cv::Mat us_image)
    {
      if (this->m_ros_initialized)
      {
        cv_bridge::CvImage tmp;
        tmp.image = us_image;
        tmp.encoding = sensor_msgs::image_encodings::MONO8;
        auto cv_ptr = std::make_shared<cv_bridge::CvImage>(tmp);
        //cv_ptr->image = us_image;
        while (this->m_unet_segmentation_pub.getNumSubscribers() < 1)
        {
          // wait for a connection to publisher
          // you can do whatever you like here or simply do nothing
        }
        m_unet_segmentation_pub.publish(cv_ptr->toImageMsg());
      }
      else
      {
        LOG_ERROR("PLEASE INITIALIZE ROS USING INITIALIZE ROS BUTTON!");
      }
    }

    void PluginAlgorithm::calculate_confidance_map()
    {
      MemImage *memImageConfiMap = m_dataListConfiMap->getImage()->mem();
      std::cout << "DATALIST SIZE: " << m_dataListConfiMap->container().size() << std::endl;
      int nImageHeight = memImageConfiMap->height();
      int nImageWidth = memImageConfiMap->width();
      cv::Mat cvMatConfiMap(nImageHeight, nImageWidth, CV_32FC1, cv::Scalar(255));
      ImFusion::OpenCV::convert(memImageConfiMap, cvMatConfiMap);

      std::cout << "m_sharedImageSetCar size: " << m_sharedImageSetCar->size() << std::endl;

      //transfer the calculated confidence map from 32f to 8u, sametime [0,1] to [0,255]
      m_cvMatConfiMapUChar = cv::Mat(nImageHeight, nImageWidth, CV_8UC1, cv::Scalar(255));
      cvMatConfiMap.convertTo(m_cvMatConfiMapUChar, CV_8UC1, 255);

      //  // save confimap
      //  cv::Mat new_img1(nImageHeight, nImageWidth, CV_8UC1, cv::Scalar(255));
      //  cv::resize(m_cvMatConfiMapUChar, new_img1, cv::Size(375, 550));
      //  cv::imwrite("/home/nehil/thesis_result_files/confimap/" + std::to_string(img_idx) + ".png", new_img1);

      //cv::flip(m_cvMatConfiMapUChar, m_cvMatConfiMapUChar, 1);
      cv::threshold(m_cvMatConfiMapUChar, m_cvMatConfiMapUChar, 253, 255, cv::THRESH_BINARY_INV);
      cv::imshow("confi_map", m_cvMatConfiMapUChar);

      // save confimap thresholded
      cv::Mat new_img(nImageHeight, nImageWidth, CV_8UC1, cv::Scalar(255));
      cv::resize(m_cvMatConfiMapUChar, new_img, cv::Size(375, 550));
      //  cv::imwrite("/home/nehil/thesis_result_files/confimap_thresholded/thresholded_" + std::to_string(img_idx) + ".png", new_img);
      this->img_idx += 1;
    }

    float PluginAlgorithm::calculate_rotation_angle()
    {

      int nHeight = m_cvMatConfiMapUChar.rows;
      int nWidth = m_cvMatConfiMapUChar.cols;

      //  std::cout << nWidth << "x" << nHeight << std::endl;

      // find moments of the image
      cv::Moments m = moments(m_cvMatConfiMapUChar, true);
      //  cv::Point p(m.m10/m.m00, m.m01/m.m00);
      cv::Point p(m.m10 / m.m00, int(nHeight / 2.0));

      cv::Point center(int(nWidth / 2.0f), int(nHeight / 2.0));

      //  // save the thresholded results with the image centroid
      //  cv::Mat new_img(nHeight, nWidth, CV_8UC1, cv::Scalar(255));
      //  cv::resize(m_cvMatConfiMapUChar, new_img, cv::Size(375, 550));
      //  cv::circle(new_img, cv::Point(p.x * 375 / nWidth, p.y * 550 / nHeight), 6, cv::Scalar(0,0,0), -1);
      //  cv::circle(new_img, cv::Point(center.x * 375 / nWidth, center.y * 550 / nHeight), 6, cv::Scalar(0,0,0), -1);
      //  cv::imwrite("/home/nehil/thesis_result_files/confimap_thresholded/" + std::to_string(img_idx) + ".png", new_img);

      p.x = int(p.x * m_pixel_width);
      p.y = int(p.y * m_pixel_height);

      center.x = int(center.x * m_pixel_width);
      center.y = int(center.y * m_pixel_height);

      cv::Point start_point(int((nWidth / 2.0f) * m_pixel_width), 0);

      cv::Vec2f start_vec(center.x - start_point.x, center.y - start_point.y);
      cv::Vec2f end_vec(p.x - start_point.x, p.y - start_point.y);

      float cos_theta = start_vec.dot(end_vec) / (cv::norm(start_vec) * cv::norm(end_vec));
      float theta = acos(cos_theta) * 180.0 / M_PI;

      auto direction = start_vec[0] * end_vec[1] + start_vec[1] * end_vec[0];

      if (center.x < p.x)
      {
        m_rotation_direction = true;
      }
      else
      {
        m_rotation_direction = false;
      }

      std::cout << "the angle is : " << theta << std::endl;
      std::cout << "the angle direction is : " << m_rotation_direction << std::endl;
      return theta;
    }

    float PluginAlgorithm::optimize_curr_probe_pose(MemImage *memImage)
    {
      int nImageHeight = memImage->height();
      int nImageWidth = memImage->width();
      cv::Mat cv_imgIn(nImageHeight, nImageWidth, CV_8UC1); //uchar
      memcpy(cv_imgIn.data, memImage->data(), memImage->byteSize());
      cv::imshow("memImage", cv_imgIn);

      // save us imaging
      cv::Mat new_img(nImageHeight, nImageWidth, CV_32FC1, cv::Scalar(255));
      cv::resize(cv_imgIn, new_img, cv::Size(375, 550));
      //  cv::imwrite("/home/nehil/thesis_result_files/confimap_img/us_image_" + std::to_string(this->img_idx) + ".png", new_img);

      m_confiMapConvex->setHeightRaw(nImageHeight);
      m_confiMapConvex->setWidthRaw(nImageWidth); //save the size of the raw image

      m_confiMapConvex->setCvMatRaw(cv_imgIn.clone());
      m_confiMapConvex->m_bFlagImgRaw = true;

      std::cout << "m_sharedImageSetCar size: " << m_sharedImageSetCar->size() << std::endl;
      ///display cartesian figure
      if (m_sharedImageSetCar->size() == 0)
      {
        m_sharedImageSetCar->add(static_cast<std::shared_ptr<Image>>(memImage));
      }
      else
      {
        std::shared_ptr<SharedImage> temp_img(new SharedImage(static_cast<std::shared_ptr<Image>>(memImage)));
        m_sharedImageSetCar->replace(0, temp_img);
      }

      Selection sel;
      sel.setSingle(m_sharedImageSetCar->size() - 1); //select only  one image
      m_sharedImageSetCar->setSelection(sel);
      m_sharedImageSetCar->setModality(Data::ULTRASOUND);

      m_dataListConfiMap = m_confiMapConvex->computeConfiMap(m_sharedImageSetCar);

      calculate_confidance_map();

      std::cout << m_sharedImageSetCar->size() << std::endl;
      auto theta = calculate_rotation_angle();
      return theta;
    }

    void PluginAlgorithm::start_streaming()
    {
      usStream->open();
      usStream->start();
      usStream->addListener(m_usListener);
    }

    void PluginAlgorithm::stop_streaming()
    {
      usStream->close();
    }

    void PluginAlgorithm::check_movement(const std_msgs::BoolPtr movement_occured)
    {
      if (movement_occured->data)
      {
        // then movement occured stop the scanning and wait until whether the restart scan button is pressed or not.
        if (isRobotConnected())
        {
          // set the curr pose of the robot
          LOG_INFO("Motion detected!");
          LOG_INFO("Robot needs to go back to:");
          m_movement_occured_pose = getCurrentRobotPose().pose;
          LOG_INFO(m_movement_occured_pose.position);
          stop_recording_us_stream();
          m_transformation_count += 1;
          LOG_INFO("Current destination point would be normally:");
          LOG_INFO(getCurrentScanPointIndex());

          // if getCurrentScanPointIndex returns 5, it means the movement occured while going from position 4 to 5
          // so first we need to go to position 4
          // then we need to go to 5
          int nVisionStepMotionIteration = getCurrentScanPointIndex();
          if (getCurrentScanPointIndex() - 2 < 0)
          {
            this->m_break_point = 0;
          }
          else
          {
            this->m_break_point = getCurrentScanPointIndex() - 2; //get at which point occurs movement
          }

          auto update_pos = m_break_point;
          if (update_pos < 0)
          {
            update_pos = 0;
          }

          // the point where we need to go back, is updated with the exact point where the movement occured
          // we didn't want to add an additional point to the trajectory so we updated the (current_scan_idx - 2)th point
          m_scanPointPoses[update_pos](6) = m_movement_occured_pose.orientation.w;
          m_scanPointPoses[update_pos](3) = m_movement_occured_pose.orientation.x;
          m_scanPointPoses[update_pos](4) = m_movement_occured_pose.orientation.y;
          m_scanPointPoses[update_pos](5) = m_movement_occured_pose.orientation.z;

          m_scanPointPoses[update_pos](0) = m_movement_occured_pose.position.x;
          m_scanPointPoses[update_pos](1) = m_movement_occured_pose.position.y;
          m_scanPointPoses[update_pos](2) = m_movement_occured_pose.position.z;

          LOG_INFO("Movement occured while going from point " + std::to_string(this->m_break_point - 1) + " to point " + std::to_string(this->m_break_point));
          LOG_INFO("Next destination point after going back to the break point needs to be " + std::to_string(this->m_break_point));
          if (nVisionStepMotionIteration > 0)
          {
            stopStepMotion();
            // wait for the transformation occured message, and set this to the
          }
          else
          {
            executeCartesianCommand(getCurrentRobotPose().pose, false);
          }
        }
      }
    }

    void PluginAlgorithm::write_txt_file(std::string file_name, unsigned int break_point, bool f_half)
    {
      std::ofstream fs("/home/zhongliang/ros/nehil/markerless_motion_capture_for_RUSS/src/sweeps/trajectory/" + file_name + ".txt");
      if (!fs)
      {
        std::cerr << "Cannot open the output file." << std::endl;
        return;
      }
      unsigned int start = 0;
      unsigned int end = this->m_scanPointPoses.size();
      if (f_half && break_point != 0)
      {
        end = break_point;
      }
      else if (!f_half && break_point != 0)
      {
        start = break_point;
      }

      for (unsigned int i = start; i < end; i++)
      {
        fs << std::to_string(m_scanPointPoses[i].x()) << " " << std::to_string(m_scanPointPoses[i].y()) << " " << std::to_string(m_scanPointPoses[i].z()) << "\n";
      }
      fs.close();
    }

    void PluginAlgorithm::write_transformation(Eigen::Matrix4f transformation)
    {
      std::ofstream fs("/home/zhongliang/ros/nehil/markerless_motion_capture_for_RUSS/src/sweeps/transformation/" + std::to_string(m_transformation_count) + ".txt");
      if (!fs)
      {
        std::cerr << "Cannot open the output file." << std::endl;
        return;
      }

      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          fs << std::to_string(transformation(i, j)) << "\n";
        }
      }
      fs.close();
    }

    void PluginAlgorithm::transform_trajectory()
    {
      std_msgs::Bool button_pressed;
      button_pressed.data = true;
      this->m_calc_transformation_pub.publish(button_pressed);
    }

    void PluginAlgorithm::connect(const std::string &probe_name)
    {
      //innitialize Ros and iiwaRos object
      std::map<std::string, std::string> emptyArgs;
      if (!ros::isInitialized())
      {
        ros::init(emptyArgs, "iiwaRos");
      }
      ros_spinner_ = std::make_unique<ros::AsyncSpinner>(1);
      ros_spinner_->start();

      ros::NodeHandle node_handle;

      pose_state_.init("iiwa", std::bind(&PluginAlgorithm::poseCallback, this, std::placeholders::_1));
      wrench_state_.init("iiwa", std::bind(&PluginAlgorithm::wrenchCallback, this, std::placeholders::_1));
      torque_state_.init("iiwa");

      pose_command_.init("iiwa");
      linear_pose_command_.init("iiwa");

      control_mode_.init("iiwa");

      OpenIGTLinkConnection dummy_connection("Service robot connection");
      tracking_stream_ = new OpenIGTLinkTrackingStream(dummy_connection, "Robot");
      tracking_stream_->open();
      is_robot_connected_ = true;
      loadCalibrationFromFile("IFLUSCalibration.config", probe_name);

      emit robotConnected();
    }

    void PluginAlgorithm::disconnect()
    {
      if (ros::ok())
      {
        if (ros_spinner_ != nullptr)
        {
          ros_spinner_->stop();
          ros_spinner_ = nullptr;
        }
      }

      //! If the stream was never passed to a DataModel, we have to dispose it.
      if (owning_stream_ && tracking_stream_ != nullptr)
      {
        delete tracking_stream_;
      }
      is_robot_connected_ = false;
      emit robotDisconnected();
    }

    PluginAlgorithm::~PluginAlgorithm() { disconnect(); }

    bool PluginAlgorithm::createCompatible(const DataList &data, Algorithm **a)
    {
      if (data.size() != 0)
      {
        return false;
      }
      if (a)
      {
        *a = new PluginAlgorithm();
      }
      return true;
    }

    void PluginAlgorithm::compute()
    {
    }

    void PluginAlgorithm::configure(const Properties *p)
    {
      if (p == nullptr)
      {
        return;
      }
      p->param("something", something_);
    }

    void PluginAlgorithm::configuration(Properties *p) const
    {
      if (p == nullptr)
      {
        return;
      }
      p->setParam("something", something_);
    }

    void PluginAlgorithm::doSomething() { emit somethingHappened(); }

    int PluginAlgorithm::getCurrentScanPointIndex()
    {
      return m_nIteration;
    }

    void PluginAlgorithm::setMoveStatus(int poseStatus)
    {
      if (poseStatus == ON_INITIAL_POSE)
      {
        m_MoveStatus = ON_INITIAL_POSE;
      }
      else if (poseStatus == ON_CURRENT_POSE)
      {
        m_MoveStatus = ON_CURRENT_POSE;
      }
      else if (poseStatus == ON_FINAL_POSE)
      {
        m_MoveStatus = ON_FINAL_POSE;
      }
      else
      {
        m_MoveStatus = ON_BACK_INITIAL_POSE;
      }
    }

    std::vector<Eigen::VectorXd> PluginAlgorithm::updateScanPointPoses(const std::vector<Eigen::VectorXd> &prePoses, const Eigen::Matrix4f &tranformation)
    {
      size_t scanPointsNumber = prePoses.size();
      std::vector<Eigen::VectorXd> updatedPoses;
      for (size_t i = 0; i < scanPointsNumber; i++)
      {
        Eigen::Quaterniond quaPose(prePoses[i](6), prePoses[i](3), prePoses[i](4), prePoses[i](5));
        Eigen::Vector3d vecPose(prePoses[i](0), prePoses[i](1), prePoses[i](2));
        Eigen::Matrix4d originalMatrix{Eigen::Matrix4d::Identity()};
        originalMatrix.block<3, 3>(0, 0) = quaPose.toRotationMatrix();
        originalMatrix.block<3, 1>(0, 3) = vecPose;
        Eigen::Matrix4f originalMatrixF{Eigen::Matrix4f::Identity()};
        for (int i = 0; i < 4; i++)
        {
          for (int j = 0; j < 4; j++)
          {
            originalMatrixF(i, j) = static_cast<float>(originalMatrix(i, j));
          }
        }
        Eigen::Matrix4f transformedMatrix = tranformation * originalMatrixF;

        Eigen::Quaternionf q{transformedMatrix.block<3, 3>(0, 0)};
        Eigen::Vector3f t{transformedMatrix.block<3, 1>(0, 3)}; // block matrix:from (0,3) select a (3,1) matrix

        Eigen::VectorXd updatedPose(7);
        updatedPose << (double)t(0), (double)t(1), (double)t(2), (double)q.x(), (double)q.y(), (double)q.z(), (double)q.w();
        updatedPoses.push_back(updatedPose);
      }
      return updatedPoses;
    }

    void PluginAlgorithm::poseCallback(const iiwa_msgs::CartesianPose &pose)
    {
      if (is_robot_connected_)
      {
        std::lock_guard<std::mutex> lock{pose_mutex_};
        current_tip_pose_ = pose.poseStamped; //this would be updated after connect automatically

        //! This object is disposed by the destructor of TrackingStreamData, or at least so it says its documentation.
        //! If you try to handle its lifetime -> CRASH, so leave it as it is.
        TrackingInstrument *tracking_instrument = new TrackingInstrument();
        tracking_instrument->active = true;
        tracking_instrument->name = "US";
        tracking_instrument->quality = 1;
        //        auto image_center_pose = poseToEigenMat4(pose.poseStamped.pose, 1000) * probe_rotation_ * ultrasound_calibration_;

        auto image_center_pose = poseToEigenMat4(pose.poseStamped.pose, 1000) * probe_rotation_;

        tracking_instrument->matrix = image_center_pose;

        current_image_center_pose_.pose = eigenMat4ToPose(image_center_pose);
        current_image_center_pose_.header = pose.poseStamped.header;

        std::vector<TrackingInstrument *> instrument_vector{tracking_instrument};
        TrackingStreamData datas(tracking_stream_, instrument_vector);

        std::chrono::system_clock::time_point arrivalTime = std::chrono::high_resolution_clock::now();
        datas.setTimestampArrival(arrivalTime);

        tracking_stream_->sendStreamData(datas);

        emit poseChanged();
      }
    }

    void PluginAlgorithm::wrenchCallback(const iiwa_msgs::CartesianWrench &wench)
    {
      if (is_robot_connected_)
      {
        std::lock_guard<std::mutex> lock{wrench_mutex_};
        current_tip_wrench_ = wench.wrench; //this would be updated after connect automatically

        emit wrenchChanged();
      }
    }

    void PluginAlgorithm::movementFinishCallback()
    {
      LOG_INFO("enter movementFinishCallback");
      switch (m_fanStatus)
      {
      case ON_CURRENT_POSE:
        LOG_INFO("go to initial pose");
        onGotoPose(m_initialPose);
        m_fanStatus = ON_INITIAL_POSE;
        break;

      case ON_INITIAL_POSE:
        LOG_INFO("go to final pose");
        onGotoPose(m_finalPose);
        m_fanStatus = ON_FINAL_POSE;
        break;

      case ON_FINAL_POSE:
        LOG_INFO("go to initial pose again");
        onGotoPose(m_initialPose);
        m_fanStatus = ON_BACK_INITIAL_POSE;
        break;

      case ON_BACK_INITIAL_POSE:
        LOG_INFO("Finished");
        m_fanStatus = -1;
        break;
      }
    }

    void PluginAlgorithm::stepMovementFinishCallback()
    {
      LOG_INFO("enter stepMovementFinishCallback");
      int n_poseNumber = m_vecPose.size();
      if (m_nIteration < n_poseNumber && m_nIteration >= 0)
      {
        LOG_INFO(m_nIteration);
        LOG_INFO(n_poseNumber);
        onStepGotoPose(m_vecPose.at(m_nIteration));
        m_nIteration++;
      }
      else
      {
        LOG_INFO("finished");
        m_nIteration = 0;
        m_vecPose.resize(0);
      }
    }

    void PluginAlgorithm::setCurrentScanPointsIteration(int Iteration)
    {
      m_nIteration = Iteration;
      LOG_WARN("Set current destination point as");
      LOG_INFO(m_nIteration);
    }

    float PluginAlgorithm::calculate_weight(size_t curr_pos, size_t pos, size_t step_size)
    {
      auto last_point_pos = (curr_pos + step_size) - (pos - curr_pos) + 1;

      Eigen::Vector3d curr_position(m_scanPointPoses[curr_pos](0), m_scanPointPoses[curr_pos](1), m_scanPointPoses[curr_pos](2));
      Eigen::Vector3d curr_dest(m_scanPointPoses[last_point_pos](0), m_scanPointPoses[last_point_pos](1), m_scanPointPoses[last_point_pos](2));
      auto over_all_distance = 0.0f;
      auto partial_distance = std::sqrt(std::pow(curr_dest.x() - curr_position.x(), 2) + std::pow(curr_dest.y() - curr_position.y(), 2) + std::pow(curr_dest.z() - curr_position.z(), 2));

      for (size_t i = curr_pos + 1; i <= (curr_pos + step_size); i++)
      {

        Eigen::Vector3d c_temp(m_scanPointPoses[i](0), m_scanPointPoses[i](1), m_scanPointPoses[i](2));

        over_all_distance += std::pow(curr_position.x() - c_temp.x(), 2) + std::pow(curr_position.y() - c_temp.y(), 2) + std::pow(curr_position.z() - c_temp.z(), 2);
      }

      std::cout << std::pow(partial_distance, 2) / over_all_distance << std::endl;
      return std::pow(partial_distance, 2) / over_all_distance;
    }

    void PluginAlgorithm::reset_trajectory_using_confimap_result(float rotation_angle)
    {
      auto rotation_direction = getRotationDirection();
      size_t step_size = 3;
      auto final_pos = m_nIteration + step_size;
      if (final_pos > m_scanPointsNumber)
      {
        final_pos = m_scanPointsNumber;
        step_size = m_scanPointsNumber - m_nIteration;
      }
      for (size_t pos = m_nIteration; pos < final_pos; pos++)
      {
        if (rotation_direction)
        {
          update_pose(pos, -1 * calculate_weight(m_nIteration - 1, pos, step_size) * rotation_angle, ROTATION_X);
        }
        else
        {
          update_pose(pos, calculate_weight(m_nIteration - 1, pos, step_size) * rotation_angle, ROTATION_X);
        }
      }
    }

    void PluginAlgorithm::set_new_stream(Data *us_stream, Data *robot_stream)
    {
      m_cepha_stream = us_stream;
      m_robot_stream = robot_stream;
    }

    void PluginAlgorithm::receiveVesselPos(std_msgs::Float32MultiArray pos)
    {
      vessel_pos = pos.data[0];
      // LOG_INFO(vessel_pos);
    }

    void PluginAlgorithm::computeFeedback(Eigen::Quaterniond &qPose, Eigen::Vector3d &translation, int iteration)
    {
      // sleep(10);
      Eigen::Matrix3d R(Eigen::Matrix3d::Identity());
      R = qPose.toRotationMatrix();
      Eigen::Vector3d _t(0,-(0.5 - vessel_pos) * 50 / 1000,  0);
      LOG_INFO(vessel_pos);
      // Eigen::Vector3d _t( 0,-(0.5 - vessel_pos) * 51.3 / 1000, 0);

      // translation =  _t + translation;
      translation = R * _t + translation;
      // Eigen::Quaterniond quaternion2(R);
      // qPose=quaternion2;
      int n_poseNumber = m_scanPointPoses.size();
      float k = -1 / (n_poseNumber - iteration + 1);
      float b = 1 - k * (iteration - 1);

      LOG_INFO("computing");
      Eigen::VectorXd cc(7);
      // cc<<translation(0),translation(1), translation(2), qPose.x(), qPose.y(), qPose.z(), qPose.w();
      cc<<translation(0),translation(1), translation(2), qPose.x(), qPose.y(), qPose.z(), qPose.w();
      if(vessel_pos<=0.01||vessel_pos>=0.95)
      {
        LOG_INFO("vessel position wrong");
        return;
      }
      if(abs(vessel_pos-0.5)>0.05)
      {
        
        LOG_INFO("insert point");
        // LOG_INFO(translation);
      int n_poseNumber = m_scanPointPoses.size();
      float k = -1 / (n_poseNumber - iteration + 1);
      float b = 1 - k * (iteration - 1);
      for (size_t i = iteration+1; i < m_scanPointPoses.size(); i++)
      {
        
      // LOG_INFO("nnnnnnnnnn");
      // LOG_INFO(i);
        Eigen::Quaterniond quaPose(m_scanPointPoses[i](6), m_scanPointPoses[i](3), m_scanPointPoses[i](4), m_scanPointPoses[i](5));
        Eigen::Vector3d vecPosition(m_scanPointPoses[i](0), m_scanPointPoses[i](1), m_scanPointPoses[i](2));
        Eigen::Matrix3d matrix{Eigen::Matrix3d::Identity()};
        matrix.block<3, 3>(0, 0) = quaPose.toRotationMatrix();
        // _t(0)=(k*i+b)*_t(0);

        vecPosition=matrix*_t+vecPosition;

        // _t(1)=(k*i+b)*_t(1);
        // vecPosition(1)=_t(1);

      // LOG_INFO("aaaaaaaaaaa");
      // LOG_INFO(vecPosition);
        for (size_t j = 0; j < 3; j++)
        {
          m_scanPointPoses[i](j)=vecPosition(j);
        }

      // LOG_INFO("bbbbbbbbbbbbb");
      }

      // LOG_INFO("ccccccccccccccccccc");
      m_scanPointPoses.insert(m_scanPointPoses.begin() + m_nIteration+1, cc);
      }
      // for (size_t i = iteration; i < m_scanPointPoses.size(); i++)
      // {
      //   Eigen::Quaterniond quaPose(m_scanPointPoses[i](6), m_scanPointPoses[i](3), m_scanPointPoses[i](4), m_scanPointPoses[i](5));
      //   Eigen::Vector3d vecPosition(m_scanPointPoses[i](0), m_scanPointPoses[i](1), m_scanPointPoses[i](2)+ INITHEIGHT/1000.0);
      //   Eigen::Matrix3d matrix{Eigen::Matrix3d::Identity()};
      //   matrix.block<3, 3>(0, 0) = quaPose.toRotationMatrix();
      //   _t(0)=(k*i+b)*_t(0);
      //   vecPosition=matrix*_t+vecPosition;
      //   for (size_t j = 0; j < 3; j++)
      //   {
      //     m_scanPointPoses[i](i)=vecPosition(i);
      //   }

      // }
    }

    void PluginAlgorithm::updateTrajectory()
    {
    }
    void   PluginAlgorithm::Delay(int   time)//time*1000为秒数 
    { 
      clock_t   now   =   clock(); 
      LOG_INFO(now);
      while(   clock()   -   now   <   time   ); 
      LOG_INFO(clock());
    } 
    // void PluginAlgorithm::doMsg(const std_msgs::Time::ConstPtr& msg_p){
    //     ROS_INFO("我听见:%s",msg_p->data.c_str());
    //     // ROS_INFO("我听见:%s",(*msg_p).data.c_str());
    // }

    void PluginAlgorithm::VisionMovementFinishCallback()
    {
        LOG_INFO("enter VisionMovementFinishCallback");
      int n_poseNumber = m_scanPointPoses.size();
      if (m_MoveStatus != ON_INITIAL_POSE)
      {
        LOG_ERROR("Please go to initial position first!");
        return;
      }

      LOG_INFO("Current destination point : " + std::to_string(m_nIteration));
      if (m_nIteration < n_poseNumber && m_nIteration >= 0 && !stopRobot)
      {
        n_poseNumber = m_scanPointPoses.size();
        Eigen::Quaterniond quaPose(m_scanPointPoses[m_nIteration](6), m_scanPointPoses[m_nIteration](3), m_scanPointPoses[m_nIteration](4), m_scanPointPoses[m_nIteration](5));
        Eigen::Vector3d vecPosition(m_scanPointPoses[m_nIteration](0), m_scanPointPoses[m_nIteration](1), m_scanPointPoses[m_nIteration](2) + INITHEIGHT / 1000.0);

        Eigen::Matrix3d matrix{Eigen::Matrix3d::Identity()};
        matrix.block<3, 3>(0, 0) = quaPose.toRotationMatrix();

        // LOG_INFO("will move to ");
          LOG_INFO("id:",m_nIteration);
        // LOG_INFO("position:",vecPosition);

        if (!feedback)
        {
          // onVisionStepGotoPose(quaPose, vecPosition, true);
          executeCartesianCommand(quaPose, vecPosition, true);
          // LOG_INFO("sleep begin");
            // sleep(15);
          // LOG_INFO("sleep done");
          m_nIteration++;
        }
          else
          {

            LOG_INFO("start move");
            computeFeedback(quaPose, vecPosition, m_nIteration);
            if(m_nIteration!=0)
            {
            executeCartesianCommand(quaPose, vecPosition, true);
            }
            // LOG_INFO("sleep begin");
            // sleep(15);
            // LOG_INFO("sleep done");
            m_nIteration++;
          }
          LOG_INFO("moving");
      }
      else
      {
        stopRobot=true;
      LOG_INFO("Robot Stopped");
      }
  }


    // void PluginAlgorithm::VisionMovementFinishCallback()
    // {

    //   LOG_INFO("enter VisionMovementFinishCallback");
    //   int n_poseNumber = m_scanPointPoses.size();
    //   if (m_MoveStatus != ON_INITIAL_POSE)
    //   {
    //     LOG_ERROR("Please go to initial position first!");
    //     return;
    //   }

    //   LOG_INFO("Current destination point : " + std::to_string(m_nIteration));
    //   if (m_nIteration < n_poseNumber && m_nIteration >= 0 && !stopRobot)
    //   {
    //     Eigen::Quaterniond quaPose(m_scanPointPoses[m_nIteration](6), m_scanPointPoses[m_nIteration](3), m_scanPointPoses[m_nIteration](4), m_scanPointPoses[m_nIteration](5));
    //     Eigen::Vector3d vecPosition(m_scanPointPoses[m_nIteration](0), m_scanPointPoses[m_nIteration](1), m_scanPointPoses[m_nIteration](2) + INITHEIGHT / 1000.0);

    //     Eigen::Matrix3d matrix{Eigen::Matrix3d::Identity()};
    //     matrix.block<3, 3>(0, 0) = quaPose.toRotationMatrix();

    //     LOG_INFO("will move to ");
    //     LOG_INFO(vecPosition);

    //     if (!feedback)
    //     {
    //       onVisionStepGotoPose(quaPose, vecPosition, true);
    //       LOG_INFO("sleep begin");
    //         // sleep(5);
    //       LOG_INFO("sleep done");
    //       m_nIteration++;
    //     }
    //       else
    //       {

    //         LOG_INFO("start move");
    //         onVisionStepGotoPose(quaPose, vecPosition, true);
    //         LOG_INFO("move end");
    //         // Delay(20000);
    //         computeFeedback(quaPose, vecPosition, m_nIteration);
    //         m_nIteration++;
    //       }
    //       LOG_INFO(m_nIteration);
    //       LOG_INFO("move done");
    //       // the first movement is from initial position to the 0th position of the sweep
    //       // if the current position of the probe is -1 then the m_nIteration will be 0
    //       // at this point since the probe won't be touching the arm, we don't want to calculate the confimap
    //       // m_nIteration shows the destination position of the current position
    //       // also if we are resetting the position of the probe due to previous movement we don't need to calculate
    //       // the confimap
    //       /*
    //   Eigen::Matrix4d updated_curr_pose{Eigen::Matrix4d::Identity()};
    //   if(m_nIteration != 0 && !m_resetting_position) {
    //     if(m_motion_back_to_normal) {
    //       start_recording_us_stream();
    //       LOG_INFO("Restarted the recording");
    //       m_motion_back_to_normal = false;
    //     }
    //     // calculate the confimap
    //     // calculate the angle in a while loop
    //     // as long as the angle is larger than some threshold, recalculate confimap and rotate the whole trajectory
    //     auto theta = optimize_curr_probe_pose(m_usListener->getMemImage());
    //     auto direction = getRotationDirection();

    //     if(theta > 4 && std::abs(prev_theta - theta) > 1e-2f) {
    //       // move the whole trajectory using the curr theta and direction
    //       if(direction) {
    //         updated_curr_pose  = calculate_pose(m_nIteration - 1, -1 * theta, ROTATION_X);
    //       }
    //       else {
    //         updated_curr_pose  = calculate_pose(m_nIteration - 1, theta, ROTATION_X);
    //       }
    //       Eigen::Quaterniond temp_curr_rot(updated_curr_pose.block<3, 3>(0, 0));
    //       Eigen::Vector3d temp_curr_vec(updated_curr_pose.block<3, 1>(0, 3));
    //       prev_theta = theta;
    //       reset_trajectory_using_confimap_result(theta);
    //       LOG_INFO("will move to ");
    //       LOG_INFO(temp_curr_vec);
    //       onVisionStepGotoPose(temp_curr_rot, temp_curr_vec, true);
    //     }
    //     else{
    //       reset_trajectory_using_confimap_result(theta);
    //       LOG_INFO("will move to ");
    //       LOG_INFO(vecPosition);
    //       onVisionStepGotoPose(quaPose, vecPosition, true);
    //       m_nIteration++;
    //     }
    //   }
    //   else {
    //     LOG_INFO("will move to ");
    //     LOG_INFO(vecPosition);
    //     onVisionStepGotoPose(quaPose, vecPosition, true);
    //     m_nIteration++;
    //     if(m_resetting_position) {
    //       m_resetting_position = false;
    //       m_motion_back_to_normal = true;
    //     }
    //   }
    // }
    // else {
    //   if(m_nIteration == n_poseNumber) {
    //     // once the sweep reached an end
    //     // then finish and save the current recording
    //     stop_recording_us_stream();
    //     LOG_INFO("stop the final us recording");
    //     m_transformation_count = 0;
    //     m_motion_back_to_normal = false;
    //   }
    //   // here stop the recording
    //   LOG_INFO("finished");
    //   LOG_INFO(m_nIteration);
    //   LOG_INFO("set the iteration n to 0");
    //   m_nIteration = 0;
    //   m_MoveStatus = ON_CURRENT_POSE;
    
    // */
    //     }
    //   }

      void PluginAlgorithm::start_recording_us_stream()
      {
        LOG_ERROR("Do Record!");
        ImageStream *usStream = static_cast<ImageStream *>(m_cepha_stream);
        LiveTrackingStream *robStream = static_cast<LiveTrackingStream *>(m_robot_stream);

        std::vector<Stream *> vecStream;
        vecStream.push_back(usStream);
        vecStream.push_back(robStream);
        m_multiUSSweepRecorder = new USSweepRecorderAlgorithm(vecStream);

        m_multiUSSweepRecorder->start();
      }

      void PluginAlgorithm::stop_recording_us_stream()
      {
        m_multiUSSweepRecorder->stop();
        LOG_INFO("US recording stooped");
        DataList datalist; //output object
        m_multiUSSweepRecorder->output(datalist);

        auto sis = static_cast<SharedImageSet *>(datalist.getItem(0)); //sis:=SharedImageSet
        auto str = getDayAndTime();
        BackgroundExporter *sweepExporter = new BackgroundExporter();
        sweepExporter->save(sis, "/home/zhongliang/ros/nehil/markerless_motion_capture_for_RUSS/src/sweeps/complete_sweep_" + std::to_string(m_transformation_count) + ".imf");
      }

      std::string PluginAlgorithm::getDayAndTime()
      {
        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, sizeof(buffer), "%d_%m_%H_%M_%S", timeinfo);
        std::string str(buffer);

        return str;
      }

      //stop at current point and stop go into callback function
      void PluginAlgorithm::stopStepFanMotion()
      {
        int n_poseNumber = m_vecPose.size();
        if (m_nIteration < n_poseNumber && m_nIteration > 0)
        {
          m_nIteration = -1; //to avoid move around become 1 after add 1
        }
      }

      void PluginAlgorithm::stopStepMotion()
      {
        int n_poseNumber = m_scanPointPoses.size();
        if (m_nIteration < n_poseNumber && m_nIteration > 0)
        {
          m_nIteration = -1; //to avoid move around become 1 after add 1
        }
      }

      void PluginAlgorithm::executeCartesianCommand(const geometry_msgs::Pose &pose, bool linear,
                                                    const std::function<void()> &callback)
      {
        LOG_INFO("enter executeCartesianCommand");
        if (is_robot_connected_)
        {
          geometry_msgs::PoseStamped ps;
          ps.header.frame_id = "iiwa_link_0";
          ps.pose = pose;

          if (linear == true)
          {
            if (callback == nullptr)
            {
              linear_pose_command_.setPose(ps);
            }
            else
            {
              linear_pose_command_.setPose(ps, callback);
            }
          }
          else
          {
            if (callback == nullptr)
            {
              pose_command_.setPose(ps);
            }
            else
            {
              pose_command_.setPose(ps, callback);
            }
          }
        }
        else
        {
          LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
        }
      }

      void PluginAlgorithm::executeCartesianCommand(const Eigen::Matrix4d &matrix, bool linear,
                                                    const std::function<void()> &callback)
      {
        executeCartesianCommand(eigenMat4ToPose(matrix), linear, callback);
      }

      void PluginAlgorithm::executeCartesianCommand(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, bool linear,
                                                    std::function<void()> callback)
      {
        Eigen::Matrix4d matrix{Eigen::Matrix4d::Identity()};
        matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
        matrix.block<3, 1>(0, 3) = t;
        executeCartesianCommand(eigenMat4ToPose(matrix), linear, callback);
      }

      // THIS IS IN METERS
      geometry_msgs::PoseStamped PluginAlgorithm::getCurrentRobotPose()
      {
        assert(is_robot_connected_ == true &&
               "The robot has to be connected before receiving any state. Call the 'connect()' method.");
        std::lock_guard<std::mutex> lock{pose_mutex_};
        return current_tip_pose_;
      }

      // THIS IS IN METERS
      Eigen::Matrix4d PluginAlgorithm::getCurrentRobotTransformMatrix(bool in_millimeters)
      {
        assert(is_robot_connected_ == true &&
               "The robot has to be connected before receiving any state. Call the 'connect()' method.");
        std::lock_guard<std::mutex> lock{pose_mutex_};
        double scaling_factor{1};
        if (in_millimeters)
        {
          scaling_factor = 1000;
        }
        return poseToEigenMat4(current_tip_pose_.pose, scaling_factor);
      }

      geometry_msgs::PoseStamped PluginAlgorithm::getCurrentImageCenterPose()
      {
        assert(is_robot_connected_ == true &&
               "The robot has to be connected before receiving any state. Call the 'connect()' method.");
        std::lock_guard<std::mutex> lock{pose_mutex_};
        return current_image_center_pose_;
      }

      //wrench
      geometry_msgs::Wrench PluginAlgorithm::getCurrentRobotWrench()
      {
        assert(is_robot_connected_ == true &&
               "The robot has to be connected before receiving any state. Call the 'connect()' method.");
        std::lock_guard<std::mutex> lock{wrench_mutex_};
        return current_tip_wrench_;
      }

      geometry_msgs::Pose PluginAlgorithm::eigenMat4ToPose(Eigen::Matrix4d matrix, double scaling_factor)
      {
        Eigen::Quaterniond q{matrix.block<3, 3>(0, 0)};
        Eigen::Vector3d t{matrix.block<3, 1>(0, 3)}; // block matrix:from (0,3) select a (3,1) matrix

        geometry_msgs::Pose pose;
        pose.position.x = t[0] * scaling_factor;
        pose.position.y = t[1] * scaling_factor;
        pose.position.z = t[2] * scaling_factor;

        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();

        return pose;
      }

      Eigen::Matrix4d PluginAlgorithm::poseToEigenMat4(const geometry_msgs::Pose &pose, double scaling_factor)
      {
        Eigen::Matrix4d matrix{Eigen::Matrix4d::Identity()};
        matrix.block<3, 3>(0, 0) =
            Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                .toRotationMatrix();
        matrix.block<4, 1>(0, 3) = Eigen::Vector4d(pose.position.x * scaling_factor, pose.position.y * scaling_factor,
                                                   pose.position.z * scaling_factor, 1);
        return matrix;
      }

      void PluginAlgorithm::loadCalibrationFromFile(const std::string &file_name, const std::string &desired_probe)
      {
        std::string config_file_path = configuration_dir + file_name;
        std::ifstream config_file(config_file_path);
        if (config_file.is_open())
        {
          std::string probe_name;
          while (std::getline(config_file, probe_name))
          {
            for (int x = 0; x < 4; x++)
            {
              std::string line;
              std::getline(config_file, line);
              std::istringstream iss(line);
              for (int y = 0; y < 4; y++)
              {
                std::string s;
                std::getline(iss, s, ';');
                ultrasound_calibration_(x, y) = std::stod(s);
              }
            }
            std::string line;
            std::getline(config_file, line);
            temporal_calibration_ = std::stof(line);
            if (probe_name.compare(desired_probe) == 0)
            {
              config_file.close();
              //        ultrasound_calibration_ = Eigen::MatrixXd::Identity(4, 4);  //jzl
              ultrasound_calibration_ = Eigen::MatrixXd::Identity(4, 4);
              ultrasound_calibration_.block<3, 1>(0, 3) << 0, 0, 27.5;
              LOG_INFO("" << std::endl
                          << "Calibration found at " << config_file_path << " for probe " << desired_probe << ": "
                          << std::endl
                          << ultrasound_calibration_ << std::endl
                          << "Temporal Calibration: " << temporal_calibration_ << std::endl
                          << std::endl);

              return;
            }
          }
        }
        else
        {
          LOG_ERROR("Unable to open file");
        }
        LOG_ERROR("Couldn't find a calibration file at " << config_file_path << " for " << desired_probe
                                                         << ", I will load identity. " << std::endl);
        ultrasound_calibration_ = Eigen::MatrixXd::Identity(4, 4);
        ultrasound_calibration_.block<3, 1>(0, 3) << 0, 27.5, 0;
        LOG_INFO(ultrasound_calibration_ << std::endl);
      }

      ////////////////////////////JZL OrienAdj task
      void PluginAlgorithm::fanShapMotion(double doffsetAngle, double dStepAngle, int nRoationAxis, int nFanMotionType)
      {
        ///  just need to adjust the angle of Ra!
        LOG_INFO("Enter into FanMotion");
        if (is_robot_connected_)
        {
          LOG_INFO(nFanMotionType);
          if (FAN_MOTION_STYPE_CONTINOUS == nFanMotionType)
          {
            LOG_INFO("Enter into FanMotionC");
            //            calculateEndPoints(doffsetAngle, nRoationAxis);   //rotate around base
            calculateEndPointsTCP(doffsetAngle, nRoationAxis); //rotate around TCP
            movementFinishCallback();
          }
          else if (FAN_MOTION_STYPE_STEP == nFanMotionType)
          {
            LOG_INFO("Enter into FanMotionStep");
            //            calculateEndPointsStep(doffsetAngle, dStepAngle, nRoationAxis);   //rotate around base
            calculateEndPointsStepTCP(doffsetAngle, dStepAngle, nRoationAxis); //rotate around TCP
            m_nIteration = 0;
            stepMovementFinishCallback();
          }
        }
        else
        {
          LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
        }
      }

      //rotate around base frame
      void PluginAlgorithm::calculateEndPoints(double fOffsetAngle, int nRotationAxis)
      {
        if (is_robot_connected_)
        {
          auto robot_pose = getCurrentRobotTransformMatrix(true);
          Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
          Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //

          Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
          if (ROTATION_X == nRotationAxis)
          {
            offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI; //unit rad
            LOG_INFO(fOffsetAngle);
          }
          else if (ROTATION_Y == nRotationAxis)
          {
            offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI; //unit rad
          }
          else
          {
            LOG_INFO("the rotation is wrong");
          }
          Eigen::Vector3d initEulerAngle = eulerAngles - offsetAngleRad;
          Eigen::Vector3d finalEulerAngle = eulerAngles + offsetAngleRad;

          LOG_INFO(initEulerAngle);
          LOG_INFO(finalEulerAngle);

          //back to pose(4*4 matrix)
          Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(initEulerAngle[0], Eigen::Vector3d::UnitX()) *
                                              Eigen::AngleAxisd(initEulerAngle[1], Eigen::Vector3d::UnitY()) *
                                              Eigen::AngleAxisd(initEulerAngle[2], Eigen::Vector3d::UnitZ());
          m_initialPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
          m_initialPose.block<3, 1>(0, 3) = translation / 1000.0;

          tempQuaternion = Eigen::AngleAxisd(finalEulerAngle[0], Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(finalEulerAngle[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(finalEulerAngle[2], Eigen::Vector3d::UnitZ());
          m_finalPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
          m_finalPose.block<3, 1>(0, 3) = translation / 1000.0;
        }
        else
        {
          LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
        }
      }

      void PluginAlgorithm::update_pose(size_t pos, double fOffsetAngle, int nRotationAxis)
      {
        auto new_pose = calculate_pose(pos, fOffsetAngle, nRotationAxis);
        Eigen::Quaterniond new_quaternion(new_pose.block<3, 3>(0, 0));
        m_scanPointPoses[pos](6) = new_quaternion.w();
        m_scanPointPoses[pos](3) = new_quaternion.x();
        m_scanPointPoses[pos](4) = new_quaternion.y();
        m_scanPointPoses[pos](5) = new_quaternion.z();
      }

      Eigen::Matrix4d PluginAlgorithm::calculate_pose(size_t pos, double fOffsetAngle, int nRotationAxis)
      {
        Eigen::Quaterniond quaPose(m_scanPointPoses[pos](6), m_scanPointPoses[pos](3), m_scanPointPoses[pos](4), m_scanPointPoses[pos](5));
        Eigen::Vector3d vecPosition(m_scanPointPoses[pos](0), m_scanPointPoses[pos](1), m_scanPointPoses[pos](2));

        double x = m_scanPointPoses[pos](0);
        double y = m_scanPointPoses[pos](1);
        double z = m_scanPointPoses[pos](2);

        double qw = m_scanPointPoses[pos](6);
        double qx = m_scanPointPoses[pos](3);
        double qy = m_scanPointPoses[pos](4);
        double qz = m_scanPointPoses[pos](5);

        Eigen::Matrix4d curr_pos;
        curr_pos << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy), x,
            2 * (qx * qy + qw * qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy * qz - qw * qx), y,
            2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
            0, 0, 0, 1;
        Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
        if (ROTATION_X == nRotationAxis)
        {
          offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI; //unit rad
                                                           //LOG_INFO(fOffsetAngle);
        }
        else if (ROTATION_Y == nRotationAxis)
        {
          offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI; //unit rad
        }
        else
        {
          LOG_INFO("the rotation is wrong");
        }

        Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                                            Eigen::AngleAxisd(offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(offsetAngleRad[2], Eigen::Vector3d::UnitZ());

        Eigen::Matrix4d rotation{Eigen::Matrix4d::Identity()};
        rotation.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
        rotation.block<3, 1>(0, 3) << 0, 0, 0;

        curr_pos = curr_pos * rotation;
        curr_pos.block<3, 1>(0, 3) = vecPosition;

        return curr_pos;
      }

      //calculate the final pose when the robot rotate about TCP
      void PluginAlgorithm::calculateEndPointsTCP(double fOffsetAngle, int nRotationAxis)
      {
        if (is_robot_connected_)
        {
          auto robot_pose = getCurrentRobotTransformMatrix(true);
          Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
          Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //

          Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
          if (ROTATION_X == nRotationAxis)
          {
            offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI; //unit rad
            LOG_INFO(fOffsetAngle);
          }
          else if (ROTATION_Y == nRotationAxis)
          {
            offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI; //unit rad
          }
          else
          {
            LOG_INFO("the rotation is wrong");
          }

          //obtain the transformation between the target about TCP frame
          Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(-offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                                              Eigen::AngleAxisd(-offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                                              Eigen::AngleAxisd(-offsetAngleRad[2], Eigen::Vector3d::UnitZ());
          m_initialPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
          m_initialPose.block<3, 1>(0, 3) << 0, 0, 0;

          tempQuaternion = Eigen::AngleAxisd(offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(offsetAngleRad[2], Eigen::Vector3d::UnitZ());
          m_finalPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
          m_finalPose.block<3, 1>(0, 3) << 0, 0, 0;

          m_initialPose = robot_pose * m_initialPose;
          m_initialPose.block<3, 1>(0, 3) = translation / 1000.0;
          m_finalPose = robot_pose * m_finalPose;
          m_finalPose.block<3, 1>(0, 3) = translation / 1000.0;
        }
        else
        {
          LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
        }
      }

      // step by step rotate around base frame
      void PluginAlgorithm::calculateEndPointsStep(double fOffsetAngle, double dStep, int nRotationAxis)
      {
        fOffsetAngle = fabs(fOffsetAngle);
        dStep = fabs(dStep); //returnn the absolute value
        if (dStep > fOffsetAngle)
        {
          LOG_INFO("the step value is larger than offseAngle");
          return;
        }
        if (is_robot_connected_)
        {
          m_vecPose.resize(0);
          //calculte how many step should be included
          int nNumberofStep = int(floor(2 * fOffsetAngle / dStep)); //get round down

          //current pose
          auto robot_pose = getCurrentRobotTransformMatrix(true);
          Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
          Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //

          // initial pose and final pose
          Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
          if (ROTATION_X == nRotationAxis)
          {
            offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI; //unit rad
            LOG_INFO(fOffsetAngle);
            LOG_INFO(dStep);
            LOG_INFO(nNumberofStep);
          }
          else if (ROTATION_Y == nRotationAxis)
          {
            offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI; //unit rad
          }
          else
          {
            LOG_INFO("the rotation is wrong");
          }

          Eigen::Vector3d initEulerAngle = eulerAngles - offsetAngleRad;
          Eigen::Vector3d finalEulerAngle = eulerAngles + offsetAngleRad;
          LOG_INFO(initEulerAngle);
          LOG_INFO(finalEulerAngle);

          //back to pose(4*4 matrix)
          Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(initEulerAngle[0], Eigen::Vector3d::UnitX()) *
                                              Eigen::AngleAxisd(initEulerAngle[1], Eigen::Vector3d::UnitY()) *
                                              Eigen::AngleAxisd(initEulerAngle[2], Eigen::Vector3d::UnitZ());
          m_initialPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
          m_initialPose.block<3, 1>(0, 3) = translation / 1000.0;

          tempQuaternion = Eigen::AngleAxisd(finalEulerAngle[0], Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(finalEulerAngle[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(finalEulerAngle[2], Eigen::Vector3d::UnitZ());
          m_finalPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
          m_finalPose.block<3, 1>(0, 3) = translation / 1000.0;

          //save the trarget pose in a row into a vector.
          //save EulerAngles in a row
          QVector<Eigen::Vector3d> vecEulerAngs;
          Eigen::Vector3d vecTempEulerAng(0.0, 0.0, 0.0);
          //save path to final position
          for (int i = 0; i <= nNumberofStep; i++)
          {
            if (ROTATION_X == nRotationAxis)
            {
              vecTempEulerAng = initEulerAngle;
              vecTempEulerAng[0] += i * dStep / 180.0 * M_PI;
              vecEulerAngs.append(vecTempEulerAng);
              std::cout << i << "      " << vecEulerAngs.at(i) << "   " << vecEulerAngs.size() << std::endl;
            }
            else if (ROTATION_Y == nRotationAxis)
            {
              LOG_INFO("ROTATION_Y");
              vecTempEulerAng = initEulerAngle;
              vecTempEulerAng[1] += i * dStep / 180.0 * M_PI;
              vecEulerAngs.append(vecTempEulerAng);
              std::cout << i << "      " << vecEulerAngs.at(i) << std::endl;
            }
          }
          vecEulerAngs.append(finalEulerAngle);
          //save path to inital pose back
          int nPoseNumber = vecEulerAngs.size();
          for (int i = 0; i < nPoseNumber; i++)
          {
            vecEulerAngs.append(vecEulerAngs.at(nPoseNumber - 1 - i));
            std::cout << i << "      " << nPoseNumber << std::endl;
          }

          //remove super close value
          nPoseNumber = vecEulerAngs.size();
          double dDifferenceX_Y = 0;
          QVector<int> vecIteration;
          for (int i = 1; i < nPoseNumber; i++)
          {
            dDifferenceX_Y = fabs(vecEulerAngs.at(i)[0] - vecEulerAngs.at(i - 1)[0]);
            dDifferenceX_Y += fabs(vecEulerAngs.at(i)[1] - vecEulerAngs.at(i - 1)[1]);
            if (dDifferenceX_Y < 0.0003)
            {
              vecIteration.append(i);
            }
          }

          int nRemoveNumber = vecIteration.size();
          for (int i = 0; i < nRemoveNumber; i++)
          {
            vecEulerAngs.remove(vecIteration.at(nRemoveNumber - 1 - i));
          }

          //From eluar to matrix into vector
          nPoseNumber = vecEulerAngs.size();
          LOG_INFO(nPoseNumber);
          Eigen::Matrix4d matTempPose{Eigen::Matrix4d::Identity()};
          for (int i = 0; i < nPoseNumber; i++)
          {
            tempQuaternion = Eigen::AngleAxisd(vecEulerAngs.at(i)[0], Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(vecEulerAngs.at(i)[1], Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(vecEulerAngs.at(i)[2], Eigen::Vector3d::UnitZ());
            matTempPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
            matTempPose.block<3, 1>(0, 3) = translation / 1000.0;
            m_vecPose.append(matTempPose);
            std::cout << i << ":      " << vecEulerAngs.at(i)[0] << "     " << vecEulerAngs.at(i)[1] << "     " << vecEulerAngs.at(i)[2] << std::endl;
          }
        }
        else
        {
          LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
        }
      }

      //step by step rotate around TCP
      void PluginAlgorithm::calculateEndPointsStepTCP(double fOffsetAngle, double dStep, int nRotationAxis)
      {
        fOffsetAngle = fabs(fOffsetAngle);
        dStep = fabs(dStep); //returnn the absolute value
        if (dStep > fOffsetAngle)
        {
          LOG_INFO("the step value is larger than offseAngle");
          return;
        }
        if (is_robot_connected_)
        {
          m_vecPose.resize(0);
          //calculte how many step should be included
          int nNumberofStep = int(floor(2 * fOffsetAngle / dStep)); //get round down

          //current pose
          auto robot_pose = getCurrentRobotTransformMatrix(true);
          Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
          Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //

          // initial pose and final pose
          Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
          if (ROTATION_X == nRotationAxis)
          {
            offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI; //unit rad
            LOG_INFO(fOffsetAngle);
            LOG_INFO(dStep);
          }
          else if (ROTATION_Y == nRotationAxis)
          {
            offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI; //unit rad
          }
          else
          {
            LOG_INFO("the rotation is wrong");
          }

          //save the trarget pose in a row into a vector.
          //save EulerAngles in a row
          QVector<Eigen::Vector3d> vecEulerAngs;
          Eigen::Vector3d vecTempEulerAng(0.0, 0.0, 0.0);
          //save path to final position
          for (int i = 0; i <= nNumberofStep; i++)
          {
            if (ROTATION_X == nRotationAxis)
            {
              vecTempEulerAng = -offsetAngleRad;
              vecTempEulerAng[0] += i * dStep / 180.0 * M_PI;
              vecEulerAngs.append(vecTempEulerAng);
              std::cout << i << "      " << vecEulerAngs.at(i) << "   " << vecEulerAngs.size() << std::endl;
            }
            else if (ROTATION_Y == nRotationAxis)
            {
              LOG_INFO("ROTATION_Y");

              vecTempEulerAng = -offsetAngleRad;
              vecTempEulerAng[1] += i * dStep / 180.0 * M_PI;
              vecEulerAngs.append(vecTempEulerAng);
              std::cout << i << "      " << vecEulerAngs.at(i) << std::endl;
            }
          }
          vecEulerAngs.append(offsetAngleRad);
          //save path to inital pose back
          int nPoseNumber = vecEulerAngs.size();
          for (int i = 0; i < nPoseNumber; i++)
          {
            vecEulerAngs.append(vecEulerAngs.at(nPoseNumber - 1 - i));
            std::cout << i << "      " << nPoseNumber << std::endl;
          }

          //remove super close value
          nPoseNumber = vecEulerAngs.size();
          double dDifferenceX_Y = 0;

          QVector<int> vecIteration;
          for (int i = 1; i < nPoseNumber; i++)
          {
            dDifferenceX_Y = fabs(vecEulerAngs.at(i)[0] - vecEulerAngs.at(i - 1)[0]);
            dDifferenceX_Y += fabs(vecEulerAngs.at(i)[1] - vecEulerAngs.at(i - 1)[1]);
            if (dDifferenceX_Y < 0.0003)
            {
              vecIteration.append(i);
            }
          }

          int nRemoveNumber = vecIteration.size();
          for (int i = 0; i < nRemoveNumber; i++)
          {
            vecEulerAngs.remove(vecIteration.at(nRemoveNumber - 1 - i));
          }

          //From eluar to matrix into vector
          nPoseNumber = vecEulerAngs.size();
          LOG_INFO(nPoseNumber);
          Eigen::Matrix4d matTempPose{Eigen::Matrix4d::Identity()};
          for (int i = 0; i < nPoseNumber; i++)
          {
            Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(vecEulerAngs.at(i)[0], Eigen::Vector3d::UnitX()) *
                                                Eigen::AngleAxisd(vecEulerAngs.at(i)[1], Eigen::Vector3d::UnitY()) *
                                                Eigen::AngleAxisd(vecEulerAngs.at(i)[2], Eigen::Vector3d::UnitZ());
            matTempPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
            matTempPose.block<3, 1>(0, 3) << 0, 0, 0;
            matTempPose = robot_pose * matTempPose;
            matTempPose.block<3, 1>(0, 3) = translation / 1000.0;
            m_vecPose.append(matTempPose);
          }
        }
        else
        {
          LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
        }
      }

      //calculate the relative angle between initial TCP frame and current frame
      //Eigen::Matrix3d& initialFrame: reference frame, recorded when we start the fan motion
      //Eigen::Matrix3d& currentFrame: real-time frame, recorded when we carry on the fan motion
      Eigen::Vector3d PluginAlgorithm::calculateReleativeRatationAngle(Eigen::Matrix3d & initialFrame, Eigen::Matrix3d & currentFrame)
      {
        Eigen::Matrix3d matRelatePose = initialFrame.inverse() * currentFrame;
        Eigen::Vector3d relativeEularAngle = matRelatePose.eulerAngles(0, 1, 2);
        return relativeEularAngle;
      }

      void PluginAlgorithm::RotateAroundTCP(double fOffsetAngle, int nRotationAxis)
      {
        auto robot_pose = getCurrentRobotTransformMatrix(true);
        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
        Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
        if (ROTATION_X == nRotationAxis)
        {
          offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI; //unit rad
          LOG_INFO(fOffsetAngle);
        }
        else if (ROTATION_Y == nRotationAxis)
        {
          offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI; //unit rad
        }
        else
        {
          LOG_INFO("the rotation is wrong");
        }
        //obtain the transformation between the target about TCP frame
        Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                                            Eigen::AngleAxisd(offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(offsetAngleRad[2], Eigen::Vector3d::UnitZ());
        Eigen::Matrix4d PoseRotateTCP{Eigen::Matrix4d::Identity()};

        PoseRotateTCP.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
        PoseRotateTCP.block<3, 1>(0, 3) << 0, 0, 0;

        PoseRotateTCP = robot_pose * PoseRotateTCP;
        PoseRotateTCP.block<3, 1>(0, 3) = translation / 1000.0;

        executeCartesianCommand(PoseRotateTCP, true);
      }

      void PluginAlgorithm::onGotoPose(const Eigen::Matrix4d &pose, bool callback)
      {
        if (callback)
        {
          executeCartesianCommand(pose, true,
                                  std::bind(&PluginAlgorithm::movementFinishCallback, this));
        }
        else
        {
          executeCartesianCommand(pose, true);
        }
      }

      void PluginAlgorithm::onStepGotoPose(const Eigen::Matrix4d &pose, bool callback)
      {
        if (callback)
        {
          executeCartesianCommand(pose, true,
                                  std::bind(&PluginAlgorithm::stepMovementFinishCallback, this));
        }
        else
        {
          executeCartesianCommand(pose, true);
        }
      }

      //vision control
      void PluginAlgorithm::onVisionStepGotoPose(const Eigen::Quaterniond &qPose, const Eigen::Vector3d &translation, bool callback)
      {
        if (callback)
        {
          executeCartesianCommand(qPose, translation, true,
                                  std::bind(&PluginAlgorithm::VisionMovementFinishCallback, this));
        }
        else
        {
          executeCartesianCommand(qPose, translation, true);
        }
      }

      int PluginAlgorithm::onGetStepFanMotionIteration()
      {
        return m_nIteration;
      }

      Eigen::Matrix4f PluginAlgorithm::getHomoTransMatrix(const std::vector<Eigen::Vector3f> &srcPoints, const std::vector<Eigen::Vector3f> &dstPoints)
      {
        Eigen::Vector3f centroidSrc, centroidDst;
        float sumXSrc = 0, sumYSrc = 0, sumZSrc = 0, sumXDst = 0, sumYDst = 0, sumZDst = 0;
        int pointsNum = static_cast<int>(srcPoints.size());
        for (size_t i = 0; i < srcPoints.size(); i++)
        {
          sumXSrc += srcPoints[i](0);
          sumYSrc += srcPoints[i](1);
          sumZSrc += srcPoints[i](2);
          sumXDst += dstPoints[i](0);
          sumYDst += dstPoints[i](1);
          sumZDst += dstPoints[i](2);
        }
        centroidSrc << static_cast<float>(sumXSrc / pointsNum), static_cast<float>(sumYSrc / pointsNum), static_cast<float>(sumZSrc / pointsNum);
        centroidDst << static_cast<float>(sumXDst / pointsNum), static_cast<float>(sumYDst / pointsNum), static_cast<float>(sumZDst / pointsNum);

        Eigen::MatrixXf src(3, 4), dst(3, 4), dstCopy(3, 4);
        src << srcPoints[0](0) - centroidSrc(0), srcPoints[1](0) - centroidSrc(0), srcPoints[2](0) - centroidSrc(0), srcPoints[3](0) - centroidSrc(0),
            srcPoints[0](1) - centroidSrc(1), srcPoints[1](1) - centroidSrc(1), srcPoints[2](1) - centroidSrc(1), srcPoints[3](1) - centroidSrc(1),
            srcPoints[0](2) - centroidSrc(2), srcPoints[1](2) - centroidSrc(2), srcPoints[2](2) - centroidSrc(2), srcPoints[3](2) - centroidSrc(2);
        dst << dstPoints[0](0) - centroidDst(0), dstPoints[1](0) - centroidDst(0), dstPoints[2](0) - centroidDst(0), dstPoints[3](0) - centroidDst(0),
            dstPoints[0](1) - centroidDst(1), dstPoints[1](1) - centroidDst(1), dstPoints[2](1) - centroidDst(1), dstPoints[3](1) - centroidDst(1),
            dstPoints[0](2) - centroidDst(2), dstPoints[1](2) - centroidDst(2), dstPoints[2](2) - centroidDst(2), dstPoints[3](2) - centroidDst(2);
        dstCopy = dst;

        dst = dstCopy.transpose(); //转置后一行一个点
        Eigen::Matrix3f S0 = src * dst;
        Eigen::BDCSVD<Eigen::MatrixXf> svd(S0, Eigen::ComputeThinU | Eigen::ComputeThinV);
        LOG_INFO("V Matrix");
        LOG_INFO(svd.matrixV());
        LOG_INFO("U Matrix");
        LOG_INFO(svd.matrixU());
        Eigen::MatrixXf M(3, 3);
        M = Eigen::MatrixXf::Identity(3, 3);
        M(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
        Eigen::Matrix3f rotMatrix = svd.matrixV() * M * (svd.matrixU().transpose());
        Eigen::Vector3f T = centroidDst - rotMatrix * centroidSrc;
        Eigen::Matrix4f HT;
        HT << rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2), T(0),
            rotMatrix(1, 0), rotMatrix(1, 1), rotMatrix(1, 2), T(1),
            rotMatrix(2, 0), rotMatrix(2, 1), rotMatrix(2, 2), T(2),
            0, 0, 0, 1;
        return HT;
      }

    } // namespace OrienAdj
  }   // namespace ImFusion
