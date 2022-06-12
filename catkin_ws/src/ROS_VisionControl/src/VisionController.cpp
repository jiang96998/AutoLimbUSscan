#include "vision_control/VisionController.h"
#include "ui_controller.h"
#include <iiwa_msgs/DOF.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <cstring>

namespace ImFusion
{
  namespace vision_control
  {

    PluginController::PluginController(PluginAlgorithm *algorithm)
        : AlgorithmController(algorithm), algorithm_{algorithm}
    {
      ui_ = std::make_shared<Ui_Controller>();
      ui_->setupUi(this);
      ui_->cmbRotationStep->setEnabled(false); //disable the step value when start the plugin

      //TCP
      tcpServer = new QTcpServer(this);
      serverSocket = new QTcpSocket(this);
      m_forceData.resize(6);

      //synchronous acquisition
      m_recordTimer = new QTimer(this);
      m_recordTimer->setTimerType(Qt::PreciseTimer);

      m_updateTimer = new QTimer(this);
      m_updateTimer->setTimerType(Qt::PreciseTimer);

      //vision control
      m_breakPointIndex = 0;
      m_confiMapConvex = new ConfidentMapConvex();
      timerForMove=new QTimer(this);
      timerForMove-> setInterval(4*1000);
      LOG_INFO("初始化");
      //m_check_movement_sub =
    }

    void PluginController::init()
    {
      addToAlgorithmDock();

      //update the robot state after checking checking box
      
      connect(ui_->pbtnStartUSStream, &QPushButton::clicked, this, &PluginController::onStartUSStreamClicked);
      connect(ui_->pbtnStopUSStream, &QPushButton::clicked, this, &PluginController::onStopUSStreamClicked);
      connect(ui_->pbtnLoadSweep_imf, &QPushButton::clicked, this, &PluginController::onLoadImfSweepClicked);
      connect(ui_->pbtnComputerConfiMap, &QPushButton::clicked, this, &PluginController::onComputeConfiMapClicked);
      connect(ui_->pbtnVisualizeConfimap, &QPushButton::clicked, this, &PluginController::onVisualizeConfiMapClicked);
      connect(ui_->pbtn_test_rotation, &QPushButton::clicked, this, &PluginController::onTestRotationButtonClicked);

      // to connect and disconnect the robot with the tool used.
      connect(ui_->pbtnConnect, &QPushButton::clicked, this, &PluginController::onConnectToRobotClick);
      connect(ui_->pbtnDisconnect, &QPushButton::clicked, this, &PluginController::onDisconnectFromRobotClick);

      // to use it in force mode set the stiffness and force and click on the force mode, else use the
      // position mode
      connect(ui_->pbtnForceMode, &QPushButton::clicked, this, &PluginController::onForceModeClick);
      connect(ui_->pbtnPositionMode, &QPushButton::clicked, this, &PluginController::onPositionModeClick);

      connect(ui_->pbtnSaveHome, &QPushButton::clicked, this, &PluginController::onSaveHome);
      connect(ui_->pbtnSaveOut, &QPushButton::clicked, this, &PluginController::onSaveOut);
      connect(ui_->pbtnGoHome, &QPushButton::clicked, this, &PluginController::onGoHome);
      connect(ui_->pbtnGoOut, &QPushButton::clicked, this, &PluginController::onGoOut);
      connect(ui_->pbtnExcuteCmd, &QPushButton::clicked, this, &PluginController::onExecuteMovementClick);

      //update the state when the state changed! changed to use timer
      connect(algorithm_, &PluginAlgorithm::poseChanged, this, &PluginController::updateUIToLastPose);
      connect(algorithm_, &PluginAlgorithm::wrenchChanged, this, &PluginController::updateUIToLastWrench);

      //Synchronous acquisition
      QObject::connect(m_recordTimer, &QTimer::timeout, this, &PluginController::onSynRecord);

      //update the robot state in UI
      QObject::connect(ui_->chbRecordWrench, &QCheckBox::stateChanged, this, &PluginController::onchbRecordStateChanged);
      QObject::connect(ui_->chbUpdateState, &QCheckBox::stateChanged, this, &PluginController::onChbUpdateStateChanged);
      QObject::connect(m_updateTimer, &QTimer::timeout, this, &PluginController::onUpdateState);

      QObject::connect(timerForMove, &QTimer::timeout, this, &PluginController::onMoveFeedBack);
      //VISION CONTROL PLUGIN

      // read the ultrasound poses from a file
      connect(ui_->pbtnReadPosesFromFile, &QPushButton::clicked, this, &PluginController::onReadPosesClick);

      // execute the ultrasound scan, and stop it anytime with a button
      connect(ui_->pbtnExecScan, &QPushButton::clicked, this, &PluginController::onExecScanClick);

      // go to the initial pose in the list of scan poses.
      connect(ui_->pbtnGoIntiPos, &QPushButton::clicked, this, &PluginController::onGoInitPosClick);

      // once the continue scan is clicked then the trasnformation will be calculated
      // the robot will move to the position where it left off and then will start scaning again.
      connect(ui_->pbtnContinueScan, &QPushButton::clicked, this, &PluginController::onContinueScanClicked);

      // start and stop the ultrasound sweep recording

      // load and do 3d compound of the sweep
      connect(ui_->pbtnDownsampleSweep, &QPushButton::clicked, this, &PluginController::onDownsampleUSSweepClicked);
      connect(ui_->pbtnCreateUSForLabelsWithoutStich, &QPushButton::clicked, this, &PluginController::onClickedCreateSweepFromLabel);
      connect(ui_->pbtn_stitching, &QPushButton::clicked, this, &PluginController::onClickedStitching);
      connect(ui_->pbtnWriteCurrRobotPose, &QPushButton::clicked, this, &PluginController::onClickedWriteRobotPose);

      connect(ui_->pbtnInitROS, &QPushButton::clicked, this, &PluginController::onClickedpbtnInitROS);

      //yuangao
      connect(ui_->stop_toggle,&QCheckBox::clicked, this, &PluginController::onToggleStopRobot);
      connect(ui_->feedback_toggle,&QCheckBox::clicked, this, &PluginController::onToggleFeedBack);
      connect(ui_->GenerateButton,&QPushButton::clicked, this, &PluginController::onWriteImf);
    }

    void PluginController::onToggleStopRobot()
    {
      if(ui_->stop_toggle->isChecked())
      {
        algorithm_->stopRobot=true;
      }
      else
      {
        algorithm_->stopRobot=false;
      }
    }
    void PluginController::onToggleFeedBack()
    {
      if(ui_->feedback_toggle->isChecked())
      {
        // onWirteImf();
        algorithm_->feedback=true;
        // algorithm_->setUnetParam(false);
      }
      else
      {
        algorithm_->feedback=false;
        // algorithm_->setUnetParam(false);
      }
    }
                                                                                                                                                                                                                                       
    void PluginController::onWriteImf()
    {
      // QString szFileName1 = "/home/y/Downloads/Patient-01-21.imf";
      // QString szFileName2 = "after.imf";
      char* index="77";
      DataList labelUSBefore = readImage_IMF("/var/tmp/ImFusionRecordedSweeps/Patient-01/",std::string("Patient-01-")+index+".imf");
      // DataList labelUSAfter = readImage_IMF(szFilePath, szFileName4);

      // m_main->dataModel()->add(USImageDataAfter.getImage(Data::UNKNOWN,Data::ULTRASOUND));
      UltrasoundSweep* labelSweepBefore = static_cast<UltrasoundSweep*>(labelUSBefore.getItem(0));

      SharedImageSet* usSweepUncompensated = new SharedImageSet();

      for(int i=0;i<4800;i=i+2)
      {
        std::cout << "------------------------------------------ " << std::endl;
        // cv::Mat img=cv::imread("/home/y/RosCode/catkin_ws/src/vesselSegment/models/OFunet/label/"+std::to_string(i)+".png");
        cv::Mat img=cv::imread(std::string("/home/zhongliang/yuangao/roscode/catkin_ws/src/vesselSegment/models/OFunet/fusion")+index+"/"+std::to_string(i)+".png");
        cv::Mat grayImg;
        cv::cvtColor(img,grayImg,cv::COLOR_BGR2GRAY);
        int nHeight = img.rows;
        int nWidth = img.cols;
        cv::Mat cvimg = cv::Mat(nHeight, nWidth, CV_8UC1, cv::Scalar(255));
        grayImg.convertTo(cvimg, CV_8UC1, 1);
        // cv::imshow("clock",cvimg);
        // cv::waitKey(0);


        MemImage *memImage = MemImage::create(Image::UBYTE, nWidth, nHeight, 1, 1);
        if (memImage->byteSize() == cvimg.total() * cvimg.elemSize())
        {
          memcpy(memImage->data(),cvimg.data, nWidth * nHeight * sizeof(uchar));
        }
        else
        {
        LOG_ERROR("onSaveImageClick: byteSize is different");
        qDebug()<<"memImage->byteSize():"<<memImage->byteSize();
        qDebug()<<"cv_imgIn->byteSize():"<<cvimg.total() * cvimg.elemSize();        
        // std::cout << "memImage size: " << memImage->byteSize() << std::endl;
        // std::cout << "cvimg.data size: " << cvimg.total() * cvimg.elemSize() << std::endl;
        // std::cout << "copy size: " << nWidth * nHeight * sizeof(uchar) << std::endl;
        }



        memImage->setSpacing(0.0765, 0.0765, 1);
        std::cout << "second spacing : " << memImage->spacing() << std::endl;
        memImage->flip(1);
        m_confiMapConvex->saveImageToDisc(memImage);
        Image imageTep(Image::UBYTE, nWidth, nHeight, 1, 1); //same as previous defined memImage
        qDebug() << "nHeight:" << imageTep.height() << ";   "
                << "nWidth:" << imageTep.width();

        //transfer memimage to shareimage
        SharedImage *sharedImageTemp = new SharedImage(imageTep);
        sharedImageTemp->update(*memImage);

        


        
        //added sharedimage to sharedimageSet
        SharedImageSet *sharedImageSetTemp = new SharedImageSet(); //new is very important!!
        sharedImageSetTemp->add(sharedImageTemp);
        sharedImageSetTemp->setModality(Data::ULTRASOUND);
        sharedImageSetTemp->get()->setDirtyMem(); //don't know

        Selection sel;
        sel.setSingle(0);
        sharedImageSetTemp->setSelection(sel);

        qDebug() << "sharedImage kind:" << sharedImageSetTemp->kind();
        qDebug() << "sharedImage Modality:" << sharedImageSetTemp->modality();

        //added sharedimageSet to datalist
        DataList *dataListTemp = new DataList();
        dataListTemp->add(sharedImageSetTemp);  
        qDebug() << "dataListTemp:" << dataListTemp->getImage()->mem()->height() << ";   "
               << "nWidth:" << dataListTemp->getImage()->mem()->width();

        //    setImageDir(dataListTemp);
        ImFusion::mat4 sMatrix = labelSweepBefore->matrix(i);
        // ImFusion::mat4 tempMatrix = sMatrix;
        // usSweepUncompensated->add(labelSweepAfter->get(0), tempMatrix);
        usSweepUncompensated->add(sharedImageTemp, sMatrix);
      }     

      BackgroundExporter* sweepExporter = new BackgroundExporter();
      sweepExporter->save(usSweepUncompensated, std::string("/home/zhongliang/yuangao/download/OFfusionwithpose")+index+".imf");

    }

    //读取扫描路径坐标
    void PluginController::onReadPosesClick()
    {

      // the number of points in the scan trajectory is equal to 0.
      algorithm_->m_scanPointsNumber = 0;

      std::string s;
      std::string scan_trajectory_file_path = ui_->ledt_poses_file_path->text().toStdString();
      std::ifstream scan_poses_file(scan_trajectory_file_path);
      // the points in the trajectory txt file, read and written to the m_scanPointPoses vector
      algorithm_->m_scanPointPoses.clear();
      // algorithm_->m_nIteration=0;
      if (scan_poses_file.is_open())
      {
        while (getline(scan_poses_file, s))
        {
          Eigen::VectorXd pointPose(7);
          int ind = 0;
          // parse the string
          std::string delimiter = " ";
          size_t pos = 0;
          std::string token;
          while ((pos = s.find(delimiter)) != std::string::npos)
          {
            token = s.substr(0, pos);
            pointPose(ind) = std::stod(token);
            s.erase(0, pos + delimiter.length());
            ind += 1;
          }
          pointPose(ind) = std::stod(s);
          algorithm_->m_scanPointPoses.push_back(pointPose);
          std::cout << "The number of poses : " << algorithm_->m_scanPointPoses.size() << algorithm_->m_scanPointsNumber << std::endl;

          algorithm_->m_scanPointsNumber += 1;
        }
        scan_poses_file.close();
      }
      
      LOG_WARN("The number of points in the trajectory is: ");
      LOG_INFO(algorithm_->m_scanPointPoses.size());
    }

    //初始化ros节点
    void PluginController::onClickedpbtnInitROS()
    {
      algorithm_->onInitROS();
    }

    //记录机械臂末端坐标（用于手眼标定）
    void PluginController::onClickedWriteRobotPose()
    {
      std::ofstream outfile;
      std::string file_path = ui_->ledt_trajectory_file_to_write->text().toStdString();
      outfile.open(file_path, std::ios_base::app); // append instead of overwrite
      auto pose = algorithm_->getCurrentRobotPose();
      // outfile << std::to_string(pose.pose.position.x) << " " << std::to_string(pose.pose.position.y)
      //         << " " << std::to_string(pose.pose.position.z) << " " << std::to_string(pose.pose.orientation.x)
      //         << " " << std::to_string(pose.pose.orientation.y) << " " << std::to_string(pose.pose.orientation.z)
      //         << " " << std::to_string(pose.pose.orientation.w) << "\n";
      outfile << std::to_string(pose.pose.position.x) << " " << std::to_string(pose.pose.position.y)
              << " " << std::to_string(pose.pose.position.z) << "\n";
      outfile.close();
    }

    //位移后从断点重新扫描
    void PluginController::onContinueScanClicked()
    {
      LOG_WARN("Rerouting the path...");
      // move robot back to break point
      // start robot button is pressed
      algorithm_->transform_trajectory();

      // wait until the transformation message comes
      geometry_msgs::TransformStamped transformation;
      transformation = *(ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/transformation"));

      // apply the transformation to the poses

      geometry_msgs::PointStamped transformed_pose;
      geometry_msgs::PointStamped initial_pose;

      LOG_ERROR(transformation);

      // the break point is one before the current destination position
      // if we are going from 0 to 1 then our m_nIteration should be 1
      // if the motion occures at this point then the robot needs to go back to 0 because we are sure we arrived
      // m_nIteration-1 st point already.
      // so the break point will be equal to m_nIteration - 1
      m_breakPointIndex = algorithm_->m_break_point;
      algorithm_->write_txt_file("before_transform_whole");
      algorithm_->write_txt_file("before_transform_first_half", m_breakPointIndex, true);
      algorithm_->write_txt_file("before_transform_second_half", m_breakPointIndex, false);

      for (auto &pose : this->algorithm_->m_scanPointPoses)
      {
        initial_pose.point.x = pose(0);
        initial_pose.point.y = pose(1);
        initial_pose.point.z = pose(2);

        tf2::doTransform(initial_pose, initial_pose, transformation);

        pose(0) = initial_pose.point.x;
        pose(1) = initial_pose.point.y;
        pose(2) = initial_pose.point.z;

        std::cout << pose << std::endl;
      }

      algorithm_->write_txt_file("after_transform_whole");
      algorithm_->write_txt_file("after_transform_first_half", m_breakPointIndex, true);
      algorithm_->write_txt_file("after_transform_second_half", m_breakPointIndex, false);

      double x = transformation.transform.translation.x;
      double y = transformation.transform.translation.y;
      double z = transformation.transform.translation.z;

      double qw = transformation.transform.rotation.w;
      double qx = transformation.transform.rotation.x;
      double qy = transformation.transform.rotation.y;
      double qz = transformation.transform.rotation.z;

      Eigen::Matrix4f movement_transform;
      movement_transform << 2 * (std::pow(qw, 2) + std::pow(qx, 2)) - 1, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy), x,
          2 * (qx * qy + qw * qz), 2 * (std::pow(qw, 2) + std::pow(qy, 2)) - 1, 2 * (qy * qz - qw * qx), y,
          2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 2 * (std::pow(qw, 2) + std::pow(qz, 2)) - 1, z,
          0, 0, 0, 1;

      algorithm_->write_transformation(movement_transform);
      algorithm_->setCurrentScanPointsIteration(m_breakPointIndex);

      //reset variables for storing break point infos
      m_breakPointIndex = 0;
      algorithm_->m_MoveStatus = ON_INITIAL_POSE;
      algorithm_->m_resetting_position = true;

      auto cepha_stream = m_main->dataModel()->get("Cephasonics Main Stream");
      auto robot_stream = m_main->dataModel()->get("Robot Tracking");
      algorithm_->set_new_stream(cepha_stream, robot_stream);

      algorithm_->VisionMovementFinishCallback();
    }

    /*****************************************
 * This function execute scan task
 ***************************************/
    //开始扫描
    void PluginController::onExecScanClick()
    {
      if (algorithm_->m_scanPointsNumber == 0)
      {
        std::cout << "No scan points!" << std::endl;
        exit(EXIT_FAILURE);
      }
      else
      {
        // Once the execute scanning button pressed, the recording of the US sweep will start as well.
        // start_recording();//暂时不录
        algorithm_->setCurrentScanPointsIteration(0);//从头开始扫描
        // algorithm_->VisionMovementFinishCallback();//
        timerForMove->start();
      }
    }
    void PluginController::onMoveFeedBack()
    {
      algorithm_->VisionMovementFinishCallback();
      if(algorithm_->stopRobot==true)
      {
        timerForMove->stop();
      }
    }

    //先将机械臂移动到第一个扫描点的上方
    void PluginController::onGoInitPosClick()
    {
      if (algorithm_->m_scanPointsNumber == 0)
      {
        std::cout << "There are no points in the scan trajectory." << std::endl;
        exit(EXIT_FAILURE);
      }
      else
      {
        algorithm_->m_transformation_count = 0;
        Eigen::Quaterniond quaPose(algorithm_->m_scanPointPoses[0](6), algorithm_->m_scanPointPoses[0](3), algorithm_->m_scanPointPoses[0](4), algorithm_->m_scanPointPoses[0](5));
        // Eigen::Vector3d vecPosition(algorithm_->m_scanPointPoses[0](0), algorithm_->m_scanPointPoses[0](1), algorithm_->m_scanPointPoses[0](2) + 50.0f / 1000.0f);
        Eigen::Vector3d vecPosition(algorithm_->m_scanPointPoses[0](0), algorithm_->m_scanPointPoses[0](1), algorithm_->m_scanPointPoses[0](2));
        LOG_INFO("position:");
        LOG_INFO(vecPosition);
        algorithm_->executeCartesianCommand(quaPose, vecPosition, false);
        // at the moment we are in the position -1
        // next position needs to be the first point of the sweep
        algorithm_->m_MoveStatus = ON_INITIAL_POSE;
        algorithm_->m_nIteration=0;
      }
    }

    //执行运动指令，将机械臂移动到文本框中位置
    void PluginController::onExecuteMovementClick()
    {
      Eigen::Vector3d rotAngles(ui_->ledtAngRa->text().toDouble(), ui_->ledtAngRb->text().toDouble(),
                                ui_->ledtAngRc->text().toDouble());
      rotAngles *= M_PI / 180.; //rad
      algorithm_->executeCartesianCommand(
          Eigen::AngleAxisd(rotAngles[0], Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(rotAngles[1], Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(rotAngles[2], Eigen::Vector3d::UnitZ()),
          Eigen::Vector3d(ui_->ledtPosX->text().toDouble() / 1000, ui_->ledtPosY->text().toDouble() / 1000,
                          ui_->ledtPosZ->text().toDouble() / 1000),
          true);
    }

    //位置模式
    void PluginController::onPositionModeClick()
    {
      LOG_INFO("Applying position model!");
      algorithm_->applyPositionControlMode();
    }

    //阻力模式
    void PluginController::onForceModeClick()
    {
      LOG_INFO("Applying Force!");
      algorithm_->applyDesiredForce(iiwa_msgs::DOF::Z, ui_->ledtDesiredForce->text().toDouble(),
                                    ui_->ledtStiffness->text().toDouble());
    }

    //
    void PluginController::onSaveOut()
    {
      if (algorithm_->isRobotConnected())
      {
        algorithm_->setRobotOutConfiguration(algorithm_->getCurrentRobotPose());
      }
    }

    //存储home点
    void PluginController::onSaveHome()
    {
      if (algorithm_->isRobotConnected())
      {
        algorithm_->setRobotHomeConfiguration(algorithm_->getCurrentRobotPose());
      }
    }

    //将机械臂移动到home点
    void PluginController::onGoHome()
    {
      LOG_INFO("enter onGoHome");
      algorithm_->executeCartesianCommand(algorithm_->getRobotHomeConfiguration().pose, true);
    }

    //
    void PluginController::onGoOut()
    {
      algorithm_->executeCartesianCommand(algorithm_->getRobotOutConfiguration().pose, true);
    }

    //连接机械臂
    void PluginController::onConnectToRobotClick()
    {
      algorithm_->connect("IFLConvex");
      algorithm_->addStreamToDataModel(m_main->dataModel());

      emit robotConnected(); //! Emit connection signal.
    }

    //断开与机械臂的连接
    void PluginController::onDisconnectFromRobotClick()
    {
      algorithm_->disconnect();
      emit robotDisconnected(); //! Emit disconnection signal.
    }

    //在窗口中更新机械臂的位姿
    void PluginController::updateUIToLastPose()
    {
      if (algorithm_->isRobotConnected())
      {
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);
        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);                        //mm
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //rad

        if (ui_->chbUpdateState->isChecked())
        {
          ui_->ledtPosX->setText(QString::number(translation.x()));
          ui_->ledtPosY->setText(QString::number(translation.y()));
          ui_->ledtPosZ->setText(QString::number(translation.z()));
          ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
          ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
          ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));
        }
      }
    }

    //在窗口中更新机械臂的力矩信息
    void PluginController::updateUIToLastWrench()
    {
      if (algorithm_->isRobotConnected())
      {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();

        if (ui_->chbUpdateState->isChecked())
        {
          ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
          ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));
          ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
          ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
          ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));
          ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));
        }
      }
    }

    //
    void PluginController::onLoadImfSweepClicked()
    {
      std::string file_path = ui_->ledt_full_path_to_imf->text().toStdString();

      DataList USImageData = readImage_IMF(file_path);
      m_main->dataModel()->add(USImageData.get(Data::UNKNOWN));
      m_disp->update();

      SharedImageSet *sharedImage = USImageData.getImage(Data::UNKNOWN);
      qDebug() << "sharedImage kind:" << sharedImage->kind();
      qDebug() << "sharedImage Modality:" << sharedImage->modality();
      MemImage *memImage = sharedImage->mem()->clone();

      int nImageHeight = sharedImage->mem()->height();
      int nImageWidth = sharedImage->mem()->width();
      qDebug() << nImageHeight << "       " << nImageWidth;
      cv::Mat cv_imgIn(nImageHeight, nImageWidth, CV_8UC1); //uchar
      m_confiMapConvex->setHeightRaw(nImageHeight);
      m_confiMapConvex->setWidthRaw(nImageWidth); //save the size of the raw image

      memcpy(cv_imgIn.data, memImage->data(), memImage->byteSize());

      m_confiMapConvex->setCvMatRaw(cv_imgIn.clone());
      m_confiMapConvex->m_bFlagImgRaw = true;
    }

    //
    void PluginController::add_image_to_imfusion(cv::Mat cvMatConfiMapUChar)
    {

      int nHeight = cvMatConfiMapUChar.rows;
      int nWidth = cvMatConfiMapUChar.cols;
      MemImage *memImage = MemImage::create(Image::UBYTE, nWidth, nHeight, 1, 1);
      memcpy(memImage->data(), cvMatConfiMapUChar.data, nWidth * nHeight * sizeof(uchar));
      memImage->setSpacing(0.260189, 0.0768156, 1);
      std::cout << "second spacing : " << memImage->spacing() << std::endl;

      //    memImage->flip(1);

      Image imageTep(Image::UBYTE, nWidth, nHeight, 1, 1); //same as previous defined memImage
      qDebug() << "nHeight:" << imageTep.height() << ";   "
               << "nWidth:" << imageTep.width();

      //transfer memimage to shareimage
      SharedImage *sharedImageTemp = new SharedImage(imageTep);
      sharedImageTemp->update(*memImage);

      //added sharedimage to sharedimageSet
      SharedImageSet *sharedImageSetTemp = new SharedImageSet(); //new is very important!!
      sharedImageSetTemp->add(sharedImageTemp);
      sharedImageSetTemp->setModality(Data::ULTRASOUND);
      sharedImageSetTemp->get()->setDirtyMem(); //don't know

      Selection sel;
      sel.setSingle(0);
      sharedImageSetTemp->setSelection(sel);

      qDebug() << "sharedImage kind:" << sharedImageSetTemp->kind();
      qDebug() << "sharedImage Modality:" << sharedImageSetTemp->modality();

      //added sharedimageSet to datalist
      DataList *dataListTemp = new DataList();
      dataListTemp->add(sharedImageSetTemp);
      qDebug() << "dataListTemp:" << dataListTemp->getImage()->mem()->height() << ";   "
               << "nWidth:" << dataListTemp->getImage()->mem()->width();

      //    setImageDir(dataListTemp);
      //display it in imfusionstart
      m_main->dataModel()->add(*dataListTemp);
      m_disp->update();
    }

    //
    void PluginController::onComputeConfiMapClicked()
    {
      auto slice_idx = ui_->ledtSliceIndex->text().toInt();
      auto shared_ = m_main->selectedData()[0];
      SharedImageSet *sharedImageSet = static_cast<SharedImageSet *>(shared_);
      Selection sel;
      sel.setAll(sharedImageSet->size()); //select all images
      //sel.setSingle(slice_idx);   //select only  one image
      sharedImageSet->setSelection(sel);

      algorithm_->optimize_curr_probe_pose(sharedImageSet->get(slice_idx)->mem()->clone());
      add_image_to_imfusion(algorithm_->getConfimapResult());
      std::cout << "Direction" << std::endl;
      std::cout << algorithm_->getRotationDirection() << std::endl;
    }

    //
    void PluginController::onVisualizeConfiMapClicked()
    {
      //std::cout << std::make_shared<MemImage>(optimizer->getMemImage()).get() << std::endl;
      auto theta = algorithm_->optimize_curr_probe_pose(algorithm_->m_usListener->getMemImage());
      add_image_to_imfusion(algorithm_->getConfimapResult());
      std::cout << "theta: " << theta << std::endl;
      std::cout << "Direction: " << algorithm_->getRotationDirection() << std::endl;

      if (algorithm_->isRobotConnected())
      {
        if (algorithm_->getRotationDirection())
        {
          algorithm_->calculateEndPointsTCP(-1 * theta, ROTATION_X);
        }
        else
        {
          algorithm_->calculateEndPointsTCP(theta, ROTATION_X);
        }
        algorithm_->onGotoPose(algorithm_->m_finalPose, false);
      }
    }

    //
    void PluginController::onStopUSStreamClicked()
    {
      algorithm_->stop_streaming();
    }

    //
    void PluginController::onStartUSStreamClicked()
    {

      for (const auto &elem : m_main->dataModel()->getAll())
      {
        std::cout << elem->name() << std::endl;
      }

      auto cepha_stream = m_main->dataModel()->get("Cephasonics Main Stream");
      ImageStream *usStream = static_cast<ImageStream *>(cepha_stream);

      algorithm_->set_us_stream(usStream);
      algorithm_->start_streaming();
    }

    //TCPTrackingStreamData
    void PluginController::onRecForce()
    {
      tcpServer->listen(QHostAddress::Any, 9999);
      connect(tcpServer, SIGNAL(newConnection()), this, SLOT(acceptConnection()));
    }

    void PluginController::acceptConnection()
    {
      serverSocket = tcpServer->nextPendingConnection();
      connect(serverSocket, SIGNAL(readyRead()), this, SLOT(replyToClient()));
    }

    //
    void PluginController::replyToClient()
    {
      qDebug() << "here C1 bytes = " << serverSocket->bytesAvailable();
      if (serverSocket->bytesAvailable() > 100)
      {
        qDebug() << "MSG:" << serverSocket->readAll(); //avoid crash
        serverSocket->flush();
        return;
      }
      std::string msg = std::string(serverSocket->readAll());
      std::string delimiter = ";";

      size_t pos = 0;
      int nindex(0); // represent the index in force

      while ((pos = msg.find(delimiter)) != std::string::npos)
      {
        m_forceData[nindex] = std::stof(msg.substr(0, pos));
        msg.erase(0, pos + delimiter.length());
        nindex++;
      }
      LOG_INFO(m_forceData.at(1));
    }

    //Synchronous acquisition
    void PluginController::onSynRecord()
    {
      if (algorithm_->isRobotConnected())
      {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);

        ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
        ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));
        ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
        ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
        ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));
        ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));

        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);                        //mm
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //rad
        //(0,1,2)represents X-Y-Z，pitch yaw roll

        ui_->ledtPosX->setText(QString::number(translation.x()));
        ui_->ledtPosY->setText(QString::number(translation.y()));
        ui_->ledtPosZ->setText(QString::number(translation.z()));
        ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
        ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
        ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));

        if (ui_->chbRecordWrench->isChecked())
        {

          auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);

          //calculate the reference angle position between real time frame and inital frame
          Eigen::Matrix3d initialFrameFan = algorithm_->m_initialFrameFan.block<3, 3>(0, 0);
          Eigen::Matrix3d currentFrameFan = robot_pose.block<3, 3>(0, 0);
          Eigen::Vector3d reletiveAngle = algorithm_->calculateReleativeRatationAngle(initialFrameFan, currentFrameFan);

          QString path = QDir::currentPath();
          QDir dir;
          path = path + QString("/ros/zhongliang/");
          if (!dir.exists(path))
            dir.mkpath(path); // create the directory if needed
          QFile file(path + "ForceData.txt");
          QString szData;
          std::vector<float> forceData = m_forceData;
          if (file.open(QIODevice::ReadWrite | QIODevice::Append))
          {
            QTextStream streamOut(&file);
            streamOut /*<< szData.setNum(robot_wrench.wrench.force.x)
                                                              << "       "  //space of two TAB*/
                << QString::number(robot_wrench.force.x)
                << "       " //space of two TAB
                << QString::number(robot_wrench.force.y)
                << "       " //space of two TAB
                << QString::number(robot_wrench.force.z)
                << "       " //space of two TAB
                << QString::number(robot_wrench.torque.x)
                << "       " //space of two TAB
                << QString::number(robot_wrench.torque.y)
                << "       " //space of two TAB
                << QString::number(robot_wrench.torque.z)
                << "       "                         //space of two TAB
                << QString::number(robot_pose(0, 0)) //x
                << "       "                         //space of two TAB
                << QString::number(robot_pose(1, 0)) //x
                << "       "                         //space of two TAB
                << QString::number(robot_pose(2, 0)) //x
                << "       "                         //space of two TAB
                << QString::number(robot_pose(0, 1)) //y
                << "       "                         //space of two TAB
                << QString::number(robot_pose(1, 1)) //y
                << "       "                         //space of two TAB
                << QString::number(robot_pose(2, 1)) //y
                << "       "                         //space of two TAB
                << QString::number(robot_pose(0, 2)) //z
                << "       "                         //space of two TAB
                << QString::number(robot_pose(1, 2)) //z
                << "       "                         //space of two TAB
                << QString::number(robot_pose(2, 2)) //z
                << "       "                         //space of two TAB
                << QString::number(robot_pose(0, 3))
                << "       " //space of two TAB
                << QString::number(robot_pose(1, 3))
                << "       " //space of two TAB
                << QString::number(robot_pose(2, 3))
                << "       " //space of two TAB
                << QString::number(reletiveAngle(0))
                << "       " //space of two TAB
                << QString::number(reletiveAngle(1))
                << "       " //space of two TAB
                << QString::number(reletiveAngle(2))
                << "       " //space of two TAB
                << QString::number(forceData[0])
                << "       " //space of two TAB
                << QString::number(forceData[1])
                << "       " //space of two TAB
                << QString::number(forceData[2])
                << "       " //space of two TAB
                << QString::number(forceData[3])
                << "       " //space of two TAB
                << QString::number(forceData[4])
                << "       " //space of two TAB
                << QString::number(forceData[5])
                << "       " //space of two TAB
                << endl;
          }
          file.close();
        }
      }
    }

    //
    void PluginController::onchbRecordStateChanged()
    {
      if (true == ui_->chbRecordWrench->isChecked())
      {
        m_recordTimer->start(TIM_INTERVAL_REC);
      }
      else
      {
        m_recordTimer->stop();
      }
    }

    //更新机械臂状态
    void PluginController::onUpdateState()
    {
      if (algorithm_->isRobotConnected())
      {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();

        ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
        ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));
        ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
        ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
        ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));
        ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));
      }

      if (algorithm_->isRobotConnected())
      {
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);
        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);                        //mm
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //rad

        ui_->ledtPosX->setText(QString::number(translation.x()));
        ui_->ledtPosY->setText(QString::number(translation.y()));
        ui_->ledtPosZ->setText(QString::number(translation.z()));
        ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
        ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
        ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));
      }
    }

    //
    void PluginController::onChbUpdateStateChanged()
    {
      if (true == ui_->chbUpdateState->isChecked())
      {
        m_updateTimer->start(TIM_INTERVAL_UPDATE);
      }
      else
      {
        m_updateTimer->stop();
      }
    }

    void PluginController::start_recording()
    {
      LOG_ERROR("Do Record!");

      auto cepha_stream = m_main->dataModel()->get("Cephasonics Main Stream");
      auto robot_stream = m_main->dataModel()->get("Robot Tracking");

      algorithm_->set_new_stream(cepha_stream, robot_stream);
      algorithm_->start_recording_us_stream();
    }

    std::string PluginController::getDayAndTime()
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

    //
    void PluginController::read_and_sort_file_names_in_direactory(const char *path, std::vector<std::string> &file_names, const std::string &file_ending)
    {
      // read the names of the files
      // add them into a vector

      auto file_ending_size = file_ending.size();
      if (auto dir = opendir(path))
      {
        while (auto f = readdir(dir))
        {
          if (f->d_name[0] == '.')
            continue; // Skip everything that starts with a dot
          else
          {
            std::string file_name(f->d_name);
            if (file_name.substr(file_name.size() - file_ending_size) == file_ending)
            {
              file_names.push_back(file_name);
              //file_name.erase(file_name.length()- file_ending_size);
              //file_names.push_back(file_name);
            }
          }
        }
        closedir(dir);
      }

      // sort the files in the directory wrt. their names
      std::sort(file_names.begin(), file_names.end());
    }

    //读取超声图像
    void PluginController::read_ultrasound_data(const std::string &path, std::vector<UltrasoundSweep *> &ultrasound_sweeps)
    {
      auto sweep_data_list = readImage_IMF(path);
      ultrasound_sweeps.push_back(static_cast<UltrasoundSweep *>(sweep_data_list.getItem(0)));
    }

    //读取超声图像
    void PluginController::read_ultrasound_data(const std::string &path, const std::vector<std::string> &file_names, std::vector<UltrasoundSweep *> &ultrasound_sweeps)
    {
      for (const auto &file_name : file_names)
      {
        std::cout << file_name << std::endl;
        auto sweep_data_list = readImage_IMF(path, file_name);
        ultrasound_sweeps.push_back(static_cast<UltrasoundSweep *>(sweep_data_list.getItem(0)));
      }
    }

    //
    void PluginController::onDownsampleUSSweepClicked()
    {
      auto sweep_path = ui_->ledtSweepPath->text().toStdString();
      auto downsampling_param = ui_->ledtDownsamplingParam->text().toUInt();
      auto ultrasound_sweep = static_cast<UltrasoundSweep *>(readImage_IMF(sweep_path).getItem(0));

      SharedImageSet *usSweepFinal = new SharedImageSet();
      for (unsigned int i = 0; i < static_cast<unsigned int>(ultrasound_sweep->size()); i++)
      {
        if ((i % downsampling_param) == 0)
        {
          usSweepFinal->add(ultrasound_sweep->get(i), ultrasound_sweep->matrix(i));
        }
      }

      // write the final result into a file

      UltrasoundSweep *usSweepFinalUncompensated = static_cast<UltrasoundSweep *>(usSweepFinal);
      auto str = getDayAndTime();
      BackgroundExporter *sweepExporter = new BackgroundExporter();
      sweepExporter->save(usSweepFinalUncompensated, "/home/zhongliang/ros/nehil/markerless_motion_capture_for_RUSS/src/sweeps/dowsampled_by_" + std::to_string(downsampling_param) + "_" + str + ".imf");
    }

    void PluginController::write_sweep_into_file(SharedImageSet *final_sweep, std::string sweep_dir)
    {
      UltrasoundSweep *usSweepFinalUncompensated = static_cast<UltrasoundSweep *>(final_sweep);
      auto str = getDayAndTime();
      BackgroundExporter *sweepExporter = new BackgroundExporter();
      sweepExporter->save(usSweepFinalUncompensated, sweep_dir + str + ".imf");
    }

    //拼接超声图像
    void PluginController::onClickedStitching()
    {
      std::string sweep_dir = ui_->ledtDataDir->text().toStdString();
      add_slash(sweep_dir);

      auto sweep_path = sweep_dir + "sweeps";
      auto label_path = sweep_dir + "labels";
      auto transformations_path = sweep_dir + "transformation/";
      std::vector<std::string> sweep_files;
      std::vector<std::string> label_files;
      std::vector<std::string> transformation_files;
      char *path_sweep;
      char *path_label;
      char *path_transformation;
      path_sweep = &sweep_path[0];
      path_label = &label_path[0];
      path_transformation = &transformations_path[0];

      // read the names of the sweep files
      // add them into a vector
      read_and_sort_file_names_in_direactory(path_sweep, sweep_files, ".imf");

      // read the names of the sweep label files
      // add them into a vector
      read_and_sort_file_names_in_direactory(path_label, label_files, ".imf");

      // read the names of the transformation files
      // add them into a vector
      read_and_sort_file_names_in_direactory(path_transformation, transformation_files, ".txt");

      if (sweep_files.size() - transformation_files.size() == 1)
      {
        // read the sweep files into vector of ultrasound data
        std::vector<UltrasoundSweep *> sweeps;
        read_ultrasound_data(sweep_path, sweep_files, sweeps);

        // read the sweep label files into vector of ultrasound data
        std::vector<UltrasoundSweep *> labels;
        read_ultrasound_data(label_path, label_files, labels);

        // read the transformations into a vector of eigen matrices
        std::vector<mat4> transformations;
        read_inversed_transformations(transformations_path, transformation_files, transformations);

        // apply the transformations in a for loop and finish stiching
        SharedImageSet *usSweepFinal = new SharedImageSet();
        mat4 transformation = mat4::Identity();
        mat4 fine_tuning_transformation = mat4::Identity();
        mat4 fine_tuning_2 = mat4::Zero();
        mat4 pos_transformed;
        vec4 temp_vec;
        for (unsigned int i = 0; i < sweeps.size(); i++)
        {
          if (i != 0)
          {
            mat4 robot_prev_pos;
            mat4 robot_curr_pos;
            transformation = transformations[i - 1];
            robot_prev_pos << usSweepFinal->matrix(usSweepFinal->size() - 1);
            robot_curr_pos << transformation * sweeps[i]->matrix(0);
            fine_tuning_transformation = robot_curr_pos.inverse() * robot_prev_pos;

            if (ui_->rbtn_both_fine_tunings->isChecked())
            {
              // which means the ultrasound segmentation needs to be done.
              pos_transformed = robot_curr_pos * fine_tuning_transformation;

              auto curr = 0;
              auto next = i;

              // create opencv mat from the last slice of the curr sweep
              MemImage *currSweepEnd = labels[curr]->get(labels[curr]->size() - 1)->mem();
              MemImage *curr_unet = sweeps[curr]->get(sweeps[curr]->size() - 1)->mem();
              int nImageHeight = currSweepEnd->height();
              int nImageWidth = currSweepEnd->width();
              cv::Mat cvImgCurr(nImageHeight, nImageWidth, CV_8UC1);       //uchar
              cv::Mat cvImgCurrByUnet(nImageHeight, nImageWidth, CV_8UC1); //uchar
              memcpy(cvImgCurr.data, currSweepEnd->data(), currSweepEnd->byteSize());
              memcpy(cvImgCurrByUnet.data, curr_unet->data(), currSweepEnd->byteSize());
              algorithm_->unet_segmentation(cvImgCurrByUnet);

              // create opencv mat from the first slice of the next sweep
              MemImage *nextSweepEnd = labels[next]->get(0)->mem();
              MemImage *next_unet = sweeps[next]->get(0)->mem();
              nImageHeight = nextSweepEnd->height();
              nImageWidth = nextSweepEnd->width();
              cv::Mat cvImgNext(nImageHeight, nImageWidth, CV_8UC1);       //uchar
              cv::Mat cvImgNextByUnet(nImageHeight, nImageWidth, CV_8UC1); //uchar
              memcpy(cvImgNext.data, nextSweepEnd->data(), nextSweepEnd->byteSize());
              memcpy(cvImgNextByUnet.data, next_unet->data(), nextSweepEnd->byteSize());
              algorithm_->unet_segmentation(cvImgNextByUnet);

              while (algorithm_->m_segmented_images.size() < 2)
              {
                LOG_INFO("Segmentation loading!");
              }
              if (algorithm_->m_segmented_images.size() == 2)
              {
                LOG_INFO("Both images are segmented!");
                cvImgCurrByUnet = algorithm_->m_segmented_images[0];
                cvImgNextByUnet = algorithm_->m_segmented_images[1];

                cv::resize(algorithm_->m_segmented_images[0], algorithm_->m_segmented_images[0], cv::Size(375, 550));
                cv::resize(algorithm_->m_segmented_images[1], algorithm_->m_segmented_images[1], cv::Size(375, 550));
                cv::imwrite(sweep_dir + "curr.jpg", algorithm_->m_segmented_images[0]);
                cv::imwrite(sweep_dir + "next.jpg", algorithm_->m_segmented_images[1]);
                algorithm_->m_segmented_images.clear();
              }

              cv::threshold(cvImgCurr, cvImgCurr, 0, 255, 0);
              cv::threshold(cvImgNext, cvImgNext, 0, 255, 0);

              cv::threshold(cvImgCurrByUnet, cvImgCurrByUnet, 0, 255, 0);
              cv::threshold(cvImgNextByUnet, cvImgNextByUnet, 0, 255, 0);

              // find the contour in the next slice and the centroid

              //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
              std::vector<std::vector<cv::Point>> contours_curr;
              cv::Mat contourOutputCurr = cvImgCurr.clone();
              cv::findContours(contourOutputCurr, contours_curr, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

              cv::Point start_point(int((nImageWidth / 2.0f) * m_pixel_width), 0); // the pixel position of the point robot touches

              cv::Moments m = moments(contours_curr[0], true);
              cv::Point centroid_curr(m.m10 / m.m00, m.m01 / m.m00);

              //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
              std::vector<std::vector<cv::Point>> contours_next;
              cv::Mat contourOutputNext = cvImgNext.clone();
              cv::findContours(contourOutputNext, contours_next, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
              m = moments(contours_next[0], true);
              cv::Point centroid_next(m.m10 / m.m00, m.m01 / m.m00);

              // find moments in unet segmented image and find the segmentation which is the closest.

              std::vector<std::vector<cv::Point>> contours_curr_unet;
              std::vector<cv::Point> centroids_curr;
              cv::findContours(cvImgCurrByUnet, contours_curr_unet, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
              centroid_curr = algorithm_->find_closest_centroid(contours_curr_unet, centroid_curr);

              std::vector<std::vector<cv::Point>> contours_next_unet;
              std::vector<cv::Point> centroids_next;
              cv::findContours(cvImgNextByUnet, contours_next_unet, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
              centroid_next = algorithm_->find_closest_centroid(contours_next_unet, centroid_next);

              cv::Point diff((centroid_curr.x - centroid_next.x) * m_pixel_width, (centroid_curr.y - centroid_next.y) * m_pixel_height);

              temp_vec << diff.x, diff.y, 0, 1;
              fine_tuning_2.block<3, 1>(0, 3) = pos_transformed.block<3, 3>(0, 0).inverse() * temp_vec.block<3, 1>(0, 0);
            }
          }

          for (int sweep_slice = 0; sweep_slice < sweeps[i]->size(); sweep_slice++)
          {
            // both fine tunings
            if (ui_->rbtn_both_fine_tunings->isChecked())
            {
              if (ui_->checkBox_include_whole_sweep->isChecked())
              {
                usSweepFinal->add(sweeps[i]->get(sweep_slice), (transformation * sweeps[i]->matrix(sweep_slice) * fine_tuning_transformation) + fine_tuning_2);
              }
              else
              {
                usSweepFinal->add(labels[i]->get(sweep_slice), (transformation * sweeps[i]->matrix(sweep_slice) * fine_tuning_transformation) + fine_tuning_2);
              }
            }
            // without fine tuning
            else if (ui_->rbtn_no_fine_tuning->isChecked())
            {
              if (ui_->checkBox_include_whole_sweep->isChecked())
              {
                usSweepFinal->add(sweeps[i]->get(sweep_slice), (transformation * sweeps[i]->matrix(sweep_slice)));
              }
              else
              {
                usSweepFinal->add(labels[i]->get(sweep_slice), (transformation * sweeps[i]->matrix(sweep_slice)));
              }
            }
            // only with the first fine tuning
            else if (ui_->rbtn_fine_tuning_1->isChecked())
            {
              if (ui_->checkBox_include_whole_sweep->isChecked())
              {
                usSweepFinal->add(sweeps[i]->get(sweep_slice), (transformation * sweeps[i]->matrix(sweep_slice) * fine_tuning_transformation));
              }
              else
              {
                usSweepFinal->add(labels[i]->get(sweep_slice), (transformation * sweeps[i]->matrix(sweep_slice) * fine_tuning_transformation));
              }
            }
          }
        }

        // write the final result into a file

        UltrasoundSweep *usSweepFinalUncompensated = static_cast<UltrasoundSweep *>(usSweepFinal);
        auto str = getDayAndTime();
        BackgroundExporter *sweepExporter = new BackgroundExporter();
        std::cout << sweep_dir + "final_" + str << std::endl;
        sweepExporter->save(usSweepFinalUncompensated, sweep_dir + "final_" + str + ".imf");
      }
      else
      {
        LOG_ERROR("The number of sweep files needs to be one larger than the number of transformation files!");
      }
    }

    void PluginController::onClickedCreateSweepFromLabel()
    {
      std::string sweep_dir = ui_->ledtMainDir->text().toStdString();
      add_slash(sweep_dir);

      auto sweep_path = sweep_dir + "sweeps";
      auto label_path = sweep_dir + "labels";
      std::vector<std::string> sweep_files;
      std::vector<std::string> label_files;
      char *path_sweep;
      char *path_label;
      path_sweep = &sweep_path[0];
      path_label = &label_path[0];

      // read the names of the sweep files
      // add them into a vector
      read_and_sort_file_names_in_direactory(path_sweep, sweep_files, ".imf");

      // read the names of the sweep label files
      // add them into a vector
      read_and_sort_file_names_in_direactory(path_label, label_files, ".imf");

      if (label_files.size() == sweep_files.size())
      {
        std::vector<UltrasoundSweep *> sweeps;
        read_ultrasound_data(sweep_path, sweep_files, sweeps);

        // read the sweep label files into vector of ultrasound data
        std::vector<UltrasoundSweep *> labels;
        read_ultrasound_data(label_path, label_files, labels);

        for (size_t idx = 0; idx < sweeps.size(); idx++)
        {
          SharedImageSet *usSweepFinal = new SharedImageSet();

          for (size_t slice_idx = 0; slice_idx < sweeps[idx]->size(); slice_idx++)
          {
            usSweepFinal->add(labels[idx]->get(slice_idx), sweeps[idx]->matrix(slice_idx));
          }
          UltrasoundSweep *usSweepFinalUncompensated = static_cast<UltrasoundSweep *>(usSweepFinal);
          auto str = std::to_string(idx);
          BackgroundExporter *sweepExporter = new BackgroundExporter();
          sweepExporter->save(usSweepFinalUncompensated, sweep_dir + "label_sweep_" + str + ".imf");
        }
      }
      else
      {
        LOG_ERROR("The number of sweep files needs to be eqaul to the number of label files!");
      }
    }

    void PluginController::read_inversed_transformations(const std::string &path, const std::vector<std::string> &file_names, std::vector<mat4> &inversed_transformations)
    {

      auto dir_path = path;
      add_slash(dir_path);

      mat4 inversed_mult;
      for (const auto &file_name : file_names)
      {
        std::string s;
        mat4 transformation = mat4::Identity();
        std::ifstream transformation_file(dir_path + file_name);
        auto line_count = 0;
        while (getline(transformation_file, s))
        {
          int pos_x = int(line_count / 4);
          int pos_y = line_count % 4;
          transformation(pos_x, pos_y) = std::stod(s);
          line_count += 1;
        }
        transformation_file.close();

        // multiply by 1000, cause the transformation data is in m, while
        // the robot tracking is in mm.
        transformation(0, 3) *= 1000;
        transformation(1, 3) *= 1000;
        transformation(2, 3) *= 1000;

        if (inversed_transformations.empty())
        {
          inversed_mult << transformation.inverse();
          inversed_transformations.push_back(inversed_mult);
        }
        else
        {
          inversed_mult = transformation.inverse() * inversed_mult;
          inversed_transformations.push_back(inversed_mult);
        }
        LOG_INFO("Transformation file read successfully!");
      }
    }

    DataList PluginController::readImage_IMF(const std::string &szFilePath)
    {
      ImFusionFile ImageFileReader(szFilePath);
      DataList USImageData;
      if (ImageFileReader.load(USImageData))
      {
        LOG_INFO("Sweep read successfully!");
      }
      else
      {
        LOG_INFO("Failed to read sweep!");
      }
      return USImageData;
    }

    DataList PluginController::readImage_IMF(const std::string &szFilePath, const std::string &szFileName)
    {
      std::string szFile;
      if (szFilePath[szFilePath.size() - 1] == '/')
        szFile = szFilePath + szFileName;
      else
        szFile = szFilePath + "/" + szFileName;
      return readImage_IMF(szFile);
    }

    void PluginController::onTestRotationButtonClicked()
    {
      Eigen::Quaterniond quaPose(-0.207318, -0.0968194, 0.967424, -0.0975086);  //w x, y z
        Eigen::Vector3d vecPosition(0.709147, -0.00825548, 0.0108848);
        LOG_INFO("position:");
        LOG_INFO(vecPosition);
        // algorithm_->executeCartesianCommand(quaPose, vecPosition, false);
      auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);
      LOG_INFO(robot_pose);
      Eigen::Quaterniond q_temp(robot_pose.block<3, 3>(0, 0));
      std::cout<<q_temp.x()<<" "<< q_temp.y()<<" "<< q_temp.z()<<" "<< q_temp.w()<<std::endl;
      // // LOG_INFO(q_temp);
      LOG_INFO(q_temp.w());
 
      algorithm_->executeCartesianCommand(quaPose, vecPosition, false);


    // LOG_
    //     Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);                        //mm
    //     Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //rad

    //     if (ui_->chbUpdateState->isChecked())
    //     {
    //       ui_->ledtPosX->setText(QString::number(translation.x()));
    //       ui_->ledtPosY->setText(QString::number(translation.y()));
    //       ui_->ledtPosZ->setText(QString::number(translation.z()));
    //       ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
    //       ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
    //       ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));
    //     }
    //   if (algorithm_->isRobotConnected())
    //   {
    //     // if the robot is connected, the tcp will be turn by 10 degrees around the X axis.
    //     algorithm_->calculateEndPointsTCP(10, ROTATION_X);
    //     algorithm_->onGotoPose(algorithm_->m_finalPose, false);
    //   }
    }

    // void PluginController::add_slash(std::string &file_dir)
    // {
    //   if (file_dir[file_dir.size() - 1] != '/')
    //   {
    //     file_dir += '/';
    //   }
    // }



  } // namespace OrienAdj
} // namespace ImFusion
