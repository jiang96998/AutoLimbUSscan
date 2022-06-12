#include "vision_control/MonitorChangeThread.h"
#include <unistd.h>
#include <opencv2/core.hpp>
#include <ImFusion/Base/Log.h>


MonitorChangeThread::MonitorChangeThread()
{
    //TODO: Mutex
    //Initialization
    m_monitor_read_count = 0;
    //m_mutex=NULL;
}

//get distance(3D) between two optical markers
double MonitorChangeThread::getMarkerDistance(Eigen::Vector3d point1, Eigen::Vector3d point2) {
    double distance;
    distance = pow((point1(0) - point2(0)),2) + pow((point1(1) - point2(1)),2) + pow((point1(2) - point2(2)),2);
    return sqrt(distance);
}

void MonitorChangeThread::run()
{
    while(1)
    {
        LOG_INFO("Monitor thread working...");

        //get the pathname of the current working directory
        char current_path_buf[ 1024 ];
        getcwd(current_path_buf, 1024);
        std::string marker_yaml_path="";
        marker_yaml_path = current_path_buf;

        //decide Zhongliang or camp-robot(Hanyu)
        if(marker_yaml_path == "/home/zhongliang")
        {
          marker_yaml_path = "/home/zhongliang/ros/imfusion_ws/src/ROS_VisionControl/docs/ir_points.yaml";
        }
        else if(marker_yaml_path == "/home/camp-robot")
        {
          //marker_yaml_path = "/home/camp-robot/ros/workspaces/robot_calibration_ws/src/ROS_VisionControl/docs/ir_points.yaml";
          marker_yaml_path = "/home/camp-robot/ir_points.yaml";
        }

        //open yaml file to load the ir marker positions
        cv::FileStorage fs_read(marker_yaml_path, cv::FileStorage::READ);
        //fas.open(marker_yaml_path, cv::FileStorage::READ);
        if(!fs_read.isOpened()){
          LOG_ERROR("Failed to load ir point yaml file ! ");
        }

        //read five ir marker positions
        Eigen::Vector3d fs_start_marker, fs_mid1_marker, fs_mid2_marker, fs_mid3_marker, fs_end_marker;
        cv::FileNode fn_read_start_pos = fs_read["start_ir_point"];
        cv::FileNodeIterator iter_start_marker = fn_read_start_pos.begin();
        for (; iter_start_marker!=fn_read_start_pos.end(); ++iter_start_marker)
        {
          fs_start_marker<<static_cast<double>((*iter_start_marker)["x"]), static_cast<double>((*iter_start_marker)["y"]), static_cast<double>((*iter_start_marker)["z"]);
        }

        cv::FileNode fn_read_mid1_pos = fs_read["mid1_ir_oint"];
        cv::FileNodeIterator iter_mid1_marker = fn_read_mid1_pos.begin();
        for (; iter_mid1_marker!=fn_read_mid1_pos.end(); ++iter_mid1_marker)
        {
          fs_mid1_marker<<static_cast<double>((*iter_mid1_marker)["x"]), static_cast<double>((*iter_mid1_marker)["y"]), static_cast<double>((*iter_mid1_marker)["z"]);
        }

        cv::FileNode fn_read_mid2_pos = fs_read["mid2_ir_point"];
        cv::FileNodeIterator iter_mid2_marker = fn_read_mid2_pos.begin();
        for (; iter_mid2_marker!=fn_read_mid2_pos.end(); ++iter_mid2_marker)
        {
          fs_mid2_marker<<static_cast<double>((*iter_mid2_marker)["x"]), static_cast<double>((*iter_mid2_marker)["y"]), static_cast<double>((*iter_mid2_marker)["z"]);
        }

        cv::FileNode fn_read_mid3_pos = fs_read["mid3_ir_point"];
        cv::FileNodeIterator iter_mid3_marker = fn_read_mid3_pos.begin();
        for (; iter_mid3_marker!=fn_read_mid3_pos.end(); ++iter_mid3_marker) {
           fs_mid3_marker<<static_cast<double>((*iter_mid3_marker)["x"]), static_cast<double>((*iter_mid3_marker)["y"]), static_cast<double>((*iter_mid3_marker)["z"]);
        }

        cv::FileNode fn_read_end_pos = fs_read["end_ir_point"];
        cv::FileNodeIterator iter_end_marker = fn_read_end_pos.begin();
        for (; iter_end_marker!=fn_read_end_pos.end(); ++iter_end_marker)
        {
          fs_end_marker<<static_cast<double>((*iter_end_marker)["x"]), static_cast<double>((*iter_end_marker)["y"]), static_cast<double>((*iter_end_marker)["z"]);
        }



        fs_read.release();

        //Initialization
        if(m_monitor_read_count==0) {
            m_pre_start_marker = fs_start_marker;
            m_pre_mid1_marker = fs_mid1_marker;
            m_pre_mid2_marker = fs_mid2_marker;
            m_pre_mid3_marker = fs_mid3_marker;
            m_pre_end_marker = fs_end_marker;


            m_current_start_marker = m_pre_start_marker;
            m_current_mid1_marker = m_pre_mid1_marker;
            m_current_mid2_marker = m_pre_mid2_marker;
            m_current_mid3_marker = m_pre_mid3_marker;
            m_current_end_marker = m_pre_end_marker;
            LOG_WARN("Ir point position initialized!");
            m_monitor_read_count++;
        }

        //After initialization, read ir markers' position in real-time
        m_current_start_marker = fs_start_marker;
        m_current_mid1_marker = fs_mid1_marker;
        m_current_mid2_marker = fs_mid2_marker;
        m_current_mid3_marker = fs_mid3_marker;
        m_current_end_marker = fs_end_marker;

        //detect movement
        double start_move_dist, mid1_move_dist, mid2_move_dist, mid3_move_dist, end_move_dist;
        start_move_dist = getMarkerDistance(m_pre_start_marker, m_current_start_marker);
        mid1_move_dist = getMarkerDistance(m_pre_mid1_marker, m_current_mid1_marker);
        mid2_move_dist = getMarkerDistance(m_pre_mid2_marker, m_current_mid2_marker);
        mid3_move_dist = getMarkerDistance(m_pre_mid3_marker, m_current_mid3_marker);
        end_move_dist = getMarkerDistance(m_pre_end_marker, m_current_end_marker);

        bool is_move = false;

        //consider movement bigger than 1.5cm as a movement
        if(start_move_dist>0.015 || mid1_move_dist>0.015 || mid3_move_dist>0.015 || end_move_dist>0.015){
            LOG_ERROR("Thread detects movement!");
            LOG_WARN("Detected position before movement of start marker: ");
            LOG_WARN(m_pre_start_marker);
            LOG_WARN("Detected position after movement of start marker: ");
            LOG_WARN(m_current_start_marker);
            LOG_WARN("Detected position before movement of middle 1 marker: ");
            LOG_WARN(m_pre_mid1_marker);
            LOG_WARN("Detected position after movement of middle 1 marker: ");
            LOG_WARN(m_current_mid1_marker);
            LOG_WARN("Detected position before movement of middle 3 marker: ");
            LOG_WARN(m_pre_mid3_marker);
            LOG_WARN("Detected position after movement of middle 3 marker: ");
            LOG_WARN(m_current_mid3_marker);
            LOG_WARN("Detected position before movement of end marker: ");
            LOG_WARN(m_pre_end_marker);
            LOG_WARN("Detected position after movement of end marker: ");
            LOG_WARN(m_current_end_marker);
            is_move = true;
            emit resultMove(is_move);

            //Here "m_" variables are only used to detect movement, and not for calculating transformation matrix
            m_pre_start_marker = m_current_start_marker;
            m_pre_mid1_marker = m_current_mid1_marker;
            m_pre_mid2_marker = m_current_mid2_marker;
            m_pre_mid3_marker = m_current_mid3_marker;
            m_pre_end_marker = m_current_end_marker;
        }
        else {
            is_move = false;
            emit resultMove(is_move);
        }
        sleep(1);
    }
}
