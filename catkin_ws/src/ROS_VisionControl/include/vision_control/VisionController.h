#pragma once
/************************************************************************
 *                           ImFusion
 * **********************************************************************/
#include <ImFusion/Base/Algorithm.h>  // for IoAlgorithm
#include <ImFusion/Base/DataList.h>   // for DataList
#include <ImFusion/Base/AlgorithmListener.h>  // for AlgorithmListener
#include <ImFusion/Base/Log.h>                // for LOG_INFO
#include <ImFusion/Base/Properties.h>         // for Properties
#include <ImFusion/Base/MemImage.h>
#include <ImFusion/Base/SharedImage.h>
#include <ImFusion/Base/SharedImageSet.h>
#include <ImFusion/Base/Progress.h>
#include <ImFusion/Base/OpenCV.h>
#include <ImFusion/Base/ImFusionFile.h>
#include <ImFusion/Base/TypedImage.h>
#include <ImFusion/Base/TrackingStream.h>
#include <ImFusion/GL/GlVolumeCompounding.h>
#include <ImFusion/GL/GlSliceView.h>
#include <ImFusion/GUI/AnnotationModel.h>
#include <ImFusion/GUI/DataWidget.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>
#include <ImFusion/GUI/ImageView2D.h>
#include <ImFusion/GUI/ImageView3D.h>
#include <ImFusion/GUI/Interactive.h>
#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/GUI/AlgorithmController.h>
#include <ImFusion/IO/BackgroundExporter.h>
#include <ImFusion/Stream/OpenIGTLinkStreamData.h>
#include <ImFusion/Stream/OpenIGTLinkImageStream.h>
#include <ImFusion/Stream/LiveTrackingStream.h>
#include <ImFusion/Stream/TrackingStreamData.h>
#include <ImFusion/US/UltrasoundSweep.h>
#include <ImFusion/US/USConfidenceMapAlgorithm.h>
#include <ImFusion/US/FanMask.h>
#include <ImFusion/US/UltrasoundGeometry.h>
#include <ImFusion/US/GlSweepCompounding.h>
#include <ImFusion/US/USSweepRecorderAlgorithm.h>
#include <ImFusion/US/USSweepCalibrator.h>
#include "vision_control/VisionAlgorithm.h"
/************************************************************************
 *                           Qt
 * **********************************************************************/
#include <QtWidgets/QWidget>
#include <QtDebug>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <memory>
#include <QtCore/QObject>
#include <qt5/QtNetwork/QTcpServer>  //for TCP
#include <qt5/QtNetwork/QTcpSocket>  //for TCP
#include <QTimer>
#include <QTime>
#include <unistd.h>
#include <functional>

/************************************************************************
 *                           Custom
 * **********************************************************************/
#include "MonitorChangeThread.h"
#include <vision_control/ConfidentMapConvex.h>

//#include <ImFusion/IO/PngIO.h>
#define TIM_INTERVAL_REC 25
#define TIM_INTERVAL_UPDATE 100


class Ui_Controller;

namespace ImFusion {
namespace vision_control {

class PluginAlgorithm;
//class ConfidentMapConvex;  //what happen if added this line?

class PluginController : public QWidget, public AlgorithmController {
  Q_OBJECT
public:
  /// Constructor with the algorithm instance
  PluginController(PluginAlgorithm* algorithm);
  virtual ~PluginController() = default;

  void init();//imfusion框架自动调用

  QTcpServer *tcpServer;
  QTcpSocket *serverSocket;
  void onRecForce();
  std::vector<float> m_forceData;
  QTimer *m_recordTimer;
  QTimer *m_updateTimer;

public slots:
  void onStartUSStreamClicked();//
  void onStopUSStreamClicked();

  void onVisualizeConfiMapClicked();

  /**
   * @brief onLoadImfSweepClicked a function which loads the given imf file path to imfusion
   */
  void onLoadImfSweepClicked();

  /**
   * @brief onComputeConfiMapClicked given the slice id in imfusion ui by the user, the confidence map of that particular sweep is
   * calculated.
   */
  void onComputeConfiMapClicked();

  void onConnectToRobotClick();
  void onDisconnectFromRobotClick();
  void onExecuteMovementClick();


  /**
   * Functions used for testing
   * */

  /**
   * @brief onSaveHome saves the position of the robot tcp as home, later this
   * can be used to move from home to out and vice versa.
   */
  void onSaveHome();

  /**
   * @brief onSaveOut saves the position of the robot tcp as out, later this
   * can be used to move from out to home and vice versa.
   */
  void onSaveOut();

  /**
   * @brief onGoHome gives a signal to robot, to move the tcp home.
   */
  void onGoHome();

  /**
   * @brief onGoOut gives a signal to robot, to move the tcp out.
   */
  void onGoOut();

  /**
   * @brief onTestButtonClicked this function is to test
   */
  void onTestRotationButtonClicked();


  /**
   * Functions which is used to change the mode of the robot.
   * */
  void onForceModeClick();
  void onPositionModeClick();

  void acceptConnection();
  void replyToClient();

  void onSynRecord();
  void onchbRecordStateChanged();
  void onUpdateState();
  void onChbUpdateStateChanged();

  /**
   * @brief onReadPosesClick a functions which reads the poses from a given path.
   * User can give a path using the line edit object ledt_poses_file_path, in the ui.
   */
  void onReadPosesClick();
  void onExecScanClick();
  void orientOptimize();

  /**
   * @brief onContinueScanClicked waits for the transformation, created from the movement, message to arrive,
   * then updates the scan trajectory and gives the robot signal to continue scanning
   */
  void onContinueScanClicked();

  /**
   * @brief onDownsampleUSSweepClicked after the user defined the full path to the sweep and the downampling index k,
   * the function saves only the every kth element to the sweep.
   */
  void onDownsampleUSSweepClicked();

  /**
   * @brief onClickedCreateSweepFromLabel creates sweeps of labels, using the tracking information taken from the sweep recording
   */
  void onClickedCreateSweepFromLabel();

  /**
   * @brief onClickedStitching Once the user decided which fine tuning they wanted and whether they want the result consist of only
   * the labels or the whole sweep, the stitching is done in this function. The user also needs to give directory path
   * which has labels, sweeps and transformation directories in it.
   */
  void onClickedStitching();
  /**
   * @brief onClickedWriteRobotPose once the button is clicked pose of the robot at a time will be written to the
   * txt file. User can enter the full path to the txt file using ledt_trajectory_file_to_write.
   */
  void onClickedWriteRobotPose();
  /**
   * @brief onGoInitPosClick if there are points in the trajectory, then this function moves the
   * robot to the first station of the scan, however the height of the robot is set 5cm higher than
   * the height of the first position, to prevent any unexpected situation.
   */
  void onGoInitPosClick();


  //JZL
  void onClickedpbtnInitROS();


  //yuangao
  void onToggleStopRobot();
  void onToggleFeedBack();
  void onMoveFeedBack();
  void onWriteImf();

signals:
  // Emit connection signals so other plugins can check the robot status.
  void robotConnected();
  void robotDisconnected();

  void finishedRecording(bool);
        ///< The algorithm instance

private:
  int m_breakPointIndex; // index of the point in the trajectory where the movement occured

  MonitorChangeThread* m_monitorChangeThread;
  QMutex *m_mutex;

  Eigen::Matrix4f m_irTransform;

  /****************************************
   ** Ultrasound Sweep
   ****************************************/
  USSweepRecorderAlgorithm* m_multiUSSweepRecorder;

  UltrasoundSweep* m_usSweep;

  UltrasoundGeometry m_ultrasoundGeometryDefinition;

  std::string getDayAndTime();

  DataList m_dataList;//DataList used to store raw image set including both before movement and after movement


  void updateUIToLastPose();
  void updateUIToLastWrench();
  void recordRobotStatus();

  void add_image_to_imfusion(cv::Mat cvMatConfiMapUChar);
  void read_and_sort_file_names_in_direactory(const char* path, std::vector<std::string> & file_names, const std::string &file_ending);
  void read_ultrasound_data(const std::string &path, const std::vector<std::string> &file_names, std::vector<UltrasoundSweep*> & ultrasound_sweeps);
  void read_ultrasound_data(const std::string &path, std::vector<UltrasoundSweep *> &ultrasound_sweeps);

  /**
   * @brief read_inversed_transformations There may be multiple movements happened during on US examination. To match
   * every single sweep recording with the first recorded sweep, the inverse of the transformations need to be calculated.
   * the files needs to be sorted
   * @param path the full path to the directory of transoformation files
   * @param file_names file names
   * @param inversed_transformations the vector of inversed transformations.
   */
  void read_inversed_transformations(const std::string &path, const std::vector<std::string> &file_names, std::vector<mat4> &inversed_transformations);

  /**
   * @brief readImage_IMF reads the imf file created by Imfusion
   * @param szFilePath the full path of directory of the file.
   * @param szFileName the file name
   * @return the US image data
   */
  DataList readImage_IMF(const std::string & szFilePath, const std::string & szFileName);

  /**
   * @brief readImage_IMF reads the imf file created by Imfusion
   * @param szFilePath the full path to the file
   * @return the US image data
   */
  DataList readImage_IMF(const std::string & szFilePath);
  /**
   * @brief start_recording a function which creates a US stream from cephasonics and robot tracking and
   * starts recording.
   */
  void start_recording();

  /**
   * @brief add_slash adds / at the end of the directory if file path does not have it
   * @param file_dir
   */
  void add_slash(std::string & file_dir);

  /**
   * @brief write_sweep_into_file write the US sweeps to an imf file represented by
   * @param final_sweep
   * @param dir_path
   */
  void write_sweep_into_file(SharedImageSet* final_sweep, std::string dir_path);

  std::shared_ptr<Ui_Controller> ui_{nullptr};  ///< The actual GUI
  PluginAlgorithm* algorithm_{nullptr};
  ConfidentMapConvex* m_confiMapConvex;
  float m_pixel_height = 0.076f; // in mm
  float m_pixel_width = 0.26f; // in mm


  QTimer* timerForMove;
};

}  // namespace vision_control
}  // namespace ImFusion
