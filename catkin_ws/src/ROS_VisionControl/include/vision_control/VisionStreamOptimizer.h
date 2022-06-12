#include <ImFusion/Stream/StreamListener.h>
#include <ImFusion/Stream/StreamData.h>
#include <ImFusion/Stream/Stream.h>
#include <ImFusion/RGBD/RGBDStream.h>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <vision_control/ConfidentMapConvex.h>

namespace ImFusion {
namespace vision_control {

class StreamOptimizer: public StreamListener {
  // StreamListener interface  
public:
//  StreamOptimizer() {
//    cv::namedWindow("example");
//  }
  void onStreamData(const StreamData &streamData);
  void onStreamStopped(const Stream *);
  void onStreamStarted(const Stream *);
  void onStreamEnded(const Stream *);
  void onChangeThread(const Stream *);
  MemImage* getMemImage() { return m_memImage; }
private:
  MemImage* m_memImage = nullptr;
};
}
}
