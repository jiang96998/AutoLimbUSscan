#include <vision_control/VisionStreamOptimizer.h>


namespace ImFusion {
namespace vision_control {

void StreamOptimizer::onStreamData(const StreamData &streamData) {
  if (streamData.stream()->kind() == ImFusion::Data::Kind::IMAGESTREAM){
      const ImageStreamData* imageData = dynamic_cast<const ImageStreamData*> (&streamData);
      if (!imageData || !imageData->images().size()) return;
      m_memImage = imageData->images().at(0)->clone();
  }
}

void StreamOptimizer::onStreamStopped(const Stream * stream){
  std::cout << "hey1" << std::endl;
  cv::destroyAllWindows();
}
void StreamOptimizer::onStreamStarted(const Stream *stream){
  std::cout << "hey2" << std::endl;
}
void StreamOptimizer::onStreamEnded(const Stream *stream){
  std::cout << "hey3" << std::endl;
}
void StreamOptimizer::onChangeThread(const Stream *stream){
  std::cout << "hey4" << std::endl;
}


}
}
