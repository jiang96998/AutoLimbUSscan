#include "vision_control/VisionFactory.h"
#include "vision_control/VisionAlgorithm.h"
#include "vision_control/VisionController.h"

namespace ImFusion {
namespace vision_control {

PluginAlgorithmFactory::PluginAlgorithmFactory() { registerAlgorithm<PluginAlgorithm>("Markerless RUSS"); }

AlgorithmController* PluginControllerFactory::create(ImFusion::Algorithm* a) const {
  if (PluginAlgorithm* algorithm = dynamic_cast<PluginAlgorithm*>(a)) { return new PluginController(algorithm); }
  return nullptr;
}

}  // namespace OrienAdj
}  // namespace ImFusion
