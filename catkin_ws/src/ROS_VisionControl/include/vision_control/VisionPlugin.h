#pragma once

#include <ImFusion/Base/ImFusionPlugin.h>

namespace ImFusion {
class AlgorithmFactory;
class AlgorithmControllerFactory;

namespace vision_control {

class Plugin : public ImFusionPlugin {
public:
  Plugin();
  virtual ~Plugin() = default;

  virtual const AlgorithmFactory* getAlgorithmFactory();

  virtual const AlgorithmControllerFactory* getAlgorithmControllerFactory();

private:
  std::unique_ptr<AlgorithmFactory> algorithm_factory_{nullptr};
  std::unique_ptr<AlgorithmControllerFactory> algorithm_controller_factory_{nullptr};
};

}  // namespace OrienAdj
}  // namespace ImFusion
