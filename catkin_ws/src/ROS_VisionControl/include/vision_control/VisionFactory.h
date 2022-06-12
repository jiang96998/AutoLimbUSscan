#pragma once

#include <ImFusion/Base/AlgorithmControllerFactory.h>
#include <ImFusion/Base/AlgorithmFactory.h>

namespace ImFusion {
namespace vision_control {

class Algorithm;

class PluginAlgorithmFactory : public AlgorithmFactory {
public:
  PluginAlgorithmFactory();
};

class PluginControllerFactory : public AlgorithmControllerFactory {
public:
  virtual AlgorithmController* create(ImFusion::Algorithm* a) const override;
};

}  // namespace OrienAdj
}  // namespace ImFusion
