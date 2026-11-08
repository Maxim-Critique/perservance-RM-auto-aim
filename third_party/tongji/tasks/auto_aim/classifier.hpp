#include <yaml-cpp/yaml.h>
#ifndef FUSION_VENDOR_AUTO_AIM__CLASSIFIER_HPP
#define FUSION_VENDOR_AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
class Classifier
{
public:
  explicit Classifier(const YAML::Node & config);

  void classify(Armor & armor);

  void ovclassify(Armor & armor);

private:
  cv::dnn::Net net_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;
};

}  // namespace auto_aim

#endif  // FUSION_VENDOR_AUTO_AIM__CLASSIFIER_HPP