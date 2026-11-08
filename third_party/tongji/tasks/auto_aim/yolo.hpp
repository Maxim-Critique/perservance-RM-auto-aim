#include <memory>
#include <yaml-cpp/yaml.h>
#ifndef FUSION_VENDOR_AUTO_AIM__YOLO_HPP
#define FUSION_VENDOR_AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim
{
class YOLOBase
{
public:
  virtual ~YOLOBase() = default;
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const YAML::Node & config, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // FUSION_VENDOR_AUTO_AIM__YOLO_HPP