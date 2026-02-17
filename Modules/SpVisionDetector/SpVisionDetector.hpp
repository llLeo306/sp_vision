#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp vision detector
constructor_args:
  - cfg:
      yolo_name: yolov5
      debug: false
      runtime:
        model_path: Modules/SpVisionCommon/assets/yolov5.xml
        device: CPU
        threshold: 150
        min_confidence: 0.8
        roi_x: 420
        roi_y: 50
        roi_width: 600
        roi_height: 600
        use_roi: false
        use_traditional: true
      classifier:
        classify_model: Modules/SpVisionCommon/assets/tiny_resnet.onnx
      detector:
        classifier:
          classify_model: Modules/SpVisionCommon/assets/tiny_resnet.onnx
        threshold: 150
        max_angle_error_deg: 45
        min_lightbar_ratio: 1.5
        max_lightbar_ratio: 20
        min_lightbar_length: 8
        min_armor_ratio: 1
        max_armor_ratio: 5
        max_side_ratio: 1.5
        min_confidence: 0.8
        max_rectangular_error_deg: 25
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <string>

#include "Modules/SpVisionCommon/SpVisionMessages.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "tasks/auto_aim/yolo.hpp"

class SpVisionDetector : public LibXR::Application
{
public:
  struct ClassifierConfig
  {
    std::string classify_model = "Modules/SpVisionCommon/assets/tiny_resnet.onnx";
  };

  struct RuntimeConfig
  {
    std::string model_path = "Modules/SpVisionCommon/assets/yolov5.xml";
    std::string device = "CPU";
    double threshold = 150.0;
    double min_confidence = 0.8;
    int roi_x = 420;
    int roi_y = 50;
    int roi_width = 600;
    int roi_height = 600;
    bool use_roi = false;
    bool use_traditional = true;
  };

  struct DetectorConfig
  {
    ClassifierConfig classifier{};
    double threshold = 150.0;
    double max_angle_error_deg = 45.0;
    double min_lightbar_ratio = 1.5;
    double max_lightbar_ratio = 20.0;
    double min_lightbar_length = 8.0;
    double min_armor_ratio = 1.0;
    double max_armor_ratio = 5.0;
    double max_side_ratio = 1.5;
    double min_confidence = 0.8;
    double max_rectangular_error_deg = 25.0;
  };

  struct Config
  {
    std::string yolo_name = "yolov5";
    bool debug = false;
    RuntimeConfig runtime{};
    ClassifierConfig classifier{};
    DetectorConfig detector{};
  };

  SpVisionDetector(LibXR::HardwareContainer & hw, LibXR::ApplicationManager & app, Config cfg);

  void OnMonitor() override {}

private:
  static std::string WriteYoloConfig(const Config & cfg, const void * tag);
  void OnImage(sp_xr::ImageFrame & frame);

  Config cfg_;
  std::string yolo_config_path_;
  auto_aim::YOLO detector_;

  LibXR::Topic::Domain vision_domain_{"sp_vision"};
  LibXR::Topic armors_topic_{"armors_result", sizeof(sp_xr::ArmorFrame), &vision_domain_};
};
