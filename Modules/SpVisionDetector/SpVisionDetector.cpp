#include "SpVisionDetector.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

#include <opencv2/highgui.hpp>

#include "logger.hpp"

std::string SpVisionDetector::WriteYoloConfig(const Config & cfg, const void * tag)
{
  const auto dir = std::filesystem::temp_directory_path() / "sp_vision_xr_cfg";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  std::ostringstream name;
  name << "detector_" << reinterpret_cast<uintptr_t>(tag) << ".yaml";
  const auto path = dir / name.str();

  std::ofstream ofs(path);
  ofs << "yolo_name: \"" << cfg.yolo_name << "\"\n";
  ofs << "yolov5_model_path: \"" << cfg.runtime.model_path << "\"\n";
  ofs << "yolov8_model_path: \"" << cfg.runtime.model_path << "\"\n";
  ofs << "yolo11_model_path: \"" << cfg.runtime.model_path << "\"\n";
  ofs << "device: \"" << cfg.runtime.device << "\"\n";
  ofs << "threshold: " << cfg.runtime.threshold << "\n";
  ofs << "min_confidence: " << cfg.runtime.min_confidence << "\n";
  ofs << "roi:\n";
  ofs << "  x: " << cfg.runtime.roi_x << "\n";
  ofs << "  y: " << cfg.runtime.roi_y << "\n";
  ofs << "  width: " << cfg.runtime.roi_width << "\n";
  ofs << "  height: " << cfg.runtime.roi_height << "\n";
  ofs << "use_roi: " << (cfg.runtime.use_roi ? "true" : "false") << "\n";
  ofs << "use_traditional: " << (cfg.runtime.use_traditional ? "true" : "false") << "\n";

  const auto classify_model =
    cfg.classifier.classify_model.empty() ? cfg.detector.classifier.classify_model
                                          : cfg.classifier.classify_model;
  ofs << "classify_model: \"" << classify_model << "\"\n";
  ofs << "max_angle_error: " << cfg.detector.max_angle_error_deg << "\n";
  ofs << "min_lightbar_ratio: " << cfg.detector.min_lightbar_ratio << "\n";
  ofs << "max_lightbar_ratio: " << cfg.detector.max_lightbar_ratio << "\n";
  ofs << "min_lightbar_length: " << cfg.detector.min_lightbar_length << "\n";
  ofs << "min_armor_ratio: " << cfg.detector.min_armor_ratio << "\n";
  ofs << "max_armor_ratio: " << cfg.detector.max_armor_ratio << "\n";
  ofs << "max_side_ratio: " << cfg.detector.max_side_ratio << "\n";
  ofs << "max_rectangular_error: " << cfg.detector.max_rectangular_error_deg << "\n";

  return path.string();
}

SpVisionDetector::SpVisionDetector(
  LibXR::HardwareContainer &, LibXR::ApplicationManager & app, Config cfg)
: cfg_(std::move(cfg)),
  yolo_config_path_(WriteYoloConfig(cfg_, this)),
  detector_(yolo_config_path_, cfg_.debug)
{
  LibXR::Topic image_topic(
    LibXR::Topic::FindOrCreate<sp_xr::ImageFrame>("image_raw", &vision_domain_));
  auto image_cb = LibXR::Topic::Callback::Create(
    [](bool, SpVisionDetector * self, LibXR::RawData & data) {
      auto * frame = reinterpret_cast<sp_xr::ImageFrame *>(data.addr_);
      self->OnImage(*frame);
    },
    this);
  image_topic.RegisterCallback(image_cb);

  app.Register(*this);
  XR_LOG_PASS("[SpVisionDetector] initialized. yolo=%s", cfg_.yolo_name.c_str());
}

void SpVisionDetector::OnImage(sp_xr::ImageFrame & frame)
{
  sp_xr::ArmorFrame output;
  output.timestamp = frame.timestamp;
  output.seq = frame.seq;
  output.armors = detector_.detect(frame.img, static_cast<int>(frame.seq));

  if (cfg_.debug) {
    cv::waitKey(1);
  }

  armors_topic_.Publish(output);
}
