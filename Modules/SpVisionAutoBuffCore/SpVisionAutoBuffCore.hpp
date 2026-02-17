#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp vision autobuff core
constructor_args:
  - cfg: ''
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"

class SpVisionAutoBuffCore : public LibXR::Application {
public:
  SpVisionAutoBuffCore(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  }

  void OnMonitor() override {}

private:
};
