#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp vision common utilities
constructor_args:
  - cfg: ''
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"

class SpVisionCommon : public LibXR::Application {
public:
  SpVisionCommon(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  }

  void OnMonitor() override {}

private:
};
