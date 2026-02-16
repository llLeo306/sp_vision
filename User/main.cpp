#include "libxr.hpp"
#include "ramfs.hpp"
#include "xrobot_main.hpp"

int main(int, char **)
{
  LibXR::PlatformInit();
  XR_LOG_PASS("Platform initialized");

  LibXR::RamFS ramfs;
  LibXR::HardwareContainer peripherals{
    LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
  };
  XRobotMain(peripherals);
  return 0;
}
