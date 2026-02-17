# sp_vision_25 中文使用说明（XRobot 模块版）

本仓库已重构为 `XRobot + libxr Topic` 模块化架构：
- 内部模块通信使用 `Topic`
- 下位机通信使用 `SharedTopic / SharedTopicClient`
- 主流程模块：`SpVisionCamera -> SpVisionDetector -> SpVisionTracker -> SpVisionAimer`

## 1. 目录说明（当前版本）
- `Modules/`：所有功能模块（算法、相机、状态、通信）
- `User/xrobot.yaml`：默认运行配置（本机视频调试）
- `User/xrobot_sharetopic.yaml`：下位机联调配置（含 SharedTopic）
- `User/xrobot_main.hpp`：由 `xrobot_gen_main` 生成的入口
- `Modules/SpVisionCommon/assets/demo/demo.avi`：测试视频

## 2. 环境依赖
建议 Ubuntu 22.04。

安装基础依赖：
```bash
sudo apt update
sudo apt install -y \
  git g++ cmake \
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev \
  libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev
```

还需要：
- OpenVINO（建议 2024.6+）
- `xrobot_gen_main` 工具可用

检查工具：
```bash
which xrobot_gen_main
```

## 3. 编译
在仓库根目录执行：
```bash
cmake -S . -B build
cmake --build build -j$(nproc) --target sp_vision_xrobot
```

## 4. 运行（默认视频 + 可视化窗口）
默认配置 `User/xrobot.yaml` 已打开可视化：
- `sp_camera.cfg.preview: true`
- `sp_detector.cfg.debug: true`

直接运行：
```bash
./build/sp_vision_xrobot
```

会看到窗口：
- `sp_vision_image_raw`（相机/视频原图）
- `detection`（检测结果）

## 5. 使用真实相机（替代视频）
编辑 `User/xrobot.yaml` 中 `sp_camera`：
- `use_video: false`
- `camera.camera_name: "hikrobot"` 或 `"mindvision"`
- 按相机实际情况调整 `exposure_ms/gain/gamma/vid_pid`

修改后重新生成入口并编译：
```bash
xrobot_gen_main -c User/xrobot.yaml -o User/xrobot_main.hpp
cmake --build build -j$(nproc) --target sp_vision_xrobot
```

## 6. 下位机联调（SharedTopic）
如果要和下位机通过共享 Topic 互通，使用：
- `User/xrobot_sharetopic.yaml`

生成并运行：
```bash
xrobot_gen_main -c User/xrobot_sharetopic.yaml -o User/xrobot_main.hpp
cmake --build build -j$(nproc) --target sp_vision_xrobot
./build/sp_vision_xrobot
```

说明：
- `SharedTopic` 负责接收下位机 Topic（如 `ahrs_quaternion`）
- `SharedTopicClient` 负责发送视觉输出（如 `aim_share`）
- 在无对应串口设备的普通 PC 上，SharedTopic 模式可能异常，建议仅在目标机/实机环境使用

## 7. 用 ShareTopic 四元数参与解算（重点）
要让下位机四元数真正进入自瞄解算，必须同时满足下面 3 项：
1. `SpVisionState` 开启 topic 四元数输入  
在 `User/xrobot_sharetopic.yaml` 中：
- `sp_state.cfg.use_topic_quaternion: true`
- `sp_state.cfg.quaternion_topic_name: "ahrs_quaternion"`
2. `SharedTopic` 订阅同名 topic  
在 `User/xrobot_sharetopic.yaml` 中：
- `sharedtopic_server.topic_configs` 包含 `"ahrs_quaternion"`
3. 下位机发送端发布同名同类型 topic  
- 名称：`ahrs_quaternion`
- 类型：`Eigen::Quaternionf`（w, x, y, z）

代码链路：
1. `SharedTopic` 从串口解析 topic 数据并发布到本机 topic 系统  
2. `SpVisionState` 订阅 `ahrs_quaternion`，写入 `gimbal_state.q`  
3. `SpVisionTracker` 读取 `gimbal_state.q`，调用 `solver_.set_R_gimbal2world(...)` 完成姿态结算

对应代码：
- `Modules/SpVisionState/SpVisionState.cpp:16`
- `Modules/SpVisionState/SpVisionState.cpp:52`
- `Modules/SpVisionTracker/SpVisionTracker.cpp:54`

调试判断是否收到四元数：
- 首次收到会打印日志：`[SpVisionState] first topic quaternion received ...`
- 如果一直没有这条日志，说明串口或 topic 名称/类型不匹配

## 8. 视频调试流程（推荐）
### 8.1 纯视频离线调试（不依赖下位机）
用 `User/xrobot.yaml`：
- `sp_camera.cfg.use_video: true`
- `sp_camera.cfg.preview: true`
- `sp_detector.cfg.debug: true`
- `sp_state.cfg.mock_mode: true`
- `sp_state.cfg.use_topic_quaternion: false`

执行：
```bash
xrobot_gen_main -c User/xrobot.yaml -o User/xrobot_main.hpp
cmake --build build -j$(nproc) --target sp_vision_xrobot
./build/sp_vision_xrobot
```

### 8.2 视频 + 下位机四元数联调
用 `User/xrobot_sharetopic.yaml`（相机仍可用视频）：
- `sp_camera.cfg.use_video: true`
- `sp_state.cfg.use_topic_quaternion: true`
- 打开 `SharedTopic/SharedTopicClient`

执行：
```bash
xrobot_gen_main -c User/xrobot_sharetopic.yaml -o User/xrobot_main.hpp
cmake --build build -j$(nproc) --target sp_vision_xrobot
./build/sp_vision_xrobot
```

可视化窗口：
- `sp_vision_image_raw`
- `detection`

## 9. Topic 数据流
内部 Topic（视觉域 `sp_vision`）：
1. `image_raw`
2. `armors_result`
3. `gimbal_state`
4. `tracked_targets`
5. `aim_command`

对下位机 Topic：
- 发送：`aim_share`
- 接收：`ahrs_quaternion`

## 10. 常见问题
### 10.1 看不到窗口
先检查：
```bash
echo $DISPLAY
```
为空说明当前终端没有图形会话（例如纯 SSH）。请在本地图形终端运行，或启用 X11 转发。

### 10.2 模型加载失败
确认模型文件存在：
- `Modules/SpVisionCommon/assets/yolov5.xml`
- `Modules/SpVisionCommon/assets/tiny_resnet.onnx`

### 10.3 改了 yaml 不生效
你需要重新生成入口：
```bash
xrobot_gen_main -c User/xrobot.yaml -o User/xrobot_main.hpp
```
然后重新编译。

## 11. 一条命令快速验证
```bash
xrobot_gen_main -c User/xrobot.yaml -o User/xrobot_main.hpp && \
cmake --build build -j$(nproc) --target sp_vision_xrobot && \
./build/sp_vision_xrobot
```

## 12. 日志与曲线图（新增）
运行程序后会自动生成：
- 文本日志：`logs/*.log`
- 自瞄结构化日志：`logs/analysis/aim_trace_*.csv`

CSV 包含关键字段：
- `cmd_yaw/cmd_pitch`
- `gimbal_yaw/gimbal_pitch`
- `yaw_err/pitch_err`
- `control/shoot/aim_valid`
- `target_distance/target_vyaw`

生成曲线图（默认用最新 CSV）：
```bash
python3 tool/plot_aim_log.py
```

指定输入和输出：
```bash
python3 tool/plot_aim_log.py \
  --input logs/analysis/aim_trace_2026-02-16_16-00-00.csv \
  --output logs/analysis/aim_trace_2026-02-16_16-00-00.png
```
