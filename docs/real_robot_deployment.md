# Ubuntu22.04 机载部署与回滚

当前候选位于独立工作树的本地 `sentry毅同融合` 分支，按用户最新要求只修改、不提交、不推送；整车构建和真实硬件未验证。不要把本说明等同于已经通过实车验收。
本次没有 SSH、修改自启动或运行真实执行机构。

## 依赖与环境

已测试离线环境：Ubuntu22.04.5、GCC11.4、CMake3.22.1、OpenCV4.5.4、Eigen3.4、Ceres2.0、OpenVINO2024.6.0、yaml-cpp、fmt、spdlog、Threads。
整车还需要原车 Hikrobot MVS SDK、libusb、ROS2 Humble/rclcpp/std_msgs 及原导航工作空间。
当前机器没有 /opt/ros/humble、/opt/MVS；sudo 需要密码，未执行安装，也未索要密码。

在已配置相应官方软件源的机载机上，由管理员安装缺失开发包：

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config libopencv-dev libeigen3-dev \
  libceres-dev libyaml-cpp-dev libfmt-dev libspdlog-dev libusb-1.0-0-dev \
  ros-humble-rclcpp ros-humble-std-msgs
```

使用原车已部署的 MVS SDK 及其 udev 规则，不用 C 的相机 SDK 替换。确认：

```bash
test -f /opt/MVS/include/MvCameraControl.h
ls /opt/MVS/lib /opt/MVS/lib/64
source /opt/ros/humble/setup.bash
ros2 pkg prefix rclcpp
```

OpenVINO 使用已验证版本或原车经验证的兼容运行时；本次验证的配置路径为 `/opt/intel/openvino_2024.6.0/runtime/cmake`。
原车相机驱动仍使用自动增益 `GainAuto=CONTINUOUS`，不是新算法强制增益0；曝光 armor/outpost=2500μs、buff=2000μs。
代码没有主动设置固定分辨率，使用 SDK 帧尺寸，必须在设备上核对，不能宣称硬件分辨率已验证。

## 模型与配置

模型在 `third_party/tongji/assets/`：yolo11.xml/.bin、yolov5.xml/.bin、yolov8.xml/.bin、tiny_resnet.onnx，来自固定 C 提交。
相对路径以 `configs/tongji_sentry.yaml` 所在目录解析；原车 model/ 下 Legacy 模型保留。
原车 main 仍使用 `Base.h` 的 `/home/perseverance/auto-aim/text.yaml`，不要忽略这条绝对路径。
若安装目录不同，先在人工审核下决定配置路径安排，不要让程序读取另一份旧标定。

## 编译和纯离线启动

在源码根目录：

```bash
cmake -S . -B build-offline -DFUSION_OFFLINE_ONLY=ON \
  -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build-offline -j2
ctest --test-dir build-offline --output-on-failure
build-offline/fusion/fusion_offline text.yaml configs/tongji_sentry.yaml \
  /path/to/video.avi 200 /tmp/detection.csv
```

普通录像默认 BGR，仅检测，不凭空补零姿态。原车录像带同名 txt 时可增加该元数据路径；此时默认恢复原 Recorder 的 RGB 数组语义。
最后可显式增加 `0` 或 `1` 覆盖 input_rgb。没有姿态的录像不会输出伪造的世界坐标或 Tracker 成功状态。
`fusion_serial_mock` 只生成 `/dev/pts/` 伪终端，不能传真实串口设备名。

整车编译：

```bash
source /opt/ros/humble/setup.bash
export PKG_CONFIG_PATH=/opt/intel/openvino_2024.6.0/runtime/lib/intel64/pkgconfig:${PKG_CONFIG_PATH:-}
cmake -S . -B build -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
cmake --build build -j2
ldd build/action
```

当前 WSL 的整车配置在 rclcpp 处失败，没有声称 action/review/aim_preview 链接成功。

## 设备、标定与 ROS2 核对

```bash
lsusb
ls -l /dev/serial/by-id/ /dev/ttyACM* /dev/video*
ros2 topic list -t
ros2 topic info /target_number
ros2 topic info /target_pos
```

原相机 VID:PID=2bdf:0001，原串口选择仍为 /dev/ttyACM0、115200/8N1；是否确为所述 MV-CS016-10UC 需 SDK 枚举确认。
ROS2 发布 target_number/std_msgs::msg::Int16，订阅 target_pos/Int16MultiArray，均保留。导航期望至少2个 int16 元素。
不要在自动控制已接通时随意做导航 topic pub。

按 armor_type_calibration.md 核验图像颜色、实际分辨率、K/D、相机到云台 R/t、姿态符号及同步。
接收驱动仍保留 A 的 yaw 额外×10、pitch 缩放、roll 减π；这些是现有协议语义，不可凭名称改掉。
原 Query 使用两侧角度简单平均，跨±π和串口短包/粘包仍需专项实机验证，当前不宣称可靠性已修复。

## 分阶段启动、停止与日志

1. 纯录像：运行 fusion_offline，不运行 action，物理执行机构保持隔离。
2. 仅相机：人工确保安全隔离后使用原 `aim_preview`，它使用零姿态作预览，不能用于世界坐标精度验收。该程序没有下位机串口对象，但本次未执行。
3. 串口只读整机流：当前 action 没有经过验证的只读/禁用输出开关，**不能把 action 当作该阶段测试工具**；需先由人工使用独立接收验证入口，或补齐输出隔离后再验收。
4. 协议 Mock：运行 build-offline/fusion/fusion_serial_mock，不连接真实 /dev/ttyACM0。
5. 人工监护、发射机构禁用、执行机构安全隔离后检查云台方向、限角、失联与急停；完成全部验收后才考虑原车整机启动。

原启动入口保留为：

```bash
cd /home/perseverance/auto-aim/build
./action
```

这条命令会进入原串口发送流程，应只在相应人工授权和隔离检查完成后使用。本次未执行它，也未写自启动配置。
正常停止使用 Ctrl+C，后台进程使用 `kill -INT <确认的PID>`。原 Exiter 只安装 SIGINT，采集/串口阻塞下的停止时延仍须现场验证，不能依赖未验证的软件退出替代硬件急停。
日志是启动工作目录下的 `logs/YYYY-MM-DD_HH-MM-SS.log`，同济算法日志带 `tongji_` 前缀。不应通过重启/自动 watchdog 在故障时反复驱动云台。

## 回滚

部署前由人工备份当前可用的 action、text.yaml 和相关模型/动态库。快速回滚必须成套恢复该可用二进制与配置，并在执行机构隔离后启动。
算法比较可以只将 text.yaml 中 `backend: tongji` 改为 `backend: legacy`，其他标定保持；Legacy 仍需实际构建和现场验证。

保留历史的代码回滚可以在独立目录检出原车版本：

```bash
git worktree add --detach ../original-car-rollback 405b40812b72705b8d5b173a5ef2529ecab84efa
```

恢复本次之前的融合历史可以使用备份 bundle，或独立检出3a52df8；它原本也未实车验证，不等同于可用实车版本。
仅在未来用户另行授权提交/推送且确已执行后，才可用普通 `git revert <实际的新融合提交SHA>` 回退并正常推送；不做 reset --hard、clean 或强制推送。
