# 旧版本与键盘测试组合分支

分支：`rollback-3a52df8-with-keyboard`。

基线：`3a52df8d805d5972364984fce242eb8b737537dd`，提交说明为“第二修改哨兵毅同融合还未验证”。

## 组合内容

- 保留旧版本整车主入口、原离线复盘入口和相机预览入口。
- 新增 `Test/src/keyboard_gimbal_test.cpp` 及独立 CMake 目标。
- 方向键按住转动、松开保持反馈角度；R 切换现有 AutoArmor 追踪自瞄。
- 按住空格请求发射，松开清零；追踪模式同时遵守原 Shooter 开火判定。B/F 选择蓝色/红色目标。
- 终端输入两个整数角度，例如 `32 21`，回车后退出追踪并按设定速度转向；空行取消目标。
- 窗口显示当前反馈方向、预测器空间点对应的方向角及终端目标；无数据时显示 `* *`。
- 新增相机、串口非阻塞读取接口及测试用串口超时选项；AutoArmor 可输出本帧检测结果和预测点，避免重复检测。
- 测试程序没有俯仰角范围限制，仅检查串口数值是否可编码；手动 yaw 仍保留启动位置正负 10 度范围。转动速度控制、数据过期处理和空格释放处理保留。

## 实机首次下载（推荐独立目录）

不要覆盖实机原工程及其配置。首次下载到一个尚不存在的新目录：

```bash
git clone --single-branch --branch rollback-3a52df8-with-keyboard \
  https://github.com/Maxim-Critique/perservance-RM-auto-aim.git \
  "$HOME/rm_projects/autoaim_keyboard_test"
cd "$HOME/rm_projects/autoaim_keyboard_test"
git log -1 --oneline
```

若仓库需要认证，先通过实机已有的 GitHub 登录方式完成认证，不要将令牌写进仓库或分享在聊天中。

后续更新这个独立目录：

```bash
cd "$HOME/rm_projects/autoaim_keyboard_test"
git status --short
git pull --ff-only origin rollback-3a52df8-with-keyboard
```

有本机修改时先备份或提交，再更新；不要使用强制重置覆盖实机配置。

## 编译

需要 CMake 3.16+、C++17、OpenCV（含 calib3d/dnn）、Eigen3、yaml-cpp、spdlog、libusb、Qt Widgets 开发包，以及 MVS SDK。
MVS 默认从 `/opt/MVS/include` 与 `/opt/MVS/lib/64` 或 `/opt/MVS/lib` 查找。
这个独立目标不依赖整车的 ROS/OpenVINO 模块。

Ubuntu 缺少 Qt 开发包时可由现场维护人员安装 `qtbase5-dev`。实机不要使用开发环境 `/tmp/autoaim-qt-dev` 路径。

```bash
cmake -S . -B build/keyboard -DKEYBOARD_GIMBAL_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/keyboard --target keyboard_gimbal_test -j2
```

## 配置与运行

保留并使用实机自己的相机标定、安装外参和补偿参数。不要直接用仓库 `text.yaml` 覆盖实机配置。
参数 1 是 YAML 的实际绝对路径；该 YAML 所在目录需要有 `model/mlp.onnx` 与 `model/label.txt`。
以下示例路径必须替换为实机原配置文件所在位置：

```bash
./build/keyboard/keyboard_gimbal_test \
  /实际的实机配置目录/text.yaml /dev/ttyACM0 0 5 1 1
```

其余参数依次为串口、敌方颜色（0 蓝/1 红）、角速度（度/秒）、yaw 方向（1/-1）、pitch 方向（1/-1）。

窗口按键需要窗口焦点；终端角度命令则在终端回车后执行，即使终端获得焦点也会继续转向。终端转向模式不请求发射，空行可停止，Ctrl+C 可退出。窗口 Q/Esc 也可退出。

发射字段采用装甲板链路的 `fire=1/0`。手动转向沿用 `found=2`，追踪保留 AutoArmor 的目标 ID/found。电控接收端不在本仓库，手动 `found=2, fire=1` 组合及实际停止响应仍需双方确认。

## 验证记录（2026-11-11）

- 已在开发环境成功编译并链接 `keyboard_gimbal_test`，输出 `100% Built target keyboard_gimbal_test`。
- `git diff --check` 通过；动态库依赖检查未发现缺失库。
- CMake 提示 MVS 自带 libusb 可能覆盖系统 libusb；实际兼容性待实机验证。
- 未运行硬件程序，未执行任何 `.sh` 脚本；尚未验证窗口、键盘、识别、实际角度或发射效果。
