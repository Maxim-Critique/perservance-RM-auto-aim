# 单相机识别与像素预测预览

入口：`Test/src/camera_detection_preview.cpp`，目标：`camera_detection_preview`。

本程序复用现有 `ArmorDetector`、`tools::ExtendedKalmanFilter` 和海康相机接口；也可通过 OpenCV 打开普通 USB 摄像头。它不构造串口对象、不启动 ROS、不调用发射或云台输出接口。

## 显示内容与范围

- 绿色圆圈/十字：当前选中灯条配对的实测像素中心。
- 紫色十字/箭头：根据图像平面匀速模型，向未来外推默认 150 ms 的像素位置。
- 顶部显示原图像素坐标、预测时长、检测数量、处理耗时及像素速度；窗口缩放不改变数字的原图坐标含义。
- 没有目标时当前点显示 `* *`；短时丢失最多沿用 150 ms 预测，随后清空。预测超出画面时保留数值和指向边缘的箭头，不把越界坐标伪装成画内预测点。
- 多组灯条中，初次选最接近画面中心的一组，之后按预测位置做近邻关联；此入口主要用于手头单块装甲板的定性检查。

不使用实机内参、安装外参、云台姿态、距离或弹速，因此不需要 `text.yaml`。这不是现有整车三维瞄准点投影，也不能用于判断实际射击精度；手持相机运动会与装甲板运动共同体现为像素速度。

当前共享检测器已开启 `ArmorDetector::kLightBarsOnly`：保留原灯条几何条件，跳过数字模型。识别不要求装甲板印有数字。未来将该开关恢复为 `false` 后，本程序会使用源码目录下 `model/mlp.onnx` 和 `model/label.txt`。

## 编译

从项目根目录执行，仅编译该独立预览，不依赖整车 ROS/OpenVINO 或 Qt 开发包：

```bash
cmake -S . -B build/camera -DCAMERA_PREVIEW_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/camera --target camera_detection_preview -j2
```

需要现有的 OpenCV（含 highgui/dnn/videoio）、Eigen、yaml-cpp、spdlog、libusb、MVS SDK。Qt 开发包不是独立依赖，但 OpenCV 本身仍需可用的桌面窗口后端。

## 打开实际相机（由使用者执行）

海康 USB 工业相机，蓝色目标、预测 0.15 秒、曝光 2500 微秒：

```bash
./build/camera/camera_detection_preview hik 0 0.15 2500
```

普通 USB 摄像头的设备索引 0：

```bash
./build/camera/camera_detection_preview usb:0 0 0.15
```

颜色参数 0 为蓝、1 为红。曝光参数只用于海康接口；实际图像通道顺序和曝光效果仍按当前采集接口实现核对。

窗口内 B/F 切换蓝/红，R 清空像素滤波器，Q/Esc 退出。先观察静止板，再缓慢左右移动：静止时两个标记应接近，平移时预测点应大致领先移动方向。预测偏移不是已测量的预测误差。

仅做无硬件合成图像检查：

```bash
./build/camera/camera_detection_preview --self-test
```

该分支在创建任何相机对象之前执行，覆盖无数字灯条检测、移动目标像素预测、叠加图渲染和短时/长时丢失处理，不打开窗口、串口或相机。
