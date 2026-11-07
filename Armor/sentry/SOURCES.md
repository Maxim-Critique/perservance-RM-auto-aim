# 哨兵核心迁移记录

来源仓库：`/home/yuningli/RM-auto-aim/sp_vision_25`。

- 主要来源：`auto_aim_sentry`，`b2c7dc5ddfce77f8c5e7557c2293d759789e787d`。
- 补充来源：`main`，`bd9f5e798fa3c6dd3b483ae6627796afb41c608d`。
- 本目录的 `LICENSE` 保留来源仓库 `main:LICENSE` 的 MIT 版权声明。
- 初次融合 A 基线：`suntry`，`04fcc30b37e1b59a3831d639cb7cf4a5e53619f8`。
- 本轮工作区升级 A 基线：`origin2/suntry`，`405b40812b72705b8d5b173a5ef2529ecab84efa`。按要求不提交，融合分支 HEAD 仍为 `a3b75d2`；这表示新增修改尚未进入提交历史，不表示仍使用旧参数。

## 实际模块选择

| A 模块 | 当前实现 | 选择理由 |
| --- | --- | --- |
| `ArmorDetector` | 保留 A，主相机和双 USB 继续使用原模型 | 尚无 YOLO 模型效果和三路并发负载的对照数据；不复制 B 的检测运行时依赖 |
| `Slover` | 保留 A 的角点、尺寸、内外参、IMU 变换和 yaw 搜索，增加 PnP 输入/结果检查 | B 的角点与装甲板坐标约定不同；候选 Ceres 重投影还存在丢弃物点 z 和未处理畸变的问题，不直接采用 |
| `ChooseProcess` | `tracker.*` + `adapter.*` | 显式确认、跟踪、短时丢失、完全丢失状态；按优先级与实际图像中心选择；保留一块观测与对应板编号的关系 |
| `Aim/ArmorEKF/ArmorPredictor` | B 的 `target.*` 11 维整车模型 | 状态为 `[xc,vx,yc,vy,zc,vz,yaw,omega,r,dr,dz]`；2/3/4 装甲板初始化、关联、观测雅可比和预测一起迁移 |
| 通用 EKF | A 已有 `tools::ExtendedKalmanFilter` | 接口与 B 兼容，矩阵维度由新 Target 初始化；不启用 B 中固定 `0.711` 阈值的 NIS 重置规则，也不更改 Buff 的滤波行为 |
| `Planner` | B 的迭代 `Aimer` 与装甲板选择逻辑，使用 A 的弹道工具 | 统一考虑处理耗时、A 配置延迟与飞行时间；未收敛或无弹道解不输出目标；未引入 MPC |
| `Transmit` 的开火判断 | `shooter.*`，参考 B `main` 的历史命令更新顺序 | 先比较后记录；补充 pitch 对齐、角度环绕、确认状态与本帧真实观测门控；失效/换目标后首个命令不允许开火 |
| `AimStateMachine` | 保留 A 的状态及 500 ms 保持时长 | 保持视觉角度与 ID/found，清除 fire，不覆盖当前导航字段 |
| 双 USB `Perceptron` | 保留 A 的线程、相机及模型 | 同步敌方颜色；暂停/换色清队列；丢弃超过 150 ms 的结果，避免恢复时使用旧目标 |
| 相机、串口、ROS、运行入口 | 保留 A | 只增加状态重置、搜索暂停和导航数组长度检查；串口头文件补充缺失的标准库 include |

旧 `ArmorTarget.cpp`、`ArmorEKF.cpp`、`ArmorPredictor.cpp`、`ArmorPlanner.cpp` 留作参考，已从运行和复盘目标的构建源中排除；其原有评分矩阵问题不再处于执行路径。

## 适配约定

- A 颜色：0 蓝、1 红；适配层逐项转换为 B 的颜色定义，非法值拒绝。
- A 的 `yaw` 表示由整车中心指向装甲板的方向；B 表示相反方向，转换为 `limit_rad(A.yaw + pi)`。位置均保留 A 的世界坐标：x 前、y 左、z 上，单位米。
- A 角点顺序为左下、左上、右上、右下；继续交给 A 的解算器，无角点重排。装甲板宽高及标定不使用 B 数值覆盖。
- 输出保持 A 的 21 字节帧。yaw/pitch 为大端 int32、度乘 10000；ID/found/fire 在索引 17/18/19，导航字段由原 `Nav2Control` 写入。
- A ID 映射：1–5 数字、6 前哨站、7 哨兵、8 基地。没有按 B 枚举下标直接发送。
- ROS 仍发布 `target_number/Int16`，订阅 `target_pos/Int16MultiArray`；生产端和消费端均要求至少两个导航元素。
- `detecting` 可给出跟踪角度，只有 `tracking` 且本帧观测有效时才参与开火判断。`temp_lost` 可预测角度但不允许开火。B 的丢失计数结束后，A 外部 500 ms 保持和 USB 搜索继续生效。
- 模式/颜色变化重置跟踪、命令历史和外部保持；Buff 模式暂停 USB 搜索。复盘以日志时间推进模型，不把运行快慢误当成处理延迟。

## 配置与边界

`text.yaml:Sentry` 给出跟踪计数、最大帧间隔、距离、旋转选板和开火阈值。延迟与 yaw/pitch 补偿继续读取原 `Transmit` 配置。默认 2.5 度开火容差与新版 A 一致，弹速已改为 22 m/s；yaw/pitch补偿为1.50/1.10度，延迟为0.08秒。前哨控制另要求原始角速度状态的绝对值处于2–3 rad/s，取B的状态索引7；范围外同时清空控制和开火历史。普通目标、平衡、前哨站半径初始化采用 B 的 0.2 / 0.2 / 0.2765 m，仍需实测调参。

`priority_ids` 使用 A 的 ID，可配置目标优先级；默认延续 A USB 的 1、3、4、哨兵优先顺序。没有引入 B 的赛季特定 2/5 号过滤。A 未提供 B 的裁判无敌状态输入，本次未伪造该状态，也未新增 ROS 话题。主相机换目标遵循该配置，USB 继续 A 的已有优先级。

`AutoArmor` 默认构造保持原用法；可传入配置路径和 `realtime=false`，用于无设备回归或复盘。主相机分类模型从配置文件同级的 `model/` 读取，模型文件未改变。默认硬件配置路径仍由 A 的 `YamlPath` 决定。

## 验证方式

在 A 根目录运行：

```sh
cmake -S . -B /tmp/auto-aim-sentry-core \
  -DAUTO_AIM_CORE_ONLY=ON -DSENTRY_SANITIZE=ON \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG='-O0 -g1'
cmake --build /tmp/auto-aim-sentry-core --parallel 2
ctest --test-dir /tmp/auto-aim-sentry-core --output-on-failure
```

回归目标编译实际的检测、解算、跟踪、瞄准、开火、串口编码、USB 搜索代码；执行时只使用合成观测、投影角点和空白图像，未构造任何相机、串口或 ROS 实例。地址及未定义行为检查覆盖两装甲板候选越界、目标关联、时间异常、丢失恢复、开火历史、弹道边界和帧编码。

这些检查不能证明检测精度、整机延迟、命中率或三相机负载提升。完整工程依赖 ROS 2、MVS、OpenVINO；安装包缺失时核心回归可独立构建。完整工程配置或编译未通过时，不应把核心回归通过表述为整机验证完成。

初次融合验证：15 项核心回归通过（ASan + UBSan）。完整工程的 CMake 配置在 `find_package(rclcpp REQUIRED)` 处失败；`/opt/ros/humble` 和 `/opt/MVS/include/MvCameraControl.h` 均不存在。OpenVINO 2024.6 实际位于 `/opt/intel/openvino_2024.6.0`，设置其 `runtime/lib/intel64/pkgconfig` 路径后可被找到。未执行 `action`、`review` 或任何串口发送/机器人操作；尚无真实回放与新旧效果对比。


## origin2/suntry 405b408 工作区升级

仅在`sentry毅同融合`工作区整合新旧A差异，没有merge/cherry-pick/rebase、没有修改分支提交、没有暂存或提交。`suntry`及同济仓库保持原状。逐文件取舍、参数对照与实际验证结果见`docs/suntry_upgrade_405b408.md`。

- 主检测器采用新版A的DILATE和按模式的自适应阈值：普通19/-2，前哨21/0；继续使用A模型，不宣称迁入同济YOLO。
- 主入口初始采用armor曝光；前哨独立使用exposure_outpost，暂停USB；继续保留融合版本的模式/颜色重置、空指针保护和导航长度检查。
- Buff参数、阈值/圆面积与小符双扇叶条件按新A更新。保留源分支开启调试窗口的行为，尚未在无显示器机载环境运行。
- 旧ArmorEKF/ArmorTarget参考源同步新A，但仍排除在执行目标之外；不把旧10维滤波的噪声、100帧锁速或1.5秒重捕获规则硬套到B的11维模型。
- 新回放进度条保留，跳帧清空跟踪/开火/外部保持；show=0不创建窗口，也不构造USB相机。Player逐帧跳过保证图像与7列原车记录一致，拒绝错误记录，避免32位微秒溢出。
- 增加`sentry_offline_replay`仅视频夹具及新参数/前哨速度/回放回归。更新后18项核心回归（ASan+UBSan）通过，review编译单元及全部Buff编译单元通过。
- 录像夹具只比较模块能否运行，不证明检测精度或姿态。已发现退出阶段的libtbb分配泄漏报告，详情和未通过事项保留在升级报告中，不把录像内存检查记为通过。
