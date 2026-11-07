# 毅同融合工作区升级：旧 suntry → origin2/suntry

## 版本和操作范围

- 当前修改分支：`sentry毅同融合`，起始/当前HEAD均为`a3b75d2`。
- 初次融合的旧A基线：`suntry`=`04fcc30b37e1b59a3831d639cb7cf4a5e53619f8`。
- 本轮新A基线：`origin2/suntry`=`405b40812b72705b8d5b173a5ef2529ecab84efa`，与用户截图一致。
- B核心仍采用`sp_vision_25:auto_aim_sentry`=`b2c7dc5`，补充`main`=`bd9f5e7`；沿用初次融合的Tracker/Target/Aimer/Shooter适配，不修改同济仓库。

本轮仅修改融合工作区，不提交、不暂存、不推送、不切换其他分支，也不建立新版本分支。所有本地分支提交号核验保持原值。`origin2`仓库的在线fetch因本工具缺少GitHub认证失败，因此明确以现有的405b408远程跟踪提交为基线，没有声称重新核验服务器实时最新。

本地没有未提交修改时才开始整合，原融合源码已归档到`/tmp/sentry-origin2-upgrade-hq04pwu8/fusion-before.tar`。没有对工作区执行Git合并或强制重置；三方合并只在临时文件上比较，再逐项适配。

## 参数对照及生效路径

| 项目 | 旧A/原融合 | 新A | 本轮实际处理 |
|---|---:|---:|---|
| 弹速 | 21.5 m/s | 22.0 m/s | Base.h更新，AutoArmor→Aimer和Buff继续使用同一宏 |
| 主自瞄yaw补偿 | 0.4° | 1.50° | Transmit YAML→Config.yaw_offset→Aimer命令，已有弧度转换保留 |
| 主自瞄pitch补偿 | 0.8° | 1.10° | 同上，进入最终弹道俯仰角 |
| 主自瞄预测延迟 | 0.07 s | 0.08 s | YAML和Config默认值同步，保留同济迭代飞行时间预测 |
| 开火角容差 | 1.5° | 2.5° | Sentry近/远两容差同步；保留首次禁发、确认、真实观测、pitch/yaw对齐门控 |
| 前哨曝光 | 没有单独字段 | 2500 μs | Camera.exposure_outpost及main独立mode3路径 |
| 初始相机曝光 | buff=2000 μs | armor=2500 μs | main初始相机构造同步新版 |
| 数字自适应阈值 | 19/-3 | 普通19/-2、前哨21/0 | ArmorDetector构造增加参数，AutoArmor模式切换传入对应值 |
| 灯条预处理 | 3×5闭运算 | 3×5膨胀 | 同步DILATE；去掉数字提取前的调试画线，防止污染分类输入 |
| 前哨控制速度条件 | 无 | 2≤abs(角速度)≤3 rad/s | 新Config参数+同济Aimer入口；使用11维状态[7]，不误用旧10维[9] |
| Buff pitch/yaw补偿 | 1.25°/0.7° | 0.9°/0.50° | 按源分支更新，预测时间0.05 s保持 |
| Buff灰度阈值 | 60 | 40 | 同步源码 |
| Buff圆轮廓面积 | 5000–8000 px² | 8000–12000 px² | 同步源码 |
| Buff调试 | debug=false，gray窗口注释 | debug=true，开启gray窗口 | 保留新A行为，未在无显示器机载环境验证 |
| 小符候选开火条件 | 需两片扇叶 | mode1可不要求两片，其余仍要求两片 | 同步表达式并修正原注释；只编译，不执行硬件 |
| 自动录制 | 每帧调用record | 注释record | main与新A一致；采集帧率和驱动不改 |

相机内参、畸变、外参、曝光其余值、USB参数、原有EKF配置组和Buff延迟全部与新A YAML逐项一致。新A这些标定数值相对旧A并未改变，因此没有推断或重新标定，也没有拿B参数覆盖A。

新增前哨门控不是只清fire：速度不足或过大时Aimer返回无控制Command，write_command清视觉字段，Shooter清历史；mode3不进入外部旧角度保持。它对应新版A在规划/编码前退出的行为。

## 18个上游差异文件的取舍

| 文件/组 | 处理 |
|---|---|
| ArmorDetector.h/.cpp | 纳入新版阈值接口、形态学与去除提前绘图；初始化列表仅调整为声明顺序以消除warning |
| ArmorEKF.h/.cpp、ArmorTarget.cpp | 同步新版参考源码，但继续从运行/复盘构建排除，没有恢复旧跟踪器 |
| AutoArmor.h | 保留融合类结构与整数离散模式缓存；原A的last_mode类型更改不适用于已经重写的接口 |
| Base.h | 纳入22m/s |
| AutoArmor.cpp | 按融合入口传入新识别参数，保持B跟踪核心；新前哨门控在Aimer适配 |
| AutoBuff.cpp、BuffDetect.h/.cpp | 同步新补偿所对应的处理、阈值、调试与小符判断 |
| main.cpp | 组合新A曝光/前哨路径与融合已有重置、USB敌色同步、导航长度保护 |
| CMakeLists.txt | 加aim_preview，继续保留融合源和旧算法排除规则；保留无硬件核心构建 |
| Test/src/aim_preview.cpp | 添加新A相机预览入口；曝光从本地配置读取，无串口发送；本轮未运行相机 |
| Test/src/test.cpp | 新进度条/暂停与融合realtime=false、模式/敌色重置结合；show=0不创建窗口；回放不构造相机 |
| text.yaml | 原有A组与origin2/suntry完全一致，另保留并更新Sentry组 |
| tools/player.cpp | 新A帧数回退；补充末行无换行、可靠跳帧、无效记录拒绝和长时间戳保护 |
| logs/aim_preview.jpg | 不是参数或模型，未把上游历史预览图片作为本次验证结果复制 |

旧A还将ChooseProcess重捕获间隔0.5改为1.5秒，调整旧EKF前哨噪声dd/da、100帧后锁速，并修正前哨旋转中心初始化。由于本融合实际运行B的11维Target，这些规则没有可直接替换的相同字段：继续使用B自己的角度环绕、收敛锁速、丢失计数和max_frame_gap，不把旧10维的索引、协方差或秒数按假定帧率硬换过去。这些差异已经保留在对照中，不声称完全复现原车旧跟踪算法。

## 中文数据流说明

`AutoArmor::AutoAim(Mat&, const Data_Save&, Data_Get&)`的图像是可写引用，反馈只读，命令是可写引用。模式变化时创建Detector并传入普通/前哨自适应阈值；A解算器按原标定求位置，Adapter按旧融合约定生成B观测，Tracker更新Target。Aimer读取新弹速、补偿、延迟并迭代求弹道，Shooter使用2.5°容差以及确认/本帧观测等条件，最后只由原编码器写Data_Get。此次没有修改21字节协议、设备名、波特率或坐标单位。

前哨门控使用`source.ekf_x()[7]`：这是B状态向量中的角速度，单位rad/s。旧A下标9表示角速度，而在B中下标9是半径差；直接照搬下标会把米误当成rad/s，因此必须显式适配。

回放跳帧后调用AutoArmor::Reset和外部AimStateMachine::Reset，并清视觉字段。Player重新打开文件并顺序跳过前N帧，保证图像和原车7列记录同时跳到N；不信任本环境曾失败的CAP_PROP_POS_FRAMES随机跳转。记录缺失/非法时停止，不返回未初始化IMU；小时级秒数不再经过32位微秒整数。

## 已执行的编译和测试

- 修改前：原融合15/15核心CTest通过，ASan+UBSan启用。
- 修改后：18/18通过，ASan+UBSan启用；包括桥接、21字节编码、2/3/4板、关联、跟踪/丢失、异常时间、优先级、首次禁发、yaw环绕、移动目标提前量、弹道、旧命令保持、PnP、空帧。
- 新增baseline_parameters：22m/s、1.50/1.10°补偿进入计算，2°对齐在新2.5°容差内允许候选，2.6°仍拒绝；首帧仍不允许候选发射。
- 新增outpost_gate：合成0、1、2.51、10rad/s场景，区分可控/不可控范围并清理无效瞄准点。
- 新增player：前后跳帧、3600秒时间、图像与姿态字段对应、无换行末行、负索引和损坏记录。
- review真实源码编译单元以及全部Buff源码编译单元通过；未构造这些组件或执行硬件。
- 完整工程配置在rclcpp缺失处失败；MVS也未安装，不能把核心编译说成整车链接通过。

命令：

```bash
cmake -S . -B /tmp/sentry-origin2-updated-build \
  -DAUTO_AIM_CORE_ONLY=ON -DSENTRY_SANITIZE=ON \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG='-O0 -g1'
cmake --build /tmp/sentry-origin2-updated-build -j2
ctest --test-dir /tmp/sentry-origin2-updated-build --output-on-failure
```

### 同段录像对照

使用同济assets/demo/demo.avi前200帧，原融合快照与新版使用相同视频、单线程、Debug O0、ASan+UBSan。输入IMU为零姿态夹具，没有混用同济标定，没有位置真值，也没有宣称模拟姿态是真实IMU。

| 项目 | 原融合 a3b75d2 | 更新工作区 |
|---|---:|---:|
| 输入帧数 | 200 | 200 |
| 输出有效目标帧 | 0 | 0 |
| 平均处理ms（单线程Debug O0+ASan） | 5.5775 | 4.5215 |
| P95 ms | 8.2883 | 5.8617 |
| CSV fire | 全0 | 全0 |
| 退出时LeakSanitizer | 792字节/3处，进程退出1 | 792字节/3处，进程退出1 |

两版在这200帧都未输出有效目标，属于空输出工况；不能用这次录像证明识别/跟踪稳定性或命中率提升，也不能把测得时延当作实车有目标处理速度。原始CSV位于`/tmp/sentry-origin2-offline-results/old-single.csv`与`new-single.csv`，完整LSan栈见validation日志。所有数值来自实际运行，进程的泄漏退出未标为通过。

首次录像执行的退出阶段，LeakSanitizer报告来自libtbb.so.2分配栈的3处共792字节泄漏；单线程后仍出现。录像内存检查不能记为通过。没有关闭或抑制泄漏检查，未修改系统库；该问题仍需定位。它和18项独立核心回归的通过结论分别记录。

安全复现录像夹具：

```bash
/tmp/sentry-origin2-updated-build/Test/sentry/sentry_offline_replay \
  "$PWD/text.yaml" /home/yuningli/RM-auto-aim/sp_vision_25/assets/demo/demo.avi \
  200 /tmp/sentry-origin2-replay.csv
```

程序只读文件/模型、执行算法并写CSV，不构造相机、SerialPort、ROS2或Buff；CSV的fire输出强制为0。它不等同于`action`。生产action保留原工程硬件通信行为，本次没有启动。

## 尚未完成

实机方向/限位、真实弹速、补偿重新实测、前哨换板/高度假设、检测准确率、真实IMU对齐、ROS2导航、下位机联调及自动发射都没有验证。新版Buff开启调试GUI，部署到无显示器环境前也要确认窗口策略；本次只编译其源文件，没有运行Buff。请依据真实机载验证结果再决定实车启用。

## 工作区检查

- 融合分支HEAD仍是a3b75d2，新修改以未暂存diff/新增文件存在。
- suntry、其它本地分支和同济HEAD未改变，同济工作区干净。
- 模型二进制、相机/串口驱动未改变。
- 没有git add/commit/push或任何真实硬件动作。

详细测试日志和参数核对结果见本目录validation；可用`git diff`审阅改动，新文件也需单独查看，暂未进入索引。
