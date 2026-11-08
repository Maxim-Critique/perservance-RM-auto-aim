# 同济来源

算法及模型来自 https://github.com/TongjiSuperPower/sp_vision_25.git
分支 auto_aim_sentry，完整 SHA b2c7dc5ddfce77f8c5e7557c2293d759789e787d。
原始文件 SHA256 见 UPSTREAM_SHA256.json，所有修改仅在融合工程的副本中。
该历史分支无 LICENSE 文件；保留官方 main bd9f5e798fa3c6dd3b483ae6627796afb41c608d 的 MIT 声明，未用 main 算法替代来源分支。

移植改动：工具命名空间隔离；配置通过 YAML::Node 注入；去除 Tracker 对 C 相机头文件的间接依赖；
动态图像中心；lost 状态不读取未初始化 Target；无 GUI 推理；ROI 颜色采样坐标修正；
大小/类别过滤集中到 Adapter；PnP 尺寸采用 A 的 130/230×50 mm；保留 IPPE 和带畸变 yaw 搜索。
C 的 Ceres 路径在构造残差时丢弃模型 z 且忽略畸变，本次禁用此路径，保留代码以便核查。

额外保留 A 的普通/前哨半径初值0.25/0.225 m；修复两甲关联越界和 EKF 诊断残差维数检查；
关闭 CPU 绑核，线程数可配置；日志加 tongji_ 前缀避免覆盖原车日志；YOLOBase 补虚析构。
