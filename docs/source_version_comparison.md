# 重新融合的来源与版本核对

核对日期：2026-11-08。所有 SHA 均来自 Git，不以截图短 SHA 代替完整版本。

| 来源 | 实际引用 | 完整 SHA | 本次用途 |
|---|---|---|---|
| A 最新原车 | `suntry-origin2` | `405b40812b72705b8d5b173a5ef2529ecab84efa` | 硬件、参数、I/O、入口、Planner 和控制编码 |
| B 融合原顶部 | `sentry毅同融合` | `3a52df8d805d5972364984fce242eb8b737537dd` | 保留的历史父提交；前两次算法实现不参与重建 |
| B 内容基线 | 历史中唯一精确标题「完成复盘内录」 | `04fcc30b37e1b59a3831d639cb7cf4a5e53619f8` | 从此树重建，然后应用 A 最新差异 |
| C 同济算法 | `tongji/auto_aim_sentry` | `b2c7dc5ddfce77f8c5e7557c2293d759789e787d` | YOLO/分类/角点、Solver、Tracker、Target、EKF |

## 实际仓库与路径

- B：`/home/yuningli/RM-auto-aim/perservance-rm-auto-aim`，其 `.git` 是指向 `/home/yuningli/.git` 的文件，并非独立 Git 目录。不能在 `/home/yuningli` 直接执行 checkout、清理或提交，以免把用户主目录视为工作区。
- `suntry-origin2` 存在于 B 的 Git 数据库中。嵌套原车目录 `perservance-rm-auto-aim/auto-aim` 是另一个仓库，实际分支名为 `suntry`，但其 HEAD 也恰好是上述完整 A SHA。本次按 `suntry-origin2` 的固定对象读取，未切换或修改嵌套原车工作区。
- C 独立仓库：`/home/yuningli/RM-auto-aim/sp_vision_25`。其 `tongji` 远程为 `https://github.com/TongjiSuperPower/sp_vision_25.git`。
- 已对同济官方 URL 执行 `git ls-remote --heads ... refs/heads/auto_aim_sentry`，远程返回完整 `b2c7dc5...`，与本地两处该对象一致。算法和模型来自这个历史分支归档，未用 main 算法代替。

B 配置的 `origin` 是 `https://github.com/lyn1272602351-lab/perservance-RM-auto-aim.git`。
本地缓存的 `origin/sentry毅同融合` 为 `3a52df8...`。用户反馈其 WSL 终端的 HTTPS 查询已成功；本执行工具再次使用同一 HTTPS URL 查询（包括登录 shell 和嵌套原车目录），仍返回无法读取 GitHub 用户名。两种环境的认证状态不能混为一谈；没有尝试 SSH，也没有把缓存当作远端最新状态。

用户最新约束为「只修改指定本地分支，不提交、不推送」。本轮只把已保存的候选工作树接到现有本地 `sentry毅同融合`，保留全部未提交改动和原提交 SHA，不执行依赖远端核实的整合、提交或推送。

## 备份和重建方法

备份目录：`/home/yuningli/RM-auto-aim/sentry-rebuild-uas0ydrd`。
其中 `fusion-3a52df8.bundle` 已通过 `git bundle verify`，包含原融合提交及祖先；`sources.json` 固定三个来源与基线。
`baseline.tar`、`original-car.tar`、`tongji.tar` 是对应对象归档。

隔离结果在 `worktree/`，现已检出本地 `sentry毅同融合`，HEAD 仍为 `3a52df8...`。先把这个新建工作树的索引与文件恢复到 `04fcc30` 内容，再应用 `04fcc30..405b408` 的完整差异，算法修改开始前已验证索引树与 A 完全相同。
随后重新移植 C。本次没有读取前两次融合实现来作为新算法代码；旧版本只在另一归档目录运行历史测试。
没有 reset --hard、clean、强制推送或修改源分支。

## 原车保留范围

`Camera`、`USBCamera`、`Transform`、`Planer`、`EKF`、`Transmit`、`Buff` 所有原有 YAML 分组与 A 语义一致，只新增 `Algorithm` 分组。
保留 A 的 22 m/s、yaw/pitch 补偿 1.50°/1.10°、预测 0.08 s、2.5° 发射误差判据、普通半径初值 0.25 m、前哨半径初值 0.225 m。

相机驱动 cpp、串口驱动 cpp、ROS2 的发布/订阅/封装 cpp 及 Base.h 与 A 字节一致。SerialPort.h 仅补齐 `<thread>`、`<atomic>`，不改字段或接口。
主入口只增加跟踪/保持状态复位、感知指针空检查及导航数组长度检查，保留所有设备构造及发送流程。
实际差异和保留核对见 `rebuild_validation/preservation.json`、`rebuild_changed_files.txt`。

## 提交/远程状态

按用户最新要求，本次仅修改本地目标分支的工作树，改动保留在索引/工作区中，不创建提交、不推送。所有本地分支的提交 SHA 与本轮开始时一致；不存在新的「最终融合提交 SHA」。
这不是等待认证后自动提交的流程：在用户明确改变「不提交、不推送」要求之前，不会执行提交或推送。
这份说明不把隔离工作树视为已经部署的工程。
