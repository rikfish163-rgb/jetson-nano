# 2026-09-16 路牌检测训练实验

## 运行位置

4090：`/home/hts/robodata/runs/20260916_detector_v1`。
环境：该目录 `venv/bin/python`。复用已有 PyTorch/CUDA，新增依赖在独立环境中。
旧数据和 Nano 在线模型未替换，未启动小车。

## 数据与证据边界

- 原始数据 10000 张，按每类连续拍摄时间（间隔不超过 60 秒）聚合后分组。
- 204 个批次的中间代表帧做了全图审核，另审核了 203 个入选批次的代表裁剪。这是抽样审核，不是逐帧人工标注。
- 9911 张导出：train 6861、val 1420、test 1630；89 张进入排除清单，原始文件保留。
- 排除批次 ID 186（代表帧为锥桶与散放的路牌）及缺框、目标跨越裁剪边界的帧。
- 批次 28–83 的派生训练图裁去左侧 190 像素，移除未确认类别的远处第二块牌；与裁剪区域相交的主目标样本排除。
- 框仍由规则生成，标记为 provisional。训练结果仅是开发集实验，不能当作独立实战准确率。
- 现有权重曾见过部分重新划分的数据，因此微调分类器的开发集分数尤其不能当作盲测。
- 原始数据、分组、框、排除原因可追溯：dataset/manifest.json、training_data/manifest.json、training_data/excluded.json。

## 初始训练配置（当前配置见文末）

| 任务 | GPU | 配置 | 日志 |
|---|---|---|---|
| YOLOv5s | 0 | 640、batch16、AdamW、最多150轮、patience25 | yolo.log |
| D-FINE-L | 2 | 640、batch8、80轮、COCO预训练微调 | dfine.log |
| ResNet18 修正版 | 1 | 现有 best.pt 初始化、batch64、最多60轮、patience12 | refiner_area.log |

所有方向牌训练禁用镜像翻转。YOLO 关闭 hue 扰动，使用位置、尺度、透视增强；分类器使用裁剪边距、缩小、模糊、透视与明暗增强。D-FINE 本轮为无翻转的基础检测对照。

每项有 `<name>_job.json`、日志、完成后的 `.exit`。查看状态时应先检查 PID 和文件时间；早期失败尝试的 exit 文件可能早于当前 job 开始时间。

## 已知错误回归

真实 LEFT 被识别 PARKING 的原始裁剪：1789529627.286903143_PARKING_crop.png。

第一次回放发现，仅将上采样从 INTER_AREA 改成 INTER_LINEAR，旧模型输出就从 PARKING 0.81456 变为 LEFT 0.99841。Nano 实际使用 INTER_AREA。

首版微调候选采用 LINEAR 预处理，按 AREA 复测仍输出 PARKING 0.68883，因此不能验收或部署。它保留在 `refiner/` 作为失败对照。

修正版 `refiner_area/`：验证与推理固定使用生产 INTER_AREA；训练混用 AREA/LINEAR，并用固定 16/24/32 像素退化验证辅助选权重。合成压力测试仍不等于真实小牌准确率。

## 验证与兼容修复

- 分组不跨集合、分组不足失败、边界裁剪、背景裁剪：4 个单元测试通过。
- 导出数据的框边界、七类覆盖与禁翻转配置检查通过。
- 首版分类器 ONNX 检查与 PyTorch/OpenCV 输出一致性通过，但真实错误回归失败；导出成功不代表识别合格。
- YOLOv5 固定 v7.0 commit 915bbf294bb74c859f0b41f1c23bc395014ea679。
- NumPy 的 trapz 旧名改用等价 trapezoid；移除 YOLO 对 pkg_resources 的依赖，使用 packaging/importlib.metadata。
- 官方 YOLO 权重必须以模型对象加载；加载器仅允许本实验 weights/ 和 yolo/ 路径，不接受外部任意文件。
- 环境冻结与依赖审计见 environment-freeze.txt、dependency-audit-final.json。

## 后续验收

候选训练完成后，需完成：真实错误帧和连续视频回放、逐类误报漏检、小目标覆盖、检测器单独与分类器复核对比、Nano 导出/延迟/预处理一致性测试。

本次没有自动替换在线模型，也没有把训练/开发集高分当成上车许可。第二个 STRAIGHT 的状态机抑制问题仍需单独处理。

## 多卡续训（2026-09-16）

- YOLO 从已完成第 53 轮的完整断点续训，使用物理 GPU 1、2。
- D-FINE 从已完成第 10 轮的完整断点续训，使用同 NUMA 节点的物理 GPU 5、6、7、8。
- 使用 DDP（每卡独立进程同步梯度），恢复 optimizer、学习率调度相关状态、模型与已有 EMA/AMP 状态；YOLO 原版断点不保存 GradScaler，恢复遵循其原生行为。
- YOLO 总 batch 从 16 改为 32，累积后的有效 optimizer batch 仍为 64。
- D-FINE 总 batch 从 8 改为 32，每卡仍处理 8 张；不额外放大学习率。每轮 optimizer 更新次数随之减少，属于训练配方变更，须比较验证结果，不能视为数值等价迁移。
- 单卡运行目录 yolo/、dfine/ 和最佳权重保留；新输出 yolo_ddp/、dfine_ddp/。迁移证据在 ddp_migration/handoff.json。
- 最近不足一轮的计算从完整断点重跑，没有从预训练权重重新开始。
- 两卡/四卡 NCCL 求和正确性通过。D-FINE 初次多卡评估发现 CPU gather 缺少后端，改为 cpu:gloo,cuda:nccl，并验证 CPU all_gather 与 GPU all_reduce；多卡初始化失败会直接报错，防止多个独立进程同时写模型。
- 2 卡 YOLO 实测每轮训练计算部分约 24–25 秒（单卡约 45–47 秒）；验证和存盘另计。
- 4 卡 D-FINE 第一轮训练计算部分实测 1 分 51 秒（迁移前最近三轮单卡约 4 分 42–43 秒），约 2.5 倍；该轮已完成验证并保存 last.pth。首轮多卡验证 mAP@0.5:0.95 为 0.83307，恢复断点的验证为 0.83664，尚不能声称精度改善。
- D-FINE 新目录预置原最佳阶段权重作为第 70 轮阶段切换的可用断点；原单卡最佳继续保留用于最终跨运行比较。
- 当前状态使用 tools/status.py，会优先读取多卡任务，不将已迁移的单卡进程误报为当前训练失败。

## 第二次加速（2026-09-16）

- YOLO 从已完成第 84 轮断点续训，改用物理 GPU 0、1、2、3，总 batch 64，每卡仍为 16，有效 optimizer batch 仍为 64。
- D-FINE 从已完成第 17 轮断点续训，改用同 NUMA 节点的物理 GPU 5、6、7、8、9，总 batch 80，每卡 16；验证总 batch 160。保留原图像尺寸范围、模型、增强、AMP 和学习率，不缩短计划轮数。
- D-FINE 扩大 batch 会减少每轮 optimizer/EMA 更新次数，属于训练配方变更；速度提升不等于收敛和精度等价。须跨运行比较最佳权重，并用真实错误帧、视频与 Nano 测试验收。
- `bench_dfine_batch.py` 在空闲卡上使用真实样本、模型、损失、反向传播与 optimizer 更新，固定最大输入 800×800，预热 3 步、计时 7 步。单卡 batch8 为 27.65 张/秒、峰值分配 8.68 GiB；batch16 为 37.68 张/秒、峰值分配 17.06 GiB，预留 19.13 GiB。该短测不含 DDP，整轮耗时以正式日志为准。
- 新输出 `yolo_fast/`、`dfine_fast/`，日志 `yolo_fast.log`、`dfine_fast.log`。原单卡和首次 DDP 目录、最佳权重全部保留。
- 迁移时冻结本实验旧进程组，验证完整 checkpoint 与 optimizer 后复制，再结束旧任务；确认 GPU 已释放后启动新任务。证据 `fast_migration/handoff.json`，未完成的那一轮从完整断点重跑。
- D-FINE 新输出预置此前阶段最佳权重，供第 70 轮阶段切换读取；训练前执行恢复断点验证。最终选权重需包括更早单卡最佳。
- `status.py` 优先读取 `_fast` 任务，其次 `_ddp` 和初始任务。加速脚本语法检查通过，实际训练/验证/存盘检查以运行日志为证。
- 正式运行验证：4 卡 YOLO 连续多轮训练部分约 12 秒（之前 2 卡约 24 秒）；5 卡 D-FINE 首轮训练部分 74 秒（之前 4 卡稳定约 100–101 秒），两者不含验证和存盘。
- 已读取新 checkpoint：YOLO 完成 90 轮，D-FINE 完成 18 轮，optimizer 均存在；新日志无 Traceback/OOM/ChildFailedError。D-FINE 第 18 轮验证 mAP@0.5:0.95 为 0.82479，迁移前第 17 轮为 0.82607，尚无精度改善结论。证据 `fast_verification.json`。
- 随后第二轮 D-FINE 训练部分降至 55 秒，完成第 19 轮验证，mAP@0.5:0.95 为 0.82235。训练速度确有提升，验证分数没有同步改善，原最佳权重继续保留。
