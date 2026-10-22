# 路牌每类 500 张重训（2026-09-14）

## 2026-09-15：补采非路牌

每类1000张、20 Hz：采集与续采命令加 `--include-background --per-class 1000 --hz 20`，
此时八类均为1000张（各 train 700、val 200、test 100），共8000张。
相机启动命令加 `framerate:=30`；实际保存速率受相机和编码写盘速度限制，只计新帧。
查看进度和完成检查也带 `--per-class 1000 --include-background`。

采集命令增加 `--include-background`，保留原七类各 500 张，另采
`background` 1500 张（train 1050、val 300、test 150），共 5000 张。
续采和 `--status`、`--check-ready` 也必须带同一参数。
非路牌组应移走真实路牌，轮换锥桶、蓝线和红绿蓝杂物；只计入正式颜色筛选
实际选中的 ROI，无候选画面仍抽样保存到 `missed/`，不计数。
训练脚本现支持 `--include-background`，以八个输出训练、评估并导出 ONNX。
输出顺序为 red、green、straight、left、right、uturn、park、background。
运行端仍是七分类；八分类候选只回传，不可直接替换在用模型。
八分类数据不要执行本文后面的旧七分类自动安装流水线。

类别及模型输出顺序固定为 red、green、straight、left、right、uturn、park。
每类训练 350、验证 100、测试 50，共 3500 张有效 ROI，同时保存对应原图。
采用正式框架的 `extract_sign_roi`。采不到 ROI 不计数；脚本只订阅图像。

## Nano 上采集

先停止自动驾驶任务。车保持静止，手持路牌改变位置、方向和光照。
前摄已经运行时不需要启动第二个相机节点；否则在第一个 Nano 终端执行：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch /home/nano/robocup_ws/tools/sign_retrain_20260907/camera_only.launch
```

第二个 Nano 终端执行（SSH 无桌面也能使用）：

```bash
cd /home/nano/robocup_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
python2 -u tools/sign_retrain_20260907/guided_collect.py
```

Nano 本机图形桌面可在最后加 `--preview`，查看原图和实际 ROI，空格暂停，q 退出。
每组提示当前牌和采集角度，摆好后回车，自动采集 50 张。默认每秒 2 张。
无候选牌、相机无新画面会提示；每组超时 10 分钟会保留进度并退出。
Ctrl+C 或 q 后运行原命令续采。计数按现有 ROI 文件重新统计，不覆盖既有图片。

样本目录：`/home/nano/robodata/signs_20260914/{train,val,test}/{label}/{session}/roi/`。
同组 `frames/` 保留全图，`missed/` 保留部分未检测图，均不计入 500 张。
不同集合需重新摆放场景，避免同一姿态的连续帧分别落入训练和测试。
不要对左/右箭头做水平翻转。颜色和语义相反的牌不能混入同一组。

采满后检查 ROI，删除错牌、裁错和模糊图，再用原命令补齐。
最终输入 `READY` 才产生 `CAPTURE_COMPLETE.json`，表示数据可用于训练。
标记包含各张 ROI 路径和内容的整体 SHA-256；同步前后均校验，修改图片后需重新检查并输入 READY。
仅数量采满不会自动标记审核完成。

进度查询：

```bash
python2 /home/nano/robocup_ws/tools/sign_retrain_20260907/guided_collect.py --status
```

## 采后训练和接入（由 Codex 工作站执行）

```bash
cd /home/hetaisheng/jetson-nano
bash tools/sign_retrain_20260907/pipeline.sh
```

流水线从 Nano 拉取独立快照至工作站 `/home/hetaisheng/robodata/runs/<时间>/data`，
再同步至 `4090:/home/hts/robodata/runs/<时间>/data`。仓库是 Nano 的 SSHFS 挂载，快照不写进仓库以免来回传输。
使用 `/home/hts/robodata/venv/bin/python` 运行现有训练脚本 100 轮，选择空闲显存最多的 GPU。
val 选择最佳模型，test 最终评估，输出 ONNX、混淆矩阵、报告和权重。
早停默认 patience=15：val 宏平均召回率连续15轮未提升则停止，仍导出最佳轮的权重。
跨集合完全重复文件会阻止训练；近似图和错标签仍需要采集者检查。

候选回传至 Nano `/home/nano/robodata/models/<时间>/`。Nano 使用独立测试快照对比新旧模型：
每类召回率至少 80%，每类下降不超过 3 个百分点，停车/掉头互相误判不超过 10%。
未通过则退出并保留候选与报告，不替换。
通过后备份并原子替换 `/home/nano/robocup_ws/src/ros/signs/scripts/resnet18.onnx`，
并同步工作站同一框架路径。已有识别进程在下次启动才加载新模型；流水线不重启车辆任务。
本地 `field_data/sign_retrain_20260914/INSTALLED.json` 是已安装完成的凭据。
失败后先查看日志和报告再决定重试；不要并发启动多个流水线。

新图未采集完成前，训练、最终精度与实际模型安装都尚未完成。
