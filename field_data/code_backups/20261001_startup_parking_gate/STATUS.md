# 绿牌起步与停车牌阶段隔离

## 已实现

- 等绿牌、启动直行和起步后的初始循线阶段，不累计 PARKING 票数。
- 绿牌放行清除旧的 PARKING 缓存，并关闭停车牌接收资格。
- 完成 LEFT、RIGHT、STRAIGHT 或 UTURN 并恢复循线后，才开放新的停车牌投票。
- BYPASS 不开放停车牌接收资格；此前已完成路口动作的资格也不会被 BYPASS 清除。
- 缓存停车牌后，等待停车蓝线时仍沿车道中心循线。
- 原有停车蓝线触发入口继续使用；停车牌投票需要重新获得三次有效观测。

## 修改文件

- src/robot/master/state_machine.py
- src/robot/signs/decisions.py
- src/robot/lane/controller.py
- src/robot/test/test_startup_parking_gate.py（新增）

## 验证证据

所有检查通过 SSH 在 Nano 上运行，没有启动车辆或发布底盘命令。

- 修改前：新增的 7 项检查中 5 项失败，复现起步缓存停车牌、避障误开放停车、绿牌未清理旧缓存以及等待停车时偏离中心跟踪。
- 修改后：新增的 7 项检查全部通过。
- 相关扩展检查：52 项中 45 项通过、7 项失败。
- 使用备份中的修改前源码在独立 Python2 进程重跑原有 45 项检查，同样的 7 项失败再次出现；没有覆盖现场源码。
- 7 项原有失败涉及 4 项起步速度预期、1 项起步交接舵量以及 2 项车道舵量预期；本次未修复这些原有问题。
- module_workspace.py check：PASS 501 source/config files assigned; module overrides valid。
- 四个修改文件均通过 Nano Python2 语法解析，after_hashes.json 保存 Nano 文件校验值。
- protected_check.json 中八个配置、算法和启动脚本文件的校验结果均为 true。

详细输出：regression_before.txt、regression_after.txt、baseline_existing_tests.txt、module_check.txt。
源码对比：changes.patch；修改前源码：before/。

## Nano 启动命令

```bash
cd /home/nano/robocup_ws
bash tools/sign_detector_20260916/run_yolo_vehicle.sh
```

需要重新启动控制器才能加载修改。实车效果尚未验证。
