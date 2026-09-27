# 变道复查后的本地修复（2026-09-25）

本轮按 [09-24 当前检查](LANE_CHANGE_CURRENT_REVIEW_20260924.md) 中可复现的问题修改本地候选。用户要求任何变道限制须有参考依据；本次没有新增速度、距离、概率或等待门槛。2026-09-25 已离线装车，道路效果未验收。

## 行为变化

1. `efficiency_lane.py` 在超车候选侧分别归属 ARS408 目标。另一侧外线缺失、目标贴边或只含预测点，不再让可用侧一起变成 `efficiencyLaneGeometryUnknown`。模型走廊以外且超过可见纵向范围的路外目标不参与本次归属；相关侧模型线缺失、目标归属在该侧不明确时仍只挡该侧。视觉线仅用于目标归属，跨线许可仍由 C3 视觉或新鲜 0x399 给出。
2. 对目标侧所有已测前方目标调用既有 CP `side_lead_unsafe` 判据；最近目标仍决定预计速度收益。原来“8 m、相对速度 +3 m/s 近目标安全，但 20 m、-10 m/s 的另一个目标不安全”却允许左侧请求的案例现在拒绝。来源为机械小鸽 CP `Carrot@c1849b501776b612af50f029b58c67ac75675b9e` 的 `SideState` 单目标判据与多目标遍历。不能据此声称 ARS408 实车标定已完成。
3. `nav_lane_intentd.py` 只在导航灯已请求，或有效推荐目标确实不在当前车道时，让主动超车让位。普通 `none/straight` 机动的 2 km 距离、未推荐的普通车道信息不再占用。参考 jihui Amap `141ebea0b4b4a4956dbf2345257f950d4c2609b9` 的实际导航动作／已发命令让位条件。
4. 已知 `laneChangeCancelled` 取消当前动作后执行既有失败冷却；当模型回到 `off`、实体灯关闭且收益再次稳定时，同一导航 session 可重新请求。物理完成未确认、源中断等情况仍保留原会话锁止。来源为 jihui Amap 同版 `finishOvertake(false)` 后失败冷却／复位及原 SP 状态机。
5. 移除协调器自行增加、未找到同用途参考的 300 ms 跨线许可连续确认，以及观察丢失撤灯后的 3 s 额外恢复等待。实际灯光反馈、视觉／399 许可、盲区、司机接管及原 SP 控制资格仍是启动条件。观察恢复仍用已有 500 ms 导航对齐稳定期；该时长在 jihui Amap 同版动作稳定路径有参考。

`diagnosticsd.py` 记录每侧归属是否已知，并列出该侧违反已有间隙判据的目标 ID，避免“一侧未知”被写成“空侧”。

## 验证与状态

本地复现脚本：`D:/C3/.codex-tmp/lane-current-review-repro-20260924.py` 与 `lane-route-reservation-repro-20260924.py`，同名 JSON 保存修复后结果。原生隔离回归在车机离线时于 `/tmp` 加载本地候选，禁用消息发布／订阅、隔离 Params 和测试文件；不安装生产代码。最终 **794 项通过**，包括每侧独立归属、全部侧前目标检查、取消后冷却恢复、无实际导航变道需求时主动超车、实体灯与跨线许可立即就绪、观察恢复和诊断记录。测试运行前后 19 个车端生产文件哈希一致，临时目录已清理。日志：`D:/C3/.codex-tmp/lane-check-native-20260925.txt`。安装前只读证据：`D:/C3/.codex-tmp/lane-current-audit-20260925.json`。

## 2026-09-25 装车回执

用户授权将已验证的变道链路候选一起装车。车机 `IsOffroad=1`，无 `controlsd`、`modeld`、`nav_lane_intentd`、`selfdrived` 或 `card` 进程时，将有实际逻辑差异的 10 个文件安装至 `/data/openpilot`：`efficiency_lane.py`、`settings.py`、`lane_intent.py`、`nav_lane_intentd.py`、`diagnosticsd.py`、`oem_lane_change_gate.py`、`lane_change_blocker.py`、`desire_helper.py` 和两个 `modeld.py`。每个旧文件先备份并核对哈希，安装后 10/10 哈希一致、10/10 模块独立导入通过；车机仍离线，未主动启动行车进程。备份：`/data/lane-change-backups/lane-20260925-20260924-235821-1186771`；本地回执：`D:/C3/.codex-tmp/lane-deploy-20260925/receipt.json`，复核：同目录 `verify.json`。仅换行差异和其他功能的未提交改动未覆盖。正常 onroad 加载与实际变道仍待验证。

## 2026-09-25 jihui 匝道分叉执行补齐（本地候选）

前一轮仅接入入口 roadType=10 并移除 ramp/exit 固定 36 km/h 目标，未补上最终分叉执行：近匝道可能退回 targetLaneIndex=-1 的只打灯请求。此次将明确 ramp/exit/merge 的最后分叉接入既有 NavLaneIntentCoordinator → DesireHelper → 模型变道意图链路。

参考版本：jihui Amap `141ebea0b4b4a4956dbf2345257f950d4c2609b9`、CP `8cb0970af4757ab65370ce9587bc23f6f65a8166`，本地参考位于 `D:/C3/.codex-tmp/jihui-amap-reference-20260924`、`jihui-cp-reference-20260924`。App `MainActivitySettings.kt:290-295` 的高速 fork_dist_h 默认 80 m；`MainActivityNaviProcess.kt:187-214,378-425` 先靠边、近分叉 doLaneForkNow。CP `selfdrive/controls/lib/desire_helper.py:534-548,906,1266` 接收 fork now、不用普通变道最低速度、无需完整邻道分类，并在完成后结束本次 fork。

- 明确方向的 ramp/exit/merge 在 0 < 距离 < 80 m 发起最后分叉；普通 turn/slight/keep 不因此强制分叉。80 m 是参考动作时机，未经 C3 道路标定，不是新安全许可阈值。
- 最后分叉可使用既有视觉／0x399 正许可；不再额外要求完整邻道分类。保留原有路缘、盲区、雷达检查、司机接管、控制资格和停车限制；不复制参考中的无条件实线／未知边界放行，ignoreSolidBoundary/allowUnknownCrossing 仍为 false。既有视觉／399 OR 判据不变。
- 正在执行的 fork 按事件保持，避免导航距离波动变回普通低速转弯；使用原模型 starting→finishing→pre 完成事件，同一事件只执行一次，不用打灯证明完成。
- 两个 modeld 入口对有效低速 fork 使用同一变道许可模式；界面仅在实际 ignoreSolidBoundary=true 时显示“实线放行”。
- 速度仍由现有建议速度、弯速及纵向规划决定，没有新增统一匝道速度、手写轨迹或方向盘控制。

验证：车机隔离目录 `/data/amap-ramp-test-lxi3rad7` 加载候选模块，417 项测试通过；另 16 组真实 App 生产序列化包经过 C3 解析、计划及协调器检查通过。覆盖左右低速分叉、实际 DesireHelper 变道意图、模型完成状态、一次完成后不重复、距离抖动、实体灯交接、双模型入口许可表达式、司机／盲区／路缘／实线等退出条件及 UI 文案。复现脚本与结果：`D:/C3/.codex-tmp/amap-ramp-20260925/verify.py`、`result.txt`。

本轮未安装车机生产文件、未重启、未修改车端参数；本轮无新增 App 源码，沿用上一轮 App 200 项测试和 APK。隔离测试不包含真实道路模型轨迹或自动上下匝道验收；仍需装车后核对实际加载及道路效果。


### 后续连续模拟验证

用户要求自行模拟验证后，增加 56 组 App 生产测试报文驱动的连续时序模拟（4 种 roadType × 左右 × 7 场景），全部通过；417 项回归和 16 组报文检查亦通过。覆盖提前靠边→最终分叉、目标先占用后离开、车道观察丢失、距离 90/0 m 抖动，以及目标持续占用、无灯反馈、模型不完成时不得误报完成。报告 `D:/C3/.codex-tmp/amap-ramp-20260925/SIMULATION_REPORT.md`，逐状态记录同目录 `simulation.json`；车端隔离目录 `/data/amap-ramp-test-fvbplhte`。传感器及模型完成概率为合成输入，未运行真实图像推理／车辆动力学／纵向控制闭环；未部署或发送控制消息。


## 最终分叉范围修正与长高架复验

已按 jihui 的高速分支给最终 force_fork 加入现有 highway(roadClass=0/6)，保留普通道路提前选道。447 项回归、16 组 App 报文、56 组连续软件场景通过。约09:09–09:41的38178个录制采样连续回放完成：入口108个多余最终分叉请求全部消除；整套当前候选在长高架不再继承原 efficiencyCompletionUnconfirmed 长期锁止；快速路出口分叉仍保留。出口回放仍读取旧模型取消反馈，不能判定新闭环已成功。未部署。完整范围和证据：`D:/C3/.codex-tmp/long-elevated-replay-20260925/REPORT.md`。

## 最终分叉保持与次数修复（本地候选，未部署）

用户要求复查后直接修复并测试。两处生产模块区分导航源有效与推荐映射有效，在同动作已选分叉中保持原方向／身份，适配视觉窗口变化；最终分叉独立于提前选道次数，由已有模型完成终态保证一次。主循环同步保持目的，防止明确取消后被普通转向灯重新覆盖，新有效事件正常释放旧终态。普通绝对计划升级分叉时同步相对完成语义。依据是 jihui CP `8cb0970` 的 fork_now 独立次数及完成状态；没有新增控制门槛或改变轨迹／纵向机制。

新增44例回归，完整491项、16组App生产报文、56组连续软件场景通过，隔离目录 `/data/amap-ramp-test-2gt1nc_d`。38,178采样在 `/data/amap-ramp-test-cia5mw96` 重放，输出与修复前修正过的回放逐帧相同；本轮缺陷组合由新增回归覆盖，实录只用于确认无新增行为变化。未部署、未道路验收，已重读最高约束并通过独立审查。精确补丁、哈希、备份、日志及边界见 `D:/C3/.codex-tmp/fork-fix-20260925/REPORT.md`。

### 后续部署完成

用户授权后，在车机离线、全部行驶控制进程停止时安装上述候选及尚未安装配套，共8文件；备份 `/data/lane-change-backups/fork-20260925-_qaby3gn`。安装哈希8/8一致，直接加载实际生产模块的491项回归、16组报文、56组场景通过。刷新服务后通过离线重启manager恢复界面／导航，最终ui、navassistd、nav_diagnosticsd连续6秒稳定；控制进程仍停止，正常行驶加载与道路效果待验。本轮未装App、未整机重启。回执 `D:/C3/.codex-tmp/fork-deploy-20260925/REPORT.md`。
