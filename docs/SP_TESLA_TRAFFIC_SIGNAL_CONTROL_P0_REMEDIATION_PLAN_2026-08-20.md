# Tesla 红绿灯纵向控制 P0 修改方案

日期：2026-08-20
适用基线：`d5ea4797adced3976c857eecfe52d2eb8b10084e` 及其后续提交
状态：待实施；本文件只定义方案，不代表已修改控制代码

## 2026-09-19 适用范围与最小侵入复审

上述日期、基线与问题是原始审查记录，不代表当前代码仍包含全部缺陷或当前设备状态。沿用本方案前，先在实际目标版本复现对应问题；已修复项不重复实施。最高工程约束为最小侵入，当前范围以[城市 NOA 总体规划](../../C3_CITY_NOA_MASTER_PLAN.md)为准，本轮只修改设计。

保留现有 observer/controller/stop profile/FinalPlanArbitrator 的责任边界，先在现有函数修复已确认的零距离制动、事件连续性和绿色资格问题。terminalStop、releaseEligible 等是必要语义，不自动要求新增状态机或消息字段；确需扩展时说明现有表达不足及消费者影响。阶段 A 复用现有日志，阶段 E 起步调优是独立后续项，不与正确性修复捆绑。所有历史标定候选值均须结合目标版本和实际证据复核，不直接套用。

原文 Shadow 验收与当前“不使用影子模式”约定冲突；下文已收敛为现有日志的离线回放与软件仿真，不新增 Shadow 模式、常驻进程、第二套 planner 或雷达副本。台架、封闭场地及部署保留为后续验证安排，本轮不执行。

## 1. 结论先行

保留当前 `FinalPlanArbitrator` 作为三套纵向规划器之后的统一控制入口，不恢复旧
`planner_adapter`，不修改 Official、Experimental、TN-NoDEC 的内部决策、MPC、来源枚举
或工厂选择逻辑。

本轮只修改交通控制隔离模块，按以下优先级推进：

1. P0：修复“剩余距离归零但车辆仍移动时不再继续制动”。
2. P0：稳定同一红灯事件，消除资格瞬断造成的事件 4/5 快速重建。
3. P0：建立只对同一已停车事件生效的绿色资格规则，使自动绿灯起步可达。
4. P1（独立后续项）：确认起步性能问题仍存在后，局部调整现有有界起步曲线，同时保持前车、驾驶员和误绿保护。
5. P0：修正日志分析判据，以 `applied/action/baseATarget/finalATarget` 判断是否介入，
   不再使用基础 `longitudinalPlanSource` 推断交通灯是否控制。

在完成对应离线回放、软件仿真与封闭场地验证前，不把该功能视为可依赖的道路自动停车功能。

## 2. 已确认问题与根因

### 2.1 红灯停车超过目标点

当前 Stop Profile 在 `remaining_distance <= 0.01m` 时直接输出零加速度。状态机又只有在
`vEgo < 0.3m/s` 后才进入 hold。两者之间存在明确空洞：

```text
remainingDistance = 0m
vEgo = 0.9–1.3m/s
phase = braking
traffic aTarget = 0
shouldStop = false
```

车辆在此状态下可能继续滑行并超过内部停止点。

### 2.2 红灯事件快速替换

日志设备的 OEM 数据长期为 `featureState=0`、多数 `unavailableReason=1`。当前兼容规则
只接受 `featureState=0 + red + stateMachine=2`。同一路口的状态机进入其他状态后，即使
原始目标仍连续，也可能失去 `validForControl`；活动事件在 0.75 秒后重置，随后重新确认成
新事件。

### 2.3 自动绿灯起步不可达

相同 OEM 条件下，绿色帧不满足当前控制资格，因此：

```text
raw green seen
  -> validForControl = false
  -> release phase unreachable
  -> plannerStartRequested = false
  -> startApplied = false
```

不能通过简单地“允许所有 featureState=0 绿色”解决，否则任意未启用 OEM 功能的误绿帧都
可能触发车辆起步。

### 2.4 即便成功起步，当前曲线也过于迟缓

当前起步受以下硬限制：最大加速度 0.35m/s²、最大速度 1.5m/s、最长 2 秒、jerk
0.5m/s³，并且模型 `shouldStop` 或基础负加速度会完全阻止起步。因此稳定绿灯出现后，可能
长期等待基础 e2e 自行释放；真正介入时又只提供很弱的加速度。

## 3. 设计边界

### 3.1 必须保持

- 用户仍只有一个开关：关闭时只采集，开启时允许控制。
- 开关关闭时保持原纵向输出透明，不创建最终仲裁器。
- 三套纵向规划器共用同一个 post-planner 仲裁入口。
- 不把交通灯伪装成 `leadOne`、`leadTwo` 或任何物理前车。
- 不修改模型输入、Tesla 车辆状态、CAN 控制信号或 FCW 语义。
- 有物理前车、雷达无效、驾驶员踩刹车/油门或转向灯有效时继续 fail closed。
- 交通灯控制不改写基础 `longitudinalPlanSource`；是否介入由独立诊断字段表示。

### 3.2 本轮不得修改

- `longitudinal_backends/factory.py`
- Official、Experimental、TN-NoDEC planner 内部逻辑
- 三套 MPC 或其生成代码
- 基础 planner 的来源选择和 tuning 参数
- 物理 radar 数据及 lead 语义

## 4. 目标数据流

```text
Tesla CAN bus 2
  -> TeslaTrafficControlObserver
     - rawDecoded：原始帧是否可解码
     - eventContinuous：是否属于当前已确认事件
     - controlEligible：是否允许新建/更新控制事件
     - releaseEligible：是否允许释放同一已停车事件
  -> TrafficRadarSource / Controller
     - candidate
     - approach/braking
     - terminalStop
     - hold
     - release
  -> trafficRadarState
  -> FinalPlanArbitrator
     - stop constraint
     - terminal catch
     - latched hold
     - bounded start
  -> final longitudinalPlan
```

关键改变：把“原始帧可解码”“事件是否连续”“是否可新建控制事件”“是否可释放已有事件”
拆成不同概念，不能继续由一个 `validForControl` 同时承担全部语义。

## 5. 分阶段实施

### 阶段 A：先完善诊断和回放基线

修改范围：现有 trafficcontrold、FinalPlanArbitrator 日志及离线分析脚本；只补本次缺陷定位所必需的记录，不改变车辆输出。现有字段能表达就不改 schema；确有缺项才最小扩展并验证生产者/消费者。

按复现问题从以下诊断候选中选择，已有字段直接复用，不要求一次新增整表：

- `rawGreenSeen`
- `redEligible`
- `releaseEligible`
- `eventContinuous`
- `eventTransitionReason`
- `terminalCatchActive`
- `stopBlockReason`
- `rawDistance`
- `latchedRemainingDistance`
- `trafficRadarAgeMs`
- 现有 `action/applied/baseATarget/finalATarget/startBlockReason/eventId`

离线分析必须按 monotonic time 对齐：

```text
carState
trafficRadarState
longitudinalPlan
longitudinalPlanSP.teslaTrafficControl
radarState
modelV2.action
```

是否实际控制的唯一判据：

```text
applied == true
and action in {stop, hold, start, release}
and final plan differs from base plan when action requires a change
```

禁止再以 `longitudinalPlanSource=e2e` 推断交通灯没有介入。

交付物：能重新分析路线 `00000046--1653c1dc9e` 的逐事件报告。需要从日志设备导出原始
qlog/rlog；只有摘要不足以验证事件 4/5 的具体转移原因。

### 阶段 B：修复红灯末端停车

修改范围：`stop_profile.py`、`controller.py`、`final_plan_arbitrator.py`。

#### B1. 删除零距离零制动的早退

`remaining_distance <= 0.01m` 且车辆仍移动时，不得返回全零加速度。目标减速度继续按
有界有效距离计算：

```text
effectiveDistance = max(remainingDistance - actuatorCompensation, 0.5m)
targetAccel = clamp(-v² / (2 * effectiveDistance), -comfortBrake, 0)
```

仍然遵守最大舒适减速度和 jerk 限制；只有实际静止或明确 hold 时才输出零速度、零加速度
保持轨迹。

#### B2. 在现有状态中表达 terminalStop 语义

先检查现有 braking/hold 分支能否表达下列行为；仅在必要区别无法表达时增加局部状态，不重建 controller。

当满足以下任一条件时进入 terminalStop，而不是继续普通 braking：

- 剩余距离已经归零但 `vEgo >= 0.3m/s`；
- 按当前速度、执行器延迟和舒适减速度计算，已进入无法按普通 profile 留余量停车的区域；
- 预测下一控制周期将穿过内部停止点。

terminalStop 行为：

- 保持负加速度请求，直到 `vEgo < 0.1m/s`；
- `allowThrottle=false`；
- 在低速阈值内设置 `shouldStop=true`，进入固定停车策略；
- 不因距离被 clamp 为零而释放 profile；
- 只有同事件稳定绿灯或驾驶员覆盖才解除。

#### B3. 停车点安全余量

内部 remaining distance 应加入执行器延迟补偿和低速停车余量。首轮建议只使用常量和
现有 `longitudinalActuatorDelay`，不引入三套 planner 专属调参。

建议初始值仅作为封闭场地标定起点：

```text
terminal speed threshold: 1.5m/s
hold entry speed: 0.1m/s
minimum effective distance: 0.5m
comfort brake cap: 2.4m/s²（保持现值）
stop jerk cap: 0.8m/s³（保持现值）
```

### 阶段 C：稳定事件身份与连续性

修改范围：`tesla_observer.py`、`controller.py`、`radar_state.py`。

#### C1. 分离事件连续性和新事件控制资格

`featureState=0 + red + stateMachine=2` 仍只能用于确认新红灯事件；事件一旦确认，后续帧
只要满足下列条件即可刷新事件连续性：

- bus 仍为 2；
- controlType/controlSource 与事件兼容；
- 原始帧可解码且时间新鲜；
- 距离随车辆运动总体单调，创新量处于有界范围；
- 没有明确出现互斥的新目标证据。

不能因为 featureState 仍为 0、stateMachine 从 2 进入同一生命周期的其他状态，就立即把
整个事件当作控制数据丢失。

#### C2. 活动事件替换需要单独确认

当前 replacement 保护主要覆盖 candidate 阶段。修改后，已进入 approach/braking 的事件也
必须经过持续替代轨迹确认，才能生成新的 eventId。

- 单帧距离跳变不得替换活动事件；
- 短暂资格丢失不得替换活动事件；
- 相同 bus/controlSource 且运动连续时保持原 eventId；
- 真正新目标必须持续至少既定 replacement window；
- hold 事件绝不因普通观察超时自行释放。

#### C3. 消除 250ms/284ms 边界抖动

不要简单无限放宽 freshness。先根据路线分布确定正常发布周期，再选择同时满足以下条件的
门槛：

- 正常系统调度抖动不会频繁造成单周期释放；
- 真正失去 trafficcontrold 时仍能快速 fail closed；
- 已提交的 hold 使用本地 latch，不依赖每一帧继续到达。

建议评估 300–350ms，但最终值必须由多路线间隔分布决定，不能只依据一次最大值 284ms。

### 阶段 D：建立安全的绿色释放资格

修改范围：`tesla_observer.py`、`controller.py`、`radar_state.py`。

禁止允许任意 `featureState=0` 绿色创建起步事件。绿色只能释放同一个已经完成红停的事件。

releaseEligible 必须同时满足：

1. 当前 controller 已经 latch 同一非零 eventId；
2. 该事件曾进入 terminalStop 或 hold；
3. 绿色来自 bus 2；
4. controlType/controlSource 与已锁定事件一致；
5. 绿色距离与锁定事件预期距离一致；
6. 连续稳定至少 0.6 秒；
7. 原始帧新鲜且可解码；
8. 无物理前车，radar 有效；
9. 驾驶员未踩刹车、未踩油门、无转向灯；
10. 车辆仍处于低速/静止窗口；
11. 同 eventId 尚未执行过起步。

以下情况必须拒绝：

- 从未有对应红灯 hold 的独立绿色；
- distance=255 或无效距离；
- bus/controlSource 变化；
- 有物理前车或 radar 不确定；
- 绿灯稳定时间不足；
- 已经执行过同事件起步；
- 驾驶员覆盖。

### 阶段 E：按证据局部调整自动起步曲线（独立后续项）

修改范围：`final_plan_arbitrator.py`、`stop_profile.py`。

先回放目标版本的起步问题。仅当现有释放/起步逻辑确实无法满足已确认需求时，再在既有仲裁入口评估下列两阶段有界调整；不得改写三套 planner 或把候选参数直接作为上线值。

#### E1. 释放确认阶段

- 清除 traffic hold，但不立即给出高加速度；
- 若基础模型已解除 `shouldStop`，直接进入正常 bounded start；
- 若模型仍短暂保持 stop，只允许非常有限的 release grace，不得无限覆盖模型停车判断；
- grace 超时后模型仍拒绝起步，则保持停车并记录 `modelStop` 阻止原因。

#### E2. 有界起步阶段

建议封闭场地首轮标定范围：

```text
maximum start acceleration: 0.50–0.60m/s²
start jerk limit: 0.70–0.80m/s³
traffic start speed handoff: 2.0–2.5m/s
maximum traffic start duration: 2.5–3.0s
```

这些数值不是直接上线值。选择标准是：稳定绿灯后能明显起步，但不会产生突兀冲车，也不会
在错误绿色下移动较远。

起步仲裁仍不得覆盖：

- 物理前车；
- radar 无效；
- 驾驶员刹车；
- 转向灯；
- 非同一 held event；
- 已完成的 eventId；
- 超时后的持续模型 stop。

## 6. 测试方案

### 6.1 单元测试

必须先增加会失败的回归测试，再修改实现。

红灯停车：

- `distance=0m, vEgo=1.1m/s` 必须继续输出负加速度；
- `distance=0m, vEgo>0.3m/s` 不得输出普通 release；
- 进入 terminalStop 后，短暂交通消息丢失仍保持制动；
- 静止后输出 hold，`allowThrottle=false`；
- 驾驶员明确覆盖后解除 traffic hold；
- 最大减速度和 jerk 不超过配置门槛。

事件连续性：

- 同事件 `stateMachine 2 -> 3/4/5` 不生成新 eventId；
- 单帧 7–8m 量化跳变不替换事件；
- 持续替代轨迹经过 replacement window 才建立新事件；
- 250–350ms 调度抖动不解除已确认事件；
- 真正消息失联仍按设计退出非 hold 事件。

绿色释放：

- `featureState=0` 独立绿色不得起步；
- `featureState=0` 同一已 hold 事件的稳定绿色可以生成一次 start request；
- 不同 eventId、bus、controlSource 或距离不一致不得起步；
- 有前车、踩刹车、转向灯、无效 radar 时不得起步；
- 同 eventId 只能起步一次；
- `distance=255` 绿色永远无效。

起步曲线：

- 稳定绿灯后在规定时间内出现正加速度；
- 加速度和 jerk 不超过新门槛；
- 达到 handoff speed 后立即交还基础 planner；
- 超时、模型持续 stop 或安全门槛变化时停止自动起步。

### 6.2 三 planner 集成测试

对 Official、Experimental、TN-NoDEC 使用相同交通输入，验证：

- 三者都通过同一 FinalPlanArbitrator 介入；
- 三者内部 planner/MPC 状态不被修改；
- 开关关闭时输出与无 traffic 仲裁器逐字段一致；
- 开关开启时只修改最终发送的 plan；
- 基础 `longitudinalPlanSource` 保持原来源；
- `baseATarget/finalATarget/action/applied` 准确记录介入结果。

### 6.3 路线回放

最低回放集合：

- `00000046--1653c1dc9e`：零距离仍移动、事件 4/5、无绿色请求；
- 已有 traffic candidate jitter fixtures；
- 有物理前车的红灯路线；
- 正常完成红停再转绿的路线；
- 误绿、255 距离、bus/source 变化路线。

回放必须使用完整原始服务，不能只用摘要统计。

## 7. 验收门槛

### 7.1 离线门槛

- 所有三 planner 的 Traffic Off 输出透明测试通过；
- `distance=0 且 vEgo>0.3` 时不存在零制动早退；
- 同一连续事件不产生无依据的新 eventId；
- 独立绿色零误起步；
- 同一 held event 的合格绿色能产生且只产生一次 start request；
- 所有安全阻止原因都有可分析诊断字段。

### 7.2 离线回放与软件仿真门槛

优先复用多次真实红灯和红转绿事件的既有日志，缺少场景另记采样需求；在开发环境回放与仿真，要求：

- 预计终点不越过内部目标；
- terminal catch 不在正常远距离制动中频繁触发；
- event replacement 只对应真实新目标；
- releaseEligible 与人工逐帧标注一致；
- 假绿色、邻道灯、无对应 hold 的绿色起步请求为零；
- 前车存在时起步请求为零。

### 7.3 封闭场地门槛

红灯停车：

- 由低到高分级测试，不从道路速度直接开始；
- 内部目标点不得出现持续正速度穿越；
- 停车后稳定 hold，不自行释放；
- 最大减速度和 jerk 满足设定上限；
- 驾驶员刹车、油门覆盖始终有效。

绿灯起步：

- 只对同一 held event 的稳定绿色起步；
- 起步无明显冲击；
- 体感明显快于当前 0.35m/s² 曲线；
- 前车、刹车、转向灯、事件不匹配均能阻止起步；
- 达到 handoff 条件后基础 planner 平顺接管。

任何一次错误起步、无法保持停车、超过内部停止点或驾驶员覆盖失败，都阻止进入下一阶段。

## 8. 推荐提交顺序

每个提交保持单一目的，便于设备回退和路线二分：

1. `test(traffic): add terminal-stop and feature-state-zero regressions`
2. `feat(traffic): record event eligibility and final-plan diagnostics`
3. `fix(traffic): preserve braking after remaining distance reaches zero`
4. `fix(traffic): retain active event across compatible OEM state transitions`
5. `feat(traffic): qualify green only for the same held event`
6. `tune(traffic): add bounded two-stage green start profile`
7. `test(traffic): replay route 00000046 across all planner backends`

不得把全部修改压成一个大提交。

## 9. 部署与回滚

部署顺序：

1. 本地单元和集成测试；
2. 日志设备离线回放；
3. 多路线日志离线回放与软件仿真；
4. 封闭场地低速红停；
5. 封闭场地红停保持；
6. 封闭场地同事件绿灯起步；
7. 扩大速度范围前重新审查所有日志。

回滚保持简单：

- 在 Offroad 关闭唯一交通灯开关；
- 下一次 Onroad session 不创建 FinalPlanArbitrator；
- trafficcontrold 继续以 observe 模式采集；
- 三套基础纵向规划器恢复完全原始发布路径。

## 10. 最终实施判断

推荐实施，但不应一次性同时放开红停和更积极的绿灯起步。正确顺序是：

```text
诊断字段
  -> 红灯末端停车
  -> 事件连续性
  -> 绿色资格离线回放
  -> 同事件绿色释放
  -> 起步曲线调优
```

其中红灯末端停车和绿色资格是安全正确性问题，必须先于体感调优；提高起步加速度只能放在
绿色资格零误触和同事件约束已经验证之后。
