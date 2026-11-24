# 高德 API 导航模型输入（本地候选）

已接通原始路线传输、底图渲染、IPC 和两套现有 `modeld` 的可选输入入口。
不改变纵向规划、灯光、转弯许可或横向控制。没有替换模型权重或部署到车机，
不能据此称官方导航模型已启用。两个仓库均在本地分支 `comma-nav-model-input-20261013`。

## 路线传输

App 在现有高德 API 路线回调中保存原始 `coordList`，不复用旧 WebSocket 的显示简化路线。
只有当前选择 API 来源、`acceptedPathId` 和 `routeRevision` 一致时才提供路线。
实时快照被 C3 确认后，通过原有 HTTP 端口的 `POST /v3/route` 独立上传，
不占用快照发送锁，不增加进程或端口。车机版高德来源没有完整 API 路线，因此不提供该输入。

JSON 必须恰好包含 `schemaVersion:3`、`messageType:"navigation_route"`、
`sessionId`、`sequence`、`routeRevision`、`sourceWallTimeMs`、`coordinateSystem`、`points`。
`points` 为包含 `latitude` 和 `longitude` 的对象数组。
单条路线限制为 20,000 点、2 MiB（接收资源上限）；超限拒绝，不静默简化。

签名 HTTP 使用现有配对身份，签名绑定 `/v3/route` 路径。
运行中的 UDP 模式沿用当前快照来源 IP 的数据所有权，不把它当成设备认证或控制许可。
带错误签名的请求不能回退到 UDP 身份。接收端在同一锁内检查来源、会话、版本、序号和时间。
只保留一条内存路线；序号必须不晚于已接受快照且相对上次路线递增。
同版本不能更换几何；切换来源/会话、版本变更、导航退出或快照中断后清除。

扩展 UDP 回执带 `modelRouteRevision`，缓存缺失为 -1，原回执仍发送以兼容旧客户端。
App 根据回执补传缺失路线，失败最多每 5 秒重试；新路线不受旧路线重试等待影响。
签名 HTTP 或旧回执无缓存字段时，每 30 秒重验一次以恢复接收端重启后的内存缓存。
所有上传依附实时快照，无独立后台轮询。停止 exporter 后取消任务并关闭连接。

## 模型数据接口

`NavAssistStore.model_input()` 返回原始路线、同坐标系的位置/航向和 47 维 `navigation` 元组。
路线与当前快照必须同会话、同版本、同坐标系；要求实时导航、路线匹配、非弱 GPS，
快照及位置/引导原始观测都在有效期内。条件不满足返回 `None`。
缺失距离不能编码为零距离转弯；未知、汇入、到达等未支持指令返回全零向量。

指令编码依据 comma 官方
[helpers.py（a03bb1d4）](https://github.com/commaai/openpilot/blob/a03bb1d4e80050cae0007780ecb8eb7cf06d0cb9/openpilot/selfdrive/navigation/helpers.py)：
46 个 Valhalla 指令槽，第 47 个值为距离米数 / 100。
这里输出 Python 浮点元组，模型入口转为 float32 张量。

## 地图与模型入口

复用上述官方版本的 `renderer.py`、`pmtiles.py`、`vector_tile.py`、`route.py` 和 `tiles.py`。
线上读取只适配本仓库 `Api` 的 DongleId 接口，并限制缓存版本变化时的文件清理范围。
地图为 256×256 uint8、1 米/像素，车位像素 (128,192)，道路值 255、路线值 128；
按官方路线切线确定朝向，并依据原始位置年龄和速度补偿进度。
GCJ-02 路线及位置统一逆变换到 WGS-84，算法来源和许可见
[COORDTRANSFORM_LICENSE.txt](COORDTRANSFORM_LICENSE.txt)。数值反算通过不等于实际道路对齐已验证。

沿用 `navassistd` 的发布循环，增加一个按需启动、最多一项渲染任务的线程；
网络或渲染等待不阻塞快照接收与发布。路线退出/换版本后清除缓存，旧任务返回不能复活旧地图；
底图不存在、渲染无有效道路或超过 500ms 时不发布模型输入。失败后按 10 秒退避。
没有新增 mapsd/navd 进程或端口。默认不启动渲染；配置 `NAV_MODEL_MAP_ARCHIVE` 指向本地
PMTiles 文件，或明确设置 `NAV_MODEL_ONLINE_MAP=1` 使用官方设备地图接口后才渲染。
本地未找到可用底图库，官方设备接口是否可访问仍待实机核验。

`NavAssistStateSP` 末尾追加 `modelMap`、`modelNavigation`、`modelMapMonoTime`、`modelInputValid`。
旧客户端默认读到空值；模型只有声明 `nav_map` 和 `navigation` 两个输入时才消费。
要求现有消息健康、横向控制激活、无司机刹车/油门/方向盘接管。
非直行指令还要求 `DesireHelper.turn_maneuver.state == 'active'`，即通过现有 TurnEntryGate；
地图与指令同时被该门约束，灯光本身不授予许可。匝道、出口和分叉也按此门约束，
不自行放宽为变道许可。每次推理先清零可选输入，失效后不保留上一帧。

旧模型 ABI 保持原路径；旧编译路径仅增加可选 `nav_map/navigation` 打包。
stock `compile_modeld.py` 检测到 `new_img` 时选择新格式编译分支，
保存 `input_specs/output_specs` 主包和各相机独立 warp；不把新模型强行转换为旧 ABI。
新分支沿用 `make_warp`，实际图像形状必须等于 ONNX `new_img`，否则拒绝编译。
所有预测先生成再写入 next-state 输出，避免写回状态影响同次推理的其它结果。
新 `input_specs/output_specs` 包走独立的 `NavigationRuntime`，
适配本仓库 Buffer.view 的元素数/类型/字节偏移，保留 GPU 递归状态及 next-state 输出别名。
主模型沿用本仓库 OOB 加载格式，独立 `driving_warp_<WxH>_tinygrad.pkl` 按官方普通 pickle 加载，
Chestnut 文件前缀为 `big_`，warp 文件放在主包旁边。编译产物必须匹配本仓库 tinygrad，
不能假定不同 tinygrad 版本的官方 pickle 可以直接运行。

## 验证与剩余工作

本地 83 项协议/地图/运行适配测试通过，1 项 HTTP 集成测试因 Windows 缺少 `fcntl` 跳过；
单独收集 UDP 原生测试也受同一依赖阻断，不记为通过。
地图验证实际解析了 PMTiles/MVT 测试归档，并检验比例、朝向和无道路拒绝；
IPC 使用真实 Capnp 序列化。运行验证使用真实 tinygrad PYTHON 后端 TinyJit 及 pickle 往返，
覆盖递归更新、地图/指令输入、失效清零和重置。另有真实 ONNX 小图从 OnnxRunner 编译、
pickle 往返至 NavigationRuntime 的验证；测试模型是小图，未冒充真实驾驶模型推理。
App 上一轮两版本构建通过，303 项单测中 302 项通过、1 项原有跳过；本轮未改 App。

尚需取得官方导航 ONNX 权重，用本仓库 tinygrad 在目标设备编译匹配的主包及相机 warp，
核验真实输出解析、eGPU/小模型回退和 20Hz 时延；Windows 未执行整个硬件依赖的编译 CLI，
真实相机 warp 及主包 OOB 装载仍需目标平台验证。没有改默认构建模型或自动安装这些产物。
还需核验本车地图接口或本地底图库、中国道路覆盖、GCJ-02 实际对齐、完整原生 HTTP/UDP/IPC，
然后进行获授权的停车部署及道路验收。没有下载驾驶权重、切换生产模型、修改车机参数或主动发 CAN/GPIO。
