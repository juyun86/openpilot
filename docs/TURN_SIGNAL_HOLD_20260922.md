# 转向灯短暂断更保持

现场 13:36 左转灯反复开关对应 App 在 healthy 与源超时之间切换。App 失效报文 event=0，原 NavTurnSignalCoordinator 在进入既有 gap grace 前先判断 event 改变，直接 turnChanged 退出，导致保持逻辑未生效。

本次仅修改 navassist/lane_intent.py 与 navassist/nav_lane_intentd.py。对同 session、routeRevision、方向且仍为实时匹配导航的新鲜 inactive/event=0 报文，允许已有转弯灯进入有界保持，保留原 requestId 和对应灯光事件身份。保持最长 4 秒，容纳观测到的约 2 秒 SDK 更新、App 1 秒恢复确认及传递延迟。没有有效源时不能新开灯，不能产生变道准备许可。换路线、换方向、停止导航、重算、超时仍退出；原 60 秒总时限保留。原来的同事件消息短时失效使用原有 1.5 秒保持。

128 项测试通过，覆盖左右转／变道协调、导航速度控制、Tesla 灯光适配器、零事件中断、恢复身份保持、超时与明确停止。设备隔离测试目录 `/data/turn-signal-hold-2bcv4sks`，本地证据 `D:/C3/.codex-tmp/turn-signal-hold-20260922/`。

首轮部署在写文件前复查时发现辅助驾驶开启，已中止。用户随后确认司机已退出，复查 enabled=false、active=false、longActive=false；latActive 为横向常开标记，且当前没有导航灯光请求。仅替换上述两个文件，没有更换模型或重启横向／纵向控制。备份位于测试目录 backup/。

运行文件 SHA-256：lane_intent.py=`a60afe995e9d86056beb4e999c9b535c019b1522c2bbf5a9fd843da55990cef8`；nav_lane_intentd.py=`c70001eb9e3b6c286e134660def3b66528fd5f55df1033c57f238db67a35e7af`。停止旧灯光进程后，原管理器未自动重启普通 PythonProcess；随后设备断网，用户明确表示主动重启 C3。待设备恢复连接后核验启动。本修复不承诺在长时间导航失效后继续保持灯光。

重启后已核对上述两个运行文件哈希一致，管理器启动 nav_lane_intentd PID 49410，navLaneIntentSP seen/alive/valid 均为 true。现场最新导航为右转 35 m，但 App gpsWeak=true、event=0，尚不能新开灯；该状态不是保持修复的验收场景。恢复连接证据见 after-reboot.json。修复已部署生效，实际同一路口短时断更期间灯光连续性仍待道路观察，SDK 弱信号／长断更问题未解决。
