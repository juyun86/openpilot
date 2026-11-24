# C3XL openpilot / NavAssist 源码快照（2026-11-24）

快照以 `navassist-jetlink-tesla` 的 `6db98169cfefd6e7e4bd1f3bdaaf70623f79750d` 为基线，从本地开发目录收录 NavAssist、导航模型输入和配套 GPS 源码与测试。该分支用于交接和复现本地候选。

## 包内内容

- NavAssist v3 完整路线接收、路线缓存与扩展 UDP 回执。完整路线通过 HTTP 上传，UDP 继续承载导航快照和回执。[路线及模型输入说明](NAV_MODEL_INPUT.md)
- 可选的路线底图、导航指令向量和两条 `modeld` 路径的输入适配。渲染默认关闭，模型需声明 `nav_map` 和 `navigation` 才使用输入。本次交接没有模型权重或底图数据。
- C3XL/Tesla 内置 GPS 发布端的经纬度替换及测试。仅当原 GPS 消息有效且原车坐标和时钟新鲜时替换经纬度。10 月 23 日的经纬度替换版本有停车安装和回读记录，正常行车发布流以及整份快照仍待车端验证。[坐标转换许可](COORDTRANSFORM_LICENSE.txt)

## 验证

在快照检出目录运行路线、协议、车辆定位、底图和模型入口相关测试，138 项通过，1 项因依赖 Linux 硬件模块在 Windows 跳过。发布器、HTTP 服务及 UDP 接收测试还需 Linux 的 `fcntl` 和原生 `msgq` 模块。本次完成源码检查和离线测试，整机编译、设备安装及道路验证待做。

特会开此前的 Android 源码快照与 11 月平板实机包有版本差异。跨端测试需记录 App 和 C3XL 两边各自的版本。[特会开交接总结](https://github.com/juyun86/TesNav/blob/74a5badce541773945b08ef6d181bf6d658ec6ea/docs/TENENGKAI_STATUS_20261124.md)
