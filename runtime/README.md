# 树莓派轻量 Runtime

当前部署/验证状态见 [项目状态](../docs/project-status.md)，ROV共存与串口归属见 [系统架构](../docs/architecture/system.md)。ROV 模式使用 `config/pi-rov.yaml`：CSI下视、USB前视，Runtime串口为空、运动关闭；由独立auv-rov桥控制STM32。最新视频优化尚未完成实机部署，目标帧率不等于实测。

这个 C++ 进程与 ROS 节点共享 `auv_core`。它仅实现第一阶段任务：AprilTag、3×3 网格、锥形物分类、路径规划和网格遍历。STM32 继续负责姿态/深度 PID、混合器控制以及硬件心跳故障保护。

`mission.profile` 默认为 `task_one`。共享Mission FSM已经定义完整比赛阶段；设置为`full`后，四锥阶段会继续进入海参、抓取、运输、释放、转盘、返航和上浮流程。但在前视视觉和后续运动控制接通前，完整模式会按阶段超时进入FAULT，不能视为可下水的完整任务配置。

`task_one` 建图/规划完成、进入遍历前会先上浮：识别到 AprilTag 后运行时自动开启上浮（`motion.surface_before_traversal` 默认 `false`，仅 `task_one` 自动置 true）。ARM 后先把深度保持到 `motion.surface_depth_m`（默认 0.5m，容差 `surface_tolerance_m`，稳定 `surface_stable_sec`），期间无水平运动；触发 `SURFACED` 事件后开始遍历并保持该深度。显式把 `surface_before_traversal` 设为 `true` 可无条件开启（例如 `full` 模式）；上浮超时 `surface_timeout_sec` 会进入 FAULT 并请求 DISARM。

遍历与离场期间启用深度保持看门狗：深度偏离 `motion.surface_depth_m` 超过 `depth_deadzone_m`（默认 5cm）时暂停水平运动、保持深度目标，由 STM32 depth PID 把深度带回死区内；在带内持续 `depth_recover_stable_sec`（默认 0.5s）后恢复运动，暂停超过 `depth_pause_timeout_sec`（默认 10s）则 FAULT 并请求 DISARM。遍历完所有交通锥后，按最后锥所在格就近离开九宫格（边/角格走最近出格方向，中间格转向无锥的边中点方向），以 `depart_speed` 盲走 `depart_duration_sec`（默认 0.08 / 2.0s）后进入 `COMPLETE` 并 DISARM；离场同样受深度看门狗覆盖。

## 原生构建

在 Debian 13 / Raspberry Pi OS 上，确保已安装 `cmake`、`ninja-build`、`g++`、`libopencv-dev`、`libyaml-cpp-dev`、`libcpp-httplib-dev` 和 `ffmpeg`，然后执行：

```sh
cmake -S . -B build-lightweight -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-lightweight -j 3
ctest --test-dir build-lightweight --output-on-failure
```

原生 CTest 覆盖核心地图/规划/状态校验，以及 PTY 串口故障测试。如果系统 Python 也已安装 OpenCV 和 NumPy，还会额外加入合成的 AprilTag / 网格 / 锥形物视频回放，以及虚拟 STM32 的 ARM / ACK / 限制 / 进程退出测试。这些测试仅使用伪终端，不会访问真实串口。

复制并编辑 `runtime/config/runtime.yaml`。示例配置为 `debug` 模式并默认拒绝运动：`motion_commands_enabled: false`，串口设备为空，控制方向和相机内参均未校准。生产环境中的相机源必须使用稳定的 `/dev/v4l/by-id/...` 符号链接。`file:/absolute/path/video.mp4` 可用于离线回放，但若未完成标定，运动仍然保持禁用状态。调试模式启动时处于 INIT 和 DISARM 状态，并需要执行 `auvctl start`；它不会在故障后自动恢复运动。

```sh
./build-lightweight/runtime/auv_runtime runtime/config/runtime.yaml
./build-lightweight/runtime/auvctl status
./build-lightweight/runtime/auvctl start
./build-lightweight/runtime/auvctl disarm
```

`auvctl` 通过 `/run/auv-runtime/control.sock`（模式 `0660`）进行控制，适合通过 SSH 使用。SSH 用户必须属于 `auv` 组。执行 ARM 还需要满足额外条件：`auvctl arm --confirm SAFE_TO_ARM`，并且必须具备新鲜且安全的 STM32 STATUS、VISIT_CONES、已校准的运动配置，以及显式启用的运动控制。`pause`、`abort`、`disarm` 和故障条件都会撤销运动并请求 DISARM。若 Linux 异常退出，STM32 心跳丢失仍然是最终的安全屏障。

## 双摄像头

参考补充代码 `two_camera.py` / `two_camera_fast.py` 的独立采集、最新帧覆盖和 CSI MJPEG 管道方式，运行时可同时采集 USB 与 CSI。操作者确认CSI为下视、USB为前视。CSI下视帧输入AprilTag、九宫格及交通锥视觉；USB前视独立采集与预览，尚未接入海参或转盘识别。

下视使用 `camera.source: csi:0`（或 `csi:1`）；启用 `camera_front.enabled: true`，前视使用实际USB `/dev/v4l/by-id/...`设备，并配置宽、高和帧率；本地待部署Pi模板目标 320×240、30 fps。CSI 需要系统提供 `rpicam-vid`。默认配置关闭第二相机，兼容单相机与离线测试。

两路独立采集，只保留最新帧；CSI 使用有界 MJPEG 缓冲、超时读取、进程重启和退出清理，避免CSI堵塞拖住USB前视或控制；下视过期仍触发原有任务视觉保护。USB 使用 V4L2 单帧缓冲。默认网页双路MJPEG独立预览；HLS兼容预览固定节奏直接取采集最新帧，两路共用一个 H.264 编码器，输出 640×240，左侧CSI下视、右侧USB前视。超过帧时限的画面置黑。

CSI 启动等待首帧最多 8 秒，连续采集后无帧 2 秒则重启该相机进程。服务等待 `network-online.target` 后启动，避免开机静态 IP 尚未就绪导致网页绑定失败。实际 Pi 的硬件编码存在后续 HLS 分段缺少 SPS/PPS 的问题，当前部署显式选择 `libx264`、`ultrafast`、单线程、15 fps；硬件编码 HLS 尚未验收，不能只凭 HTTP 200 判定视频可播放。软件编码添加关键帧参数头，机制参考 [FFmpeg bitstream filter 文档](https://ffmpeg.org/ffmpeg-bitstream-filters.html#dump_005fextra)，实机需通过后续分段独立解码检查。

只读接口 `/api/camera/down.jpg` 和 `/api/camera/front.jpg` 返回各自最新 JPEG；过期时返回 HTTP 503。`/api/status` 增加 `down_capture_frames`、`front_frames`、`front_hz`、`front_camera_age_sec`、`front_degraded` 和 `front_detail`。采集帧率、视觉处理帧率与视频输出帧率分别统计，不能相互代替。

`ctest --test-dir build-lightweight --output-on-failure` 包含 MJPEG 分包/缓冲上限测试，以及双相机模拟、CSI分别作为下视/前视时卡住后的另一相机及控制持续运行、子进程退出清理测试；这些测试不访问真实硬件或串口。

## 待办：前视建图 + 下视定位重构（规划中，未实现）

目标：由**前视摄像头**先完成初步建图与遍历路径规划；再由**下视摄像头**观测当前所在格点，实现路径记录（`visited` 标记）与当前格点判定。本节记录任务、风险点与涉及代码的规划，当前尚未落地。本节生效后，上方「双摄像头」章节描述的相机职责（下视建图、前视仅预览）将随之变更。

### 目标职责划分

| 摄像头 | 阶段 | 职责 | 产出 |
| --- | --- | --- | --- |
| 前视 | 建图 / 规划 | 建图（GridMapper + ConeDetector/Tracker）、建图期位姿，无 AprilTag | `map_`、前视位姿、`video_frame_`（建图视角） |
| 下视 | 全阶段 | AprilTag 触发、观测当前格点（→ 路径记录 + 格点判定）、锥体分类修正 | `tag_found_`、下视位姿、修正结果 |

### 任务清单

1. `vision_loop` 改造为前视建图循环：消费 `front_frame_/front_sequence_/front_time_`，用前视内参 undistort，去掉 AprilTag，`map_` complete 后冻结（`if (!map_.complete) map_ = map`）。
2. `front_loop` 增加 `front_sequence_` 计数（供视觉线程按新帧去重）。
3. 新增 `down_vision_loop`：AprilTag → `tag_found_`；GridMapper → 下视位姿（`down_row_/down_col_`）；ConeDetector/Tracker → 修正。
4. 位姿按相位切换：`≤kPlanCones` 取前视位姿，`kVisitCones` 及以后取下视位姿（per-camera 位姿字段 + `active_pose(phase)`）。
5. 下视修正（下视 wins、改地图不改路由）：分类不一致改 `object_type`；存在性不一致（前视漏锥）补入地图 cell；两种情况都不重规划、不碰 `plan_`/`route_`。
6. `camera_front` 增加 `camera_matrix`/`distortion_coefficients`（additive，默认空）；`enabled` 默认 `false`，测试显式 `true`。
7. 保留单 `safety.frame_timeout_sec`（不做双超时，避免改动部署模板与 `pi-rov.yaml`）。

### 风险点（按严重程度排序）

1. **前视漏锥不可自愈**：路线在建图/规划期冻结，前视因遮挡/畸变漏判的锥不会被排进路线，物理上漏访；下视事后只能改地图、改不了路线。
2. **双相机网格坐标系一致性**：前视与下视的 `camera_row/col` 必须指向同一物理格子；镜像/旋转会导致修正写错 cell、位姿切换跳变。
3. **前视单点故障**：建图全压前视，前视失效 → `BUILD_MAP` 走 `map_timeout_sec` FAULT，无下视兜底建图。
4. **计算量翻倍**：grid+cone 检测跑两遍（前视 + 下视），Pi 4B 可能掉帧 → 误报 `frame_timeout` FAULT。
5. **位姿相位切换跳变**：PLAN 用前视定起始格，VISIT_CONES 用下视定位，两者不一致则遍历起点错。
6. **回归风险**：深度看门狗/离场逻辑读全局位姿，重构为 per-camera 位姿后必须保留离场（`kDeparting`）的 pose 超时豁免。
7. **验证缺口**：单静止图 harness 无法覆盖相位切换与修正，需移动相机模拟（第二阶段）才能真正验证。

### 涉及代码（规划）

- `runtime/config/runtime.yaml`：`camera_front` 增加内参 key。
- `runtime/src/runtime.cpp`：
  - `Config` / `load_config`：前视内参解析与校验。
  - `front_loop`：加 `front_sequence_`。
  - `vision_loop`：换输入/内参、去 tag、冻结 `map_`、写前视位姿。
  - 新增 `down_vision_loop`。
  - 位姿切换：per-camera 位姿 + `active_pose(phase)`，替换 `arm_gate_ready` / pose 超时 / `route_.set_pose` 的位姿读取。
  - 修正 helper：改 `map_.grid.cells[i].cell.object_type`，不碰 `plan_` / `route_`。
- `runtime/test_scripts/harness.py` 及测试：双路相机注入、断言重写（`apriltag_found` 来自下视、`map_` 来自前视、`VISIT_CONES` 位姿来自下视）。

## 自主模式

完成相机、控制方向、速度限制和真实 STM32 安全验收后，比赛配置可以设置 `operation.mode: autonomous`、`auto_start: true` 和 `auto_arm: true`。自主模式会先保持 DISARM，等待启动延时，并要求相机采集、视觉处理与安全 STM32 STATUS 在 `startup_stable_sec` 内持续新鲜，然后自动启动 Mission；进入 `VISIT_CONES` 且完整 ARM 安全门仍满足时，才自动发送一次 ARM 请求。自主模式拒绝远程 `start`、`arm`、`pause`、`resume`、`abort` 和 `reset`，只保留只读 `status` 与紧急 `disarm`。紧急 DISARM 会把任务锁定到 FAULT。

STM32 heartbeat还受独立的控制循环看门狗约束。若控制循环超过 `safety.control_watchdog_timeout_sec` 未完成一次周期，即使串口线程仍存活，也会停止heartbeat、请求DISARM并等待STM32自身的heartbeat failsafe生效。这避免“通信线程健康但控制决策已经卡死”时维持危险输出。

每次系统上电最多允许一次自主任务启动。首次自动 START 会建立 `/run/auv-runtime/control.sock.autonomous-started` 锁存；服务崩溃或被 systemd 重启后不会再次自动开始或 ARM。锁存只在整机重启后清除。不要手工删除锁存来绕过现场安全流程。

自主模式不会降低任何运动标定要求，且 `auto_arm: true` 必须同时启用经过标定的运动配置。仓库默认配置始终保持调试模式和运动禁用。

完整任务模式还要求持续有效且已标定的夹爪遥测，否则不会通过自主启动就绪门。运行时在`GRAB`发送一次关闭命令、在`RELEASE`发送一次打开命令，并以STM32的`CLOSED/OPENED`状态作为Mission确认。暂停、终止、DISARM、FAULT和进程退出都会发送夹爪STOP；非ARM状态不会执行抓取或释放。

网页地址为 `http://192.168.137.201:8080/`，它是只读页面。`hls.js` 已本地打包。视频采用 FFmpeg 的 `h264_v4l2m2m` 编码，分辨率为 640×480，20 fps，码率 2 Mbit/s，HLS 分段长度为 0.5 s。若编码失败，状态会报告视频降级。软件编码 `libx264` 需要开启 `video.software_fallback_enabled: true`，或者显式修改 `video.encoder: libx264`。若构建时缺少 cpp-httplib，或网络地址不可用，HTTP 也会被降级处理。上述任一失败都不会中断任务控制。NDJSON 会记录带时间戳的事件、路径和周期性状态，并按大小或日期自动轮转。已完成的网格调试帧保存在 `logging.debug_dir`。

## 部署

在树莓派上检出代码后，执行 `runtime/deploy/install_pi.sh`。它会安装依赖、构建项目、运行测试、安装服务，并以 DISARMED 状态启动。现有的 `/etc/auv-runtime/runtime.yaml` 会被保留。检查状态可用：`systemctl status auv-runtime`、`journalctl -u auv-runtime` 和 `/var/log/auv-runtime/events.ndjson`。远程访问请使用 SSH 密钥；本仓库不包含任何密码处理逻辑。

从 PC 端执行：`runtime/deploy/deploy_from_pc.sh pi-user@192.168.137.201 /home/pi/auv`，它会通过 SSH 拷贝当前代码并执行 Pi 安装脚本。脚本禁用了 SSH 密码登录；远程 `sudo` 可能会通过终端提示。

请参考 [deploy/TESTING.md](deploy/TESTING.md) 中分阶段、只读的 Pi 检查流程。`run_bench.sh preflight` 会验证部署安全性，并将缺失的硬件标记为 pending；`camera`、`serial`、`fault-watch`、`endurance` 和 `collect` 提供聚焦检查和保存的 JSON 报告。耐久测试脚本会调用 `acceptance.py`，测量视觉/控制/心跳速率、滚动帧延迟、进程树 CPU 和 RSS、温度及节流状态，并要求具备有效的相机和 STM32 状态。缺失节流数据不计入通过。

在校准相机、行/列到机体坐标系的符号方向、速度限制和串口设备之前，不要设置 `motion_commands_enabled: true`。首次运动测试必须断开推进器电源，移除螺旋桨，或确保推进器牢固固定。2026-10-04 日，Debian 13 原生构建和两次 Pi CTest 已通过。30 分钟热性能运行、真实相机/HLS 测试、真实 STM32 试验台运行，以及无螺旋桨闭环验收仍待硬件完成。

当前硬件无漏水检测；固件和任务判断已移除此分支。状态协议中的旧漏水状态位保留且不使用，ROS Stm32Status不再含该字段，需要重新编译相关ROS包。急停、传感器有效性和通信超时保护保留。新版页面为只读任务中心，控制仍使用auvctl。


## 回传与训练录像帧率

`/api/camera/down.mjpeg`、`/api/camera/front.mjpeg`提供multipart长连接；每帧包含Content-Length、X-Frame-Time-Monotonic、X-Camera-Source。CSI直接复用rpicam MJPEG字节；USB采集后只编码一次。JPEG快照接口同样复用缓存。图像和时间标记在同一把锁中发布，时间是Pi收到/解码该帧时的单调时钟，不是相机硬件曝光时间，不代表硬件同步双摄。

`/api/status`新增down_hz、video_enabled；down_hz/front_hz为采集间隔估计，vision_hz仅为识别处理速度。PC录制显示received_fps，且按来源时间戳去重，不把重复帧计作新采集帧。目标30fps和实际稳定帧率应分别核对。原有HLS历史实测配置仍为15fps软件编码，本次模板调整尚未部署；不能把本地模板或模拟测试结果当作实机测量。

长连接实现使用cpp-httplib的[chunked content provider](https://github.com/yhirose/cpp-httplib#chunked-transfer-encoding)，整帧一次写入并启用TCP_NODELAY。最多4条流、8个HTTP工作线程，慢客户端只读取最新缓存，写入超时2秒；不占用控制循环或视觉处理线程。无新帧2秒关闭流，网页下一秒尝试重连；录制读取失败重连并保留已有视频，持续失败停止并收尾。树莓派资源紧张而不需要HLS时可将video.enabled设为false，MJPEG与录像仍可用；保持web.enabled为true。

固件改为岸上手动水平基准，STM32重启后需要通过ROV驾驶台显式完成校准再请求ARM。未完成校准的自动任务ARM也会被底层安全门拒绝。此版本不要在水下姿态不明确时校零。


Pi相机专用配置现保存在`runtime/config/pi-rov.yaml`，并随安装复制到配置目录；默认两摄320×240、30fps，MJPEG开启，HLS编码关闭以减轻CPU负担。此文件不会自动覆盖既有运行配置。部署后应先只读测速和检查录制片段，再启用运动。
