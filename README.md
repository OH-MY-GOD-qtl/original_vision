# 鼎行·双创战队 自瞄骨架

机器人视觉自瞄的工程骨架：相机、检测、解算、跟踪、瞄准、串口收发均已实现，开火判定与前馈待接。

链路、接口、配置、构建与标定工具都已就位。

## 当前状态

| 模块 | 状态 | 职责 |
|---|---|---|
| `Camera` | ✅ 已实现 | 海康相机取流，输出图像 + 曝光中点时刻 |
| `Detector` | ✅ 已实现 | 图像 → 装甲板像素角点（传统色差法，参数待真机标定） |
| `Solver` | ✅ 已实现 | 像素角点 → 世界系坐标（IPPE PnP + 手眼变换） |
| `Tracker` | ✅ 已实现 | 多帧装甲板 → 唯一目标（8 维旋转卡尔曼：圆心 + 转角） |
| `Aimer` | ✅ 已实现 | 预瞄点外推 + 重力补偿 + 端到端延迟补偿；开火判定未接 |
| `Gimbal` | ✅ 已实现 | 指令 ↔ 下位机串口（115200 8N1，CRC16） |

## 数据流

```
Camera ──Frame{img, t}──► Detector ──Armor──► Solver ──► Tracker ──Target──► Aimer ──Command──► Gimbal
```

`t` 是**曝光中点**，从相机一路传到跟踪，全链路不使用 `now()`。

云台姿态（相机光轴的世界 yaw/pitch）由 `Gimbal` 从串口回传，不再单独建 `Imu` 模块。

## 依赖

- C++17、CMake ≥ 3.16
- OpenCV 4.x、Eigen3、yaml-cpp
- 海康 MVS SDK（已放在 `third_party/`，无需额外安装）
- 相机：MV-CS016-10UC（USB3，1440×1080，BayerRG8）

## 构建

```bash
cmake -S . -B build
cmake --build build -j
```

## 运行

```bash
./build/camera_test configs/auto_aim.yaml                 # 相机取流并显示，按 q 退出
./build/detector_test configs/auto_aim.yaml               # 检测结果叠加到画面，按 q 退出
./build/solver_test configs/auto_aim.yaml                 # 解算结果：距离 / yaw / pitch，按 q 退出
./build/plot_test configs/auto_aim.yaml                   # 整链路数据推 PlotJuggler，按 q 退出
./build/tracker_test configs/auto_aim.yaml                # 跟踪可视化：绿框检测 / 深绿框滤波 / 红框红点击打位置，按 q 退出
./build/auto_aim_test configs/auto_aim.yaml               # 全链路：画面 + 推 PlotJuggler，红框红点击打位置，不发指令
./build/apps/auto_aim configs/auto_aim.yaml               # 总链路（开火判定未接，其余已通）
./build/calibration/timestamp_check configs/auto_aim.yaml # 时间戳链路自检
./build/calibration/detector_tuning configs/auto_aim.yaml # 检测参数标定，ESC 退出
./build/calibration/capture configs/auto_aim.yaml configs/calibration.yaml assets/calib # 采集标定图
./build/calibration/camera_calib configs/calibration.yaml assets/calib # 算内参，打印可粘贴的 yaml
```

## 目录

```
apps/          总链路
calibration/   标定工具，一个文件一个标定
configs/       配置，各模块读自己那一段
include/       头文件
src/           实现，与 include 同名
test/          真机测试
third_party/   海康 MVS SDK
```

## 设计要点与工程决策

这一节记录每处“为什么这么写”与各模块的设计要点；操作类说明见后文。

### 时间基准：全链路只用一条时间轴

- `Frame` 把图像与**曝光中点**绑在一起，`t` 从相机一路传到 `Tracker::track` 与 `Aimer::aim`，全链路不调用 `now()`
- 用**设备时钟**而不是主机时刻：设备戳单位 10ns、标记曝光开始，帧间隔恒定 6.047ms（165.37fps），不受主机调度影响
- 对时只在构造时做一次：预热 100 帧，取“到手时刻 − 设备时刻”最小的一帧当基准；首帧带着流建立开销，拿它对时会让之后每帧稳定偏晚数百微秒
- 时间戳 = `host_epoch + 设备时刻 + 曝光时长 / 2`，设备时钟零点到主机时刻的映射只维护这一处
- 不标定传输延迟：它和曝光时长共用一个观测量（帧到手时刻），拆不开；而 `frame.timestamp` 只参与帧间求差，常数偏移自动抵消
- `timestamp_check` 自检帧间隔、到手延迟与时钟漂移

### 坐标与角度：定义一次，与姿态模型精确互逆

- 云台系与相机系同构：$x$ 右、$y$ 下、$z$ 前；机械对准时手眼矩阵就是单位阵
- `yaw = atan2(x, z)` 正为向右；`pitch = asin(y / ‖p‖)` 正为向下，也就是云台仰角
- 姿态按 $R_y(\text{yaw})\,R_x(-\text{pitch})$ 构造，前进轴是 $(\sin\psi\cos\theta,\ \sin\theta,\ \cos\psi\cos\theta)$；`pitch` 与这个模型精确互逆，用 `atan2(y, z)` 会在 yaw 变大时偏小（6m 处 yaw 45°、仰角 10° 差 420mm）
- `direction_of` / `angles_of` 是唯一的换算入口，往返残差在浮点精度内；Aimer、绘图、测试工具不再各写一份三角函数
- 解算结果直接存**世界系**（$p_{cam} \to p_{pitch} \to p_{world}$），云台转动时静止目标坐标不变，跟踪器不会把云台转动误当成目标运动
- `Armor::yaw` 是板面法线的世界系偏航角，指向相机那一侧，也就是装甲板在圆上的方位
- 全链路不取负：目标在图像中心偏右对应 yaw 正，偏下对应 pitch 正

### 检测：色差 + 闭运算，算法只跑一份

- 敌方通道减对方通道过 `color_threshold`，再过灰度 `brightness_threshold`，两图取与得到灯条像素
- 按 `morph_size` 做一次闭运算，把点阵屏灯条的断点连成一片；闭运算不撑大外框，膨胀会把灯条两端各撑开半核，越远误差占比越大
- 轮廓取最小外接旋转矩形，先用 `min_lightbar_area` 滤碎点，再用 `min_lightbar_ratio` 滤不够细长的
- 配对过四关：高度相近（`min_height_ratio`）、倾角一致（`max_tilt_diff_deg`）、中心对齐（`max_center_offset`）、间距落在 `min_armor_ratio` ~ `max_armor_ratio` 之间
- 四步拆开暴露给调参工具，生产链路仍是一句 `detect()`；参数由 `DetectorParams` 定义一次

### 解算：IPPE + 手眼，输出世界系

- 模型点取灯条中轴线四端点，顺序左上→右上→右下→左下，板面按 `armor_tilt_deg` 绕自身宽度轴倾斜（本机 112°，即相对竖直后仰 22°），用 `SOLVEPNP_IPPE` 解位姿
- PnP 对模型的刚性旋转不变，倾角只决定画面上框的形状，位置解与板面方位角都不受影响
- 相机系 → 俯仰系走手眼 $R,t$；相机装在俯仰臂上，所以平移向量随俯仰转动
- 俯仰系 → 世界系走 $R_y(\text{yaw})\,R_x(-\text{pitch})$，静止目标的世界坐标不随云台转动变化
- 板面朝向取 PnP 旋转矩阵第三列再取负（模型 $+z$ 指向板内），转到世界系后取偏航角，正好是装甲板在圆上的方位
- `project()` 是 `solve()` 的逆变换（世界系 → 俯仰系 → 相机系 → 像素），只给可视化用，不作为精度依据
- 解算失败不抛异常，`Armor::solved` 保持 `false`，下游跳过

### 模块边界：共享类型头是唯一耦合面

- 构造函数只收 `config_path`，模块自己读配置
- 一个模块一个静态库，**target 名 = 文件名**
- 模块之间不互相 include，只依赖共享类型头：
  - `camera.hpp` 的 `Frame` —— Camera 产、全链路消费
  - `armor.hpp` —— Detector 产、Solver 补齐
  - `target.hpp` —— Tracker 产、Aimer 消费
  - `command.hpp` —— Aimer 产、Gimbal 消费
  - `angle.hpp` —— 角度约定 `direction_of` / `angles_of`，全项目共用
  - `viz.hpp` —— 画面配色与 `draw_quad`，只给 `test/` 用
- 图像与时间戳用 `Frame` 强绑定，不会配错
- 错误沿数据流走，不靠返回值：解算失败 `Armor::solved` 保持 `false`，连续丢失超过 `track_max_lost` 帧才返回 `nullopt`
- 相机 SDK 的头只在 `camera.cpp` 里包含（句柄用 `void*`）；调用方只依赖 `include/`，SDK 由 `camera` 目标私有链接
- 只有 `apps/auto_aim` 调 `Gimbal::send`；`test/` 与 `calibration/` 全部只读，调参不会误发指令

### 配置与算法：各只有一份

- `configs/auto_aim.yaml` 是唯一数据源：曝光、检测、内参、手眼、跟踪、瞄准、串口全在里面；模块不保留硬编码默认值，缺项或类型错误直接抛异常
- `configs/calibration.yaml` 只放棋盘格几何（内角点、方格边长），相机参数仍从总配置读
- `DetectorParams` 在 `detector.hpp` 定义一次：生产链路读它，`detector_tuning` 通过 `params()` / `set_params()` 复用同一套算法，不在工具里复制第二份
- 检测四步（`lightbar_mask` / `close` / `find_lightbars` / `pair`）单独暴露，只为让调参工具看中间结果；生产链路仍然是一句 `detect()`
- 标定拆成两步：`capture` 负责摆姿势采集（识别到角点才允许存图），`camera_calib` 负责反复重算，结果打印成可直接粘贴的 YAML
- 重复实现一律收口：四角求中心只有 `center_of` 一份，角度换算全在 `angle.hpp`，重力进 YAML

### 跟踪建模：圆心 + 转角，而不是自由板

- 装甲板固定在底盘四周的圆上，所以跟踪的是圆心与转角；半径取 `armor_radius`，板子尺寸固定所以写死，不参与估计
- 状态 8 维 `[cx cy cz vx vy vz theta omega]`，`theta` 是装甲板在圆上的方位，也是板面朝向；观测是装甲板世界坐标（3）+ 板面角度（1），位置对转角有耦合，雅可比里 $\partial h/\partial\theta = r$
- 换板不是“丢目标重新跟”：相邻两块夹角 `360 / armor_count`，转角用 `remainder` 折到同一分支，看到另一块时先把转角对齐，位置和角度一起跳，转速不受影响
- 预瞄沿圆周走 `target.armor(flight)`，自转目标的下一个击打点在圆上，而不是直线外推
- 局限：关联是最近邻，没有距离门限，两台机器人同框时可能跳板

预瞄点的计算就一行：

```cpp
target.armor(t) // 圆心 + 速度 * t + 半径 * (sin(theta + omega t), 0, cos(theta + omega t))
```

### 瞄准：沿圆周预瞄，重力补偿在最后

- 飞行时间先按当前位置粗算一轮，再用粗算预瞄点的距离修正一次，避免“越远飞得越久、飞得越久又更远”的循环
- 预瞄时刻是 `flight + latency_ms`：`Frame.t` 是**曝光中点**，从快门到云台真的指过去还有处理、串口、云台响应三段（约 10~15ms），目标在这段时间里又走了一段。只推 `flight` 会系统性落在运动目标的后方，静止靶看不出来
- `latency_ms` 视觉内部测不出来（`timestamp_check` 只量到帧到手，剩下的全在下游），靠打运动靶标定
- 重力下坠 $\frac{1}{2}gt^2$ 从 $y$ 减掉（$y$ 朝下，减 $y$ 即抬高），最后才转成世界系 yaw / pitch
- 输出绝对角与距离；`yaw_vel` / `yaw_acc` 等前馈字段已预留，目前为 0
- 无目标时先原位停留 `lost_hold_ms`，再按 `return_deg_per_s` 缓慢归零；首帧以当前云台姿态为原位，避免上电无目标直接下发 0,0 把云台拽向世界零点

### 通信：定长帧、非阻塞读、离线可跑

- 收发都是定长紧凑结构体（29 / 31 字节）并用 `static_assert` 锁长度；帧头 `'SP'`、小端、CRC16 查表
- 读是非阻塞的：一次排空缓冲区，只保留最新一帧，不拖视觉循环
- `Gimbal` 只管通信，不管弹道；收发的 yaw 都是相机光轴的世界航向，大小 yaw 的分配由电控合成
- `com_port` 留空即离线模式：不打开串口，`read()` 不更新姿态，`send()` 丢弃指令；没接电控也能跑检测、解算、跟踪可视化

### 可观测性：每个模块一个观察窗

- 六个测试工具按“流水线切在第 N 个模块”排列：`camera_test` → `detector_test` → `solver_test` → `plot_test` → `tracker_test` → `auto_aim_test`；越靠后包含越多上游模块
- 测试工具全部只读；`auto_aim_test` 把即将发出的 31 字节帧折算成 `tx/*` 键推给 PlotJuggler，配合 `err/*` 可以直接看“指令减云台”还差多少
- 画面配色统一语义：绿框本帧检测、深绿框滤波位置、红框红点击打位置、灰框检出但未解算；常量定义在 `viz.hpp`，四个工具不再各写一份
- 标定链同样按职责拆分：`timestamp_check` 看时间、`detector_tuning` 调检测、`capture` + `camera_calib` 标内参
- 因为配置与算法只有一份，观察窗里看到的就是总链路里实际跑的东西

### 代码风格与当前边界

- 最直白的直线代码，不为抽象而抽象；理想环境不加防御：SDK 返回码、空指针、单相机 `Finalize` 一律不兜底
- 单位在边界处转换：配置尽量用工程单位（ms、mm、度、m/s），模块内部统一用秒、米、弧度
- 注释一行一条贴代码，不留多行注释块；超过 100 列的行要收掉
- 已闭环：取流 → 检测 → 解算 → 跟踪 → 弹道 → 串口收发
- 未接：`yaw_vel` / `pitch_vel` 前馈（`Command` 字段已留）与开火判定

## 坐标与角度约定

云台系取 $x$ 右、$y$ 下、$z$ 前，与相机系同构，机械对准时手眼矩阵就是单位阵。

- `yaw = atan2(x, z)`，正为向右
- `pitch = asin(y / ‖p‖)`，正为向下，也就是云台的仰角（不是 `atan2(y, z)`）
- 装甲板模型点取灯条中轴线四端点，顺序左上→右上→右下→左下，板面按 `armor_tilt_deg` 相对竖直倾斜

`Armor::xyz` 与 `Target` 存**世界系**坐标，云台转动时静止目标的坐标不变。

`Armor::yaw` 是板面法线的世界系偏航角，指向相机那一侧，也就是装甲板在圆上的方位。

## 相机模块

- **像素格式**：显式固定 `BayerRG8`，转码用 `COLOR_BayerRG2RGB`（本机相位与 OpenCV 命名相反，用 `BayerRG2BGR` 会红蓝互换）
- **不做防御**：相机断连、取帧失败的返回码一律不检查（理想环境前提）

时间戳链路见「时间基准」。自检跑 `timestamp_check`：帧间隔是否恒为 6.047ms、到手延迟是否稳定、设备时钟相对主机漂多少 ppm。

## 检测参数标定

跑 `detector_tuning`：左侧窗口原图叠加绿框灯条与红线配对，右侧窗口是闭运算后的灯条二值图，
滑动条实时调全部检测参数，ESC 退出时打印可直接粘贴回 `auto_aim.yaml` 的配置。

工具调用的就是 `Detector` 本体，判定逻辑见「检测」，两边不会跑偏。

## 相机内参标定

采集和计算分成两个工具：采集要举着板子摆姿势，计算会反复跑。

1. 跑 `capture`，实时预览上叠加识别到的角点，**识别到才允许存图**。按 `s` 存一张，按 `q` 退出。
   第一个参数是相机参数（与总链路共用 `auto_aim.yaml`），第二个是棋盘格几何。
   摆 15~20 个姿势：覆盖画面四角与中心、远近各一档、板面绕两个轴各倾斜 20~40 度；
   别让角点贴边，避开反光和过曝。
2. 跑 `camera_calib`，同一个目录，逐张打印重投影误差，最后打印可直接粘进 `auto_aim.yaml` 的
   `camera_matrix` 与 `distort_coeffs`。某张误差明显偏大就删掉重算。

棋盘格几何只放在 `configs/calibration.yaml`：`pattern_cols` / `pattern_rows` 是**内角点**数
（格子数减一），`square_mm` 是方格边长。平均重投影误差 0.5 px 以内可用。

## 下位机通信

`Gimbal` 独占一个串口（默认 `/dev/ttyACM0`，115200 8N1），双向定长帧，小端，结构体紧凑无填充。

| 方向 | 长度 | 内容 |
|---|---|---|
| 电控 → 视觉 | 29 字节 | mode、相机光轴的世界 yaw/pitch 及角速度、弹速、id |
| 视觉 → 电控 | 31 字节 | mode、目标绝对 yaw/pitch 及一二阶导、shoot、is_empty |

- 帧头 `'S' 'P'`，CRC16 覆盖帧头到校验字段之前，小端存放
- CRC16：查表右移、反射多项式 `0x8408`、初值 `0xFFFF`、无最终异或（`"123456789"` → `0x6F91`）
- 读是**非阻塞**的：每次 `read()` 排空缓冲区并保留最新一帧，不拖视觉循环
- 收发两个方向的 `yaw` 都指**相机光轴的世界航向**，大小 yaw 的分配由电控负责
- 前馈四项（vel / acc）目前填 0；预瞄已经用上 `Target.velocity`，但发出去的角速度前馈还没算
- **`com_port` 留空即离线模式**：不打开串口，`read()` 不更新姿态（yaw/pitch 恒为 0）、
  `send()` 直接丢弃指令。没接电控时也能跑 `detector_test` / `solver_test` / `plot_test`

## 实时曲线

`plot_test` 把整条链路的量用 UDP JSON 推到 PlotJuggler，**只收不发，不会指挥云台**。

1. PlotJuggler 里选 **Streaming → UDP Server**，端口填 `9870`，格式选 **JSON**，点开始
2. 另开终端跑 `./build/plot_test configs/auto_aim.yaml`

键名里的 `/` 会被 PlotJuggler 自动分组：

| 键 | 含义 |
|---|---|
| `loop/fps` | 主循环帧率 |
| `detector/armors` | 检测到的装甲板数 |
| `solver/solved` | 是否有解算成功的板（0/1），用它判断目标相关字段该段是否有效 |
| `target/x` `y` `z` `dist` | 目标世界坐标与距离，m |
| `target/yaw_deg` `pitch_deg` | 目标方向，度 |
| `predict/dist` | 滤波后的距离，与 `target/dist` 对比可看出滤波效果 |
| `predict/theta_deg` | 装甲板在圆上的方位角，换板时会跳 `360 / armor_count` |
| `predict/omega_deg` | 估计的转速，度/秒 |
| `gimbal/yaw_deg` `pitch_deg` | 云台当前姿态，度 |
| `gimbal/yaw_vel_deg` `pitch_vel_deg` | 云台回传的角速度，度/秒 |
| `err/yaw_deg` `pitch_deg` | 目标减云台，度 |
| `gimbal/bullet_speed` | 回传弹速 |

没有解算结果时，目标相关字段填 0。

`auto_aim_test` 推同一个端口，键名多一组 `tx/`——**即将发给云台的帧内容**：

| 键 | 含义 |
|---|---|
| `tx/mode` / `tx/shoot` / `tx/is_empty` | 帧里的模式位与标志位 |
| `tx/yaw_deg` `tx/pitch_deg` `tx/dist` | 目标绝对角与距离 |
| `tx/yaw_vel_deg` `tx/yaw_acc_deg` `tx/pitch_vel_deg` `tx/pitch_acc_deg` | 前馈项，度制，目前恒 0 |

串口上下行都是弧度，**推出去的所有角量都折成度**：带 `_deg` 的是度，角速度是度/秒，
角加速度是度/秒²。

`err/*` 是 `tx` 减 `gimbal`，也就是云台还差多少。两个工具共用 9870 端口，不要同时开。

## 配置

`configs/auto_aim.yaml` 是**唯一一份相机参数**，也是总配置，各模块的参数都在里面；
`configs/calibration.yaml` 只放棋盘格几何，相机参数从总配置读。

## 待办

按顺序补齐：`Aimer` 的速度前馈（`yaw_vel` / `pitch_vel`）与开火判定；
`latency_ms` 的 12.0 是估值，要打运动靶标定一次。

