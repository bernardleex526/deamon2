# 山猫 M20 Pro 人体跟随：双雷达融合点云 vs 仅前雷达 选型分析

分析对象：`deamon2`（上游 `6-robot/jie_deamon`）在 M20 Pro GOS 上的「雷达跟随」链路
（`/LIDAR/POINTS` → `pointcloud_to_laserscan` → `/m20/scan` → 跟随算法 → `/m20/cmd_vel_raw` → 门控 → AOS UDP）。

分析日期：基于本机素材（`D:\repos\dt_text\*`、`D:\repos\deamon2-src\deamon2-main\*`）。**未联网**，未修改任何输入文件。

---

## 一、结论

1. **推荐：继续使用厂商融合点云 `/LIDAR/POINTS`（`send_separately: false` 不动），但在用户侧做「前向扇区加权 + 自车几何遮罩」**——即 360° 全量进算法，但**停止区判定、目标质心、APF 三项只吃前向 200° 扇区**，并把自车遮罩扩大到覆盖两个雷达自身位置（x≈±0.32 m）。
2. **不要为了「只要前雷达」去改 `send_separately`**：文档明确该开关会让「导航、定位功能将无法使用，自主充电功能部分受限」，且需要厂商确认 GOS 单独改是否会破坏原厂链路——收益（省带宽）与代价（丢掉原厂功能 + 需厂商支持）不成比例。
3. **纯「仅前雷达」在工程上做不到零成本**：融合点云里前后雷达的 `ring` 字段是否可区分**文档未给出，需现场实测**；`pointcloud_to_laserscan` 只能按高度/角度/距离过滤，没有 `ring` 过滤能力。等价且零成本的替代是**按角度扇区过滤**（`angle_min/angle_max` 收到 ±100°），其几何效果≈「只用前雷达看到的前向区域」。
4. **后雷达对「人」这个目标的跟随增益很小**：当前算法没有「人在身后掉头跟」的行为（目标搜索以 `target=(follow_distance, 0)` 为中心的 0.3 m 半径圆），后雷达的真实贡献只有「侧后方覆盖」和「后方避障」，而后者在 2D 单层 + 后向停止区配置下**反而贡献了 81/150 帧的误停**（实测最近回波 0.45 m 在「后左」，来源未定，自车遮罩不足是更可能的成因）。
5. **因此结论是「融合但前向加权」，不是「只用前雷达」**：保留融合链路（不改原厂、不改话题、故障域简单），但把算法与门控的有效视场收窄到前向扇区，并修掉自车回波。

---

## 二、事实依据（逐条引用文档原文 + 出处）

### 2.1 雷达数量、型号、安装位置

| 事实 | 原文 | 出处 |
| --- | --- | --- |
| 前后双 96 线 | 「山猫 M20 配备前后双 96 线激光雷达，用于环境感知、建图与导航避障。」 | `lidar.txt` §1 概述 |
| 类型/数量 | 「类型 96 线激光雷达」「数量 2（前置×1 + 后置×1）」 | `lidar.txt` §2 硬件参数 |
| 独立 IP | 「前雷达 10.21.33.201，后雷达 10.21.33.202」 | `lidar.txt` §2 |
| 量程 | 「探测距离 0.2 m ~ 60 m」 | `lidar.txt` §2 |
| 静态外参 | 前激光雷达 `x=0.32028, y=0, z=-0.013, roll=0`；后激光雷达 `x=-0.32028, y=0, z=-0.013, roll=0` | `lidar.txt` §3 静态外参 |
| 传感器坐标（mm） | 前激光雷达 `320.28 / 0 / -13`；后激光雷达 `-320.28 / 0 / -13`（相对身体坐标系） | `hardware.txt` §1.10 本体传感器坐标 |
| 机身尺寸 | 「长度(Lbody) 0.82m」「宽度(Wbody) 0.506m」 | `hardware.txt` §1.7 身体连杆参数 |
| 髋间距 | 「髋前后间距(Lhip) 0.625m」「髋左右间距(Whip) 0.137m」 | `hardware.txt` §1.7 |

**视场角（FOV）：文档未给出，需现场实测。** 素材中只有「96 线」「0.2 m ~ 60 m」和安装位置，没有任何水平/垂直视场角、垂直分辨率、盲区尺寸的数值。**盲区**同理：文档只给出量程下限 0.2 m，未给出机身遮挡造成的侧向盲区——只能从机身尺寸（0.82×0.506 m）与雷达位置（x=±0.32028）做几何推导，且**推导结果对前后雷达归属不唯一**（见 §2.6）。

**文档明确的不适用场景**（对跟随任务直接相关）：

> 「●大面积玻璃环境（穿透/镜像反射）
> ●光滑长走廊或空旷场景（特征不足）
> ●高动态环境（大量移动物体）」
> —— `lidar.txt` §2.1 不适用场景

### 2.2 融合点云的坐标系、频率、点数

| 事实 | 原文 | 出处 |
| --- | --- | --- |
| 坐标系已转 base_link | 「雷达驱动在输出 /LIDAR/POINTS 时，已将点云转换到 base_link 坐标系下（dds_frame_id 配置为 lidar_link，但内部已做静态外参变换）。用户无需手动执行坐标变换。」 | `lidar.txt` §3 |
| 合并语义 | 「话题名 /LIDAR/POINTS」「说明 前后雷达拼接点云数据」「消息类型 sensor_msgs/msg/PointCloud2」「频率 10Hz」 | `lidar.txt` §5.1 |
| 数据说明 | 「/LIDAR/POINTS 输出的点云数据已在驱动层完成前后雷达拼接和坐标变换（转换至 base_link 坐标系），用户无需手动处理。」 | `lidar.txt` §5.1 |
| 话题速查 | 「点云数据话题，默认传输前后雷达拼接点云；可配置为传输前雷达点云，详见激光雷达」「频率 10Hz」「话题可见性 AOS+NOS+GOS上均可见（满足上述依赖的情况下）」 | `topics.txt` §1 |
| GOS 依赖 | 「AOS/NOS 内仅 rsdriver.service；GOS 内额外依赖雷达组播转发服务 multicast-relay.service」 | `lidar.txt` §5.1 / `topics.txt` §1 |
| 实际点数与频率（实测） | 「PointCloud2 rate about 10.01 Hz」「Point count about 97k-100k points/frame」「Fields x, y, z, intensity, ring, timestamp」「Header age about 0.12-0.26 s」 | `M20_DRY_RUN_2026-09-08.md` |
| 扫描侧实测 | 「Scan samples 150」「Scan frequency 10.003 Hz」「Maximum observed interarrival gap 0.129 s」「Source age range 128.34-259.17 ms」 | `M20_GUARD_CHECK_2026-09-08.md` |

**坐标标签的关键例外**（本仓库已处理）：

> 「专页明确：合并点云已经转换到 `base_link` 物理坐标，但 `dds_frame_id` 配置仍可为 `lidar_link`。因此本配置令 pointcloud_to_laserscan 的 `target_frame` 为空，保留头部标签与时间戳，不再次 TF 变换。……不要为了『标签看起来正确』额外平移 ±0.32028 m。」
> —— `M20_ADAPTATION.md` §2 坐标的关键例外

对应配置：

```yaml
m20_cloud_to_scan:
  ros__parameters:
    # Vendor merged points are already body coordinates despite lidar_link label.
    # Empty target_frame prevents a second extrinsic transform.
    target_frame: ''
```
—— `config/m20.yaml`

### 2.3 `/LIDAR/POINTS2` 是什么、`send_separately` 的后果

| 事实 | 原文 | 出处 |
| --- | --- | --- |
| 配置项 | 「配置项：`send_separately: false` / `# true: 前后雷达独立发送` / `# false: 前后雷达数据合并发送（默认）」 | `lidar.txt` §5 |
| 配置文件与重启 | 「配置文件路径（需 root 权限）：`/opt/robot/share/node_driver/config/config.yaml`」「修改后需重启雷达驱动生效：`sudo systemctl restart rsdriver.service`」 | `lidar.txt` §5 |
| 独立发送模式话题 | 「/LIDAR/POINTS 说明 传输前雷达点云」「/LIDAR/POINTS2 说明 传输后雷达点云」「频率 10Hz」 | `lidar.txt` §5.1 |
| POINTS2 默认状态 | 「点云数据话题，默认不启用；可配置为传输后雷达点云，详见激光雷达」 | `topics.txt` §2 |
| **后果警告（原文）** | 「警告：开启独立发送后，导航、定位功能将无法使用，自主充电功能部分受限。请仅在明确需要独立数据的场景下启用。」 | `lidar.txt` §5.1 |
| 配置作用域 | 「雷达驱动配置为每个主机独立。修改某台主机的配置文件只影响该主机的雷达数据输出，不影响其他主机。例如修改 106（NOS）上的配置，103（AOS）和其他主机的雷达数据解析不受影响。」 | `lidar.txt` §5.2 作用范围 |
| 本仓库既有立场 | 「默认 `send_separately: false`……本适配要求合并模式，**不修改厂商雷达驱动配置，也不另起一套 GOS 雷达驱动争抢端口**。」 | `M20_ADAPTATION.md` §2 |
| 现场实测状态 | 「NOS and GOS factory lidar `config.yaml` files had the same SHA256 hash and retained `send_separately: false`.」 | `M20_DRY_RUN_2026-09-08.md` |
| 诊断教训 | 「Merely discovering POINTS2 is not evidence it is actively transmitting.」 | `M20_DOCS_DIAGNOSIS_2026-09-07.md` |

**`/LIDAR/POINTS2` 在当前配置下不存在**（默认不启用）。它只有在 `send_separately: true` 时才出现，而该模式带上面那条警告。

### 2.4 数据链路与主机职责

| 事实 | 原文 | 出处 |
| --- | --- | --- |
| 三主机分工 | 「运动主机（AOS）：运动控制主控，运行basic_server……和rl_deploy」「导航主机（NOS）：感知决策，运行建图、定位、规划、避障、自主充电」「通用主机（GOS）：通用主机，供用户二次开发」 | `architecture.txt` §1 |
| 雷达驱动归属 | 「rslidar_node 激光雷达驱动，接收前后双雷达 UDP 数据，合并输出 10Hz 点云」（归属 NOS） | `architecture.txt` §2.2 |
| GOS 点云通路 | 「GOS主机内部已对原始组播雷达数据进行接收、处理和转发，因此在GOS主机上可直接通过ROS话题获取点云数据」 | `lidar.txt` §6.2 |
| 组播转发服务 | 「multicast-relay 是山猫 M20 的雷达点云数据组播转发服务，用于将机器狗上的原始组播雷达数据通过组播方式转发到31网段」「multicast-relay.service 服务出厂时默认处于关闭状态」 | `lidar.txt` §6 / §6.1 |
| 版本要求 | 「山猫M20软件包V1.1.7开始支持该服务，请务必将机器人升级至V1.1.7及以上版本。」 | `lidar.txt` §6.1 |
| V1.1.7 接口 | 「ROS2话题-GOS主机新增激光点云数据话题输出」 | `updates.txt` V1.1.7【接口相关】 |
| 点云缺失修复 | 「修复了开机后没有点云的问题」「修复了光线突变后雷达数据异常的问题」「修复雷达关闭后仍上报残留数据的问题」 | `updates.txt` V1.1.8【修复】/ V1.2.0.1【修复】 |
| 定位对雷达的依赖 | `drdds/msg/LocationStatus` 中 `InputStatusValue input_val { uint8 lidar; ... }` | `topics.txt` §6 |
| 计算平台 | 「处理器 RK3588 (ARM, big.LITTLE 8 核)」「内存 16GB LPDDR4x」（AOS/NOS/GOS 三主机相同） | `compute.txt` §1 |
| 绑核建议 | 「RK3588 的小核（0-3）性能偏低，资源紧张，且 CPU0 被大量用于处理硬件中断和系统中断。如果开发者的二次开发程序运行在机器人内部任一主机上，建议优先绑定到大核（CPU 4-7）上运行」 | `compute.txt` §2.2 |
| 资源配额 | 「GOS (104) 大核 4-7 16GB（大部分可供二开使用）」「NOS (106) 大核 4-7 16GB（扣除系统后约 10GB 可用）」 | `compute.txt` §2.4 |
| 部署纪律 | 「用户自定义节点应优先部署在 GOS (104) 上，NOS（106）次之，应尽量避免抢占 NOS/AOS 核心进程资源。」 | `compute.txt` §2 |

### 2.5 算法本体对雷达的用法（代码事实）

| 事实 | 代码位置 |
| --- | --- |
| 2D 单层切片：`min_height: -0.1`、`max_height: 0.5`、`angle_min/max: ±π`、`angle_increment: 0.0087266`（0.5°）、`range_min: 0.1`、`range_max: 12.0`、`concurrency_level: 1` | `config/m20.yaml` `m20_cloud_to_scan` |
| 目标搜索是「以 `target` 为圆心、半径 `TARGET_RADIUS=0.3 m` 的圆内点取质心」；没命中就输出零速度 | `lidar_tracker.hpp`（`dist_to_target_center < TARGET_RADIUS`）、`common_types.hpp`（`TARGET_RADIUS`） |
| 初始目标 = `(follow_distance, 0)`，M20 配置为 `follow_distance: 1.2` | `robot_nexus.cpp`（`setTarget(tracker_config.follow_distance, 0.0)`）、`config/m20.yaml` |
| 走廊居中：以「机器人→目标」向量为轴、宽度 `corridor_width=0.8 m` 的走廊内取左右最近边界 | `lidar_tracker.hpp`（`proj_y` / `left_y_min` / `right_y_min`）、`config/m20.yaml` |
| APF 势场：`APF_INFLUENCE_DIST=0.25`、`APF_EMERGENCY_DIST=0.2`、`APF_SLOWDOWN_DIST=0.25`，且只对 `point_x > -0.1` 的点生效 | `common_types.hpp`、`lidar_tracker.hpp` |
| 自车排除框（M20 配置）：`frame_front=0.41 / frame_back=0.41 / frame_left=0.253 / frame_right=0.253` | `config/m20.yaml`（注释：「Initial body dimensions, NOT a validated swept leg footprint.」） |
| 门控 `inspect_scan` 默认停止区：前后各 0.7 m、半宽 0.4 m，且**明确不做自车遮罩** | `m20_adapter/core.py`（`"""Scan coordinates must already be body x-forward/y-left. No self masking."""`） |
| 门控的保守设计说明 | 「桥不做自体点剔除，避免错误遮罩把近障隐藏。自体回波可能使其无法 arm，需实测解决。」 —— `M20_ADAPTATION.md` §5 |

### 2.6 关键实测：0.45 m「后左」回波的几何归属

实测原文：

> 「Closest observed stop-region return: distance 0.44784 m, x=-0.36685 m, y=+0.25687 m. In the documented body coordinate convention, this is rear-left. Its physical source has not been identified: a real nearby object and body/leg returns remain possibilities.」
> —— `M20_GUARD_CHECK_2026-09-08.md`

> 「Valid scans with stop-region returns | 81 frames」「Stop-region returns per frame | 0-16」
> —— `M20_GUARD_CHECK_2026-09-08.md`

**几何推算（本节为推导，非文档原文，已标注）**：该点相对两个雷达原点的极坐标如下：

| 雷达 | 原点（机身坐标） | 到该点距离 | 该点在雷达自身坐标系的角度 | 视线是否被机身遮挡 |
| --- | --- | --- | --- | --- |
| 前雷达 | `(+0.32028, 0)` | `0.7336 m` | `+159.5°`（几乎正后方） | 会立即穿入机身（机身 x∈[-0.41,0.41]、y∈[-0.253,0.253]），**被遮挡** |
| 后雷达 | `(-0.32028, 0)` | `0.2611 m` | `+100.3°`（近乎正侧方） | 视线在 y=0.253 处穿出机身，对应距离约 `0.253 m`，**与实测 0.44784 m 不符** |

两个距离都**大于**文档量程下限 0.2 m，所以「哪个雷达测到的」**不能由量程唯一判定**；而两条视线在几何上都不干净。**可确定的推论有两条**：

1. 该点落在机身包络内（`|x|=0.367 < 0.41`、`|y|=0.257 ≳ 0.253`，恰在包络边界上），且 2D 切片高度区间 `[-0.1, +0.5]`（机身坐标）**正好覆盖雷达自身所在的 z=-0.013 高度**——也就是说，**这一层切片切到自车壳体的概率很高**，与配置注释「雷达可能扫描到的内部支架」以及「NOT a validated swept leg footprint」呼应。
2. 实测原文本身已给出保守结论：「Its physical source has not been identified: a real nearby object and body/leg returns remain possibilities.」——**不应据此断言它来自后雷达**。

**因此**：「关掉后雷达就能消除误停」的直觉并不成立。更可能的成因是**自车遮罩（现值 `0.15/0.35/0.15/0.15`）小于机身本体（`0.41/0.253`）**，以及**后向停止区把机身回波当成了障碍**。正确做法是现场清空环境后复测（见 §6.3）再定稿遮罩尺寸，并用角度扇区把门控限制到前向。

**同一组实测里更重要的算法信号**：

> 「Last raw velocity | about `x=+0.326 m/s`, `y=+0.500 m/s`, `yaw=0`」「The lateral component is too large to accept as an expected straight-follow result.」
> —— `M20_DRY_RUN_2026-09-08.md`

人在正前方（实测质心 `x=1.88 m, y=0.03 m`）却输出 `y=+0.500 m/s` 横向速度，说明**横向项（走廊居中 `-(left_y_min+right_y_min)`）被单侧点云污染**，与前后雷达配准误差/单侧自车点同源。这是本选型必须一起解决的问题。

---

## 三、融合 vs 仅前雷达 对比表

| 维度 | 融合点云（现状，360°） | 仅前雷达（用户侧角度/几何筛选） | 改 `send_separately: true` 后只用 POINTS |
| --- | --- | --- | --- |
| 水平覆盖 | 360°（前后各一，机身遮挡后实际约前 140~200° + 后 140~200° + 侧向缺口，**具体 FOV 文档未给出**） | 前向扇区（文档未给 FOV；按角度 ±90~±100° 截取可得前向 ~200°） | 前雷达全量（同上） |
| 侧向/后方盲区 | 侧向存在机身遮挡盲区（0.82×0.506 m 机身，雷达在 x=±0.32 m 端部）；后方由后雷达覆盖 | 侧后方无覆盖 | 侧后方无覆盖 |
| 自车回波 | 两个雷达都可能看到自车/腿部；实测最近回波 0.44784 m（后左）来源未定，且原自车遮罩小于机身 → **门控 81/150 帧误停** | 同样存在（自车回波不因只取前向而消失，但前向遮罩更易定义），需靠遮罩解决 | 同样存在 |
| 算力（GOS） | 全量 ~9.7~10 万点/帧 @10 Hz 进 `pointcloud_to_laserscan`（单线程 `concurrency_level: 1`）；**实测 CPU 占用文档未给出** | 若只是角度截取，**CPU 不降**（点数在驱动层已合并、DDS 带宽已占用）；若要真降需另加过滤节点（反而增加一次反序列化） | 理论减半，但代价见下一行 |
| 带宽 | ~9.7~10 万点 × 约 32 B ≈ 3.1 MB/帧 ≈ 31 MB/s @10 Hz（**按字段 x,y,z,intensity,ring,timestamp 推算**；文档未给带宽值） | 不变（订阅的是同一个融合话题） | 可减半，但需改厂商配置 |
| 延迟 | 实测 header age 0.12~0.26 s；扫描源年龄 128~259 ms | 相同 | 相同（不改善链路延迟） |
| 故障域 | 单雷达失效时话题仍在（另一雷达继续发），**门控看不到「前雷达没了」**——危险失效模式 | 同左 | 前雷达失效 → POINTS 静默 → 门控 fail-closed（更安全），但需要改原厂配置 |
| 与官方导航/定位/充电共存 | 不动配置，`send_separately: false`，原厂链路不受影响 | 不动配置，最安全 | 「导航、定位功能将无法使用，自主充电功能部分受限」；虽然配置按主机独立，**是否只改 GOS 就安全，文档未给出，需厂商确认** |
| 实现复杂度 | 零（现状） | 低：改 `angle_min/angle_max` + 扩大自车遮罩（无需新节点、无需 `ring`） | 高：root 改文件 + 重启 rsdriver + 厂商确认 + 原厂功能回归测试 |
| 对跟随算法的增益 | 后雷达唯一可用增益是侧后方覆盖与后方避障；**但当前算法没有「人在身后掉头跟」行为**，且后向停止区是误停主因 | 前向扇区完全够跟随（目标始终在 `(1.2, 0)` 附近的 0.3 m 半径圆内） | 同左 |
| 综合判断 | **推荐（配合前向加权）** | 等价实现方式，作为「纯前向」简化版可选 | **不推荐**（收益小、代价大、需厂商支持） |

---

## 四、推荐方案与落地配置

### 4.1 推荐方案：融合点云 + 前向扇区加权 + 自车遮罩

**一句话**：话题、驱动、`send_separately` 全部不动；把「算法有效视场」和「门控停止区」都收窄到前向扇区，并把自车遮罩扩大到覆盖两个雷达自身位置；后雷达点云保留在话题里，但**不再参与停止区判定、目标质心和 APF**。

理由：
- 不动原厂配置 → 原厂导航/定位/充电零风险（文档警告只针对 `send_separately: true`）。
- 前向扇区已足够跟随：目标是 `(1.2, 0)` 附近 0.3 m 半径圆内的点（`follow_distance: 1.2`、`TARGET_RADIUS=0.3`）。
- 后雷达的「侧后方覆盖」对当前算法无正收益（无掉头行为），而后向停止区是实测 81/150 帧误停的直接原因。
- 融合链路的故障域优势保留：单雷达失效时话题仍在，可**另加健康判定**（见 4.4）。

### 4.2 `pointcloud_to_laserscan` 参数（`config/m20.yaml`）

```yaml
m20_cloud_to_scan:
  ros__parameters:
    # Vendor merged points are already body coordinates despite lidar_link label.
    # Empty target_frame prevents a second extrinsic transform.
    target_frame: ''
    # 高度切片：保持现状（z 的零点语义必须先实测确认，见 §六）
    min_height: -0.1
    max_height: 0.5
    # 关键改动：由 ±π（360°）收窄为 ±100°（前向 200° 扇区）
    angle_min: -1.7453292519943296     # -100 deg
    angle_max:  1.7453292519943296     # +100 deg
    angle_increment: 0.008726646259971648   # 0.5 deg -> 400 bins
    scan_time: 0.1
    range_min: 0.1
    range_max: 12.0
    use_inf: true
    concurrency_level: 1
```

**为什么角度截取 ≈「只用前雷达」**（几何论证，非文档原文）：对同一物理点，前雷达原点在 `x=+0.32`、后雷达在 `x=-0.32`；`pointcloud_to_laserscan` 在每个角度 bin 只保留**最近**的点。在 `x>0` 的前向扇区内，前雷达对该点的距离恒小于后雷达，因此**前向 bin 里胜出的必然是前雷达的回波**；后雷达只在后向扇区胜出，而该扇区已被 `angle_min/angle_max` 排除。

**收益**：
- 门控不再因后向点而误停（后向点根本不进 `/m20/scan`）。
- 目标质心不会被后雷达对同一人的重复回波劈成两簇（前后雷达外参 yaw 误差会让人被切成两片，导致质心横向漂移）。
- 走廊居中项只吃前向走廊内的点，减少 `y=+0.5 m/s` 那种横向误输出。

**代价**：失去后向停止区保护。必须同时满足：跟随算法**不允许后向运动**（`dist_error<0` 时的倒退需限幅或禁掉）、现场禁止在人/障碍位于机器人正后方时启动跟随。

### 4.3 自车遮罩（必须一起改，否则误停仍在）

`config/m20.yaml` 中 `robot_nexus` 的自车排除框要覆盖两个雷达的自身位置与近场：

```yaml
robot_nexus:
  ros__parameters:
    # 覆盖 x=±0.32028 的两个雷达本体 + 机身 0.82×0.506 m + 余量
    frame_front: 0.55
    frame_back: 0.55
    frame_left: 0.32
    frame_right: 0.32
```

依据：机身半长 0.41 m、半宽 0.253 m（`hardware.txt` §1.7），雷达在 x=±0.32028（`lidar.txt` §3）；实测近距回波落在 `x=-0.367, y=+0.257`（`M20_GUARD_CHECK_2026-09-08.md`，来源未定），原遮罩 `0.15/0.35/0.15/0.15` 明显小于机身本体，遮不住。**该建议值需用 §6.3 的现场复测确认后再定稿**，不要凭本表直接上线。

**注意**：`M20_ADAPTATION.md` §5 与 `M20_RUNBOOK.md` §7 都明确「桥不屏蔽自体点……不能为消除误停而盲目扩大排除区域」。因此这里的做法是**分层**：
- `robot_nexus` 的自车遮罩只管算法（质心、走廊、APF）——放大是安全的，因为它是「算法输入净化」；
- 桥 `inspect_scan` 的停止区**不改遮罩**，而是改用角度扇区过滤（见 4.4），保持「不隐藏真实近障」的保守原则。

**代价必须写清楚**：自车遮罩放大到 0.55 m 后，`0.55 m` 以内的真实障碍对 APF（`APF_EMERGENCY_DIST=0.2`、`APF_INFLUENCE_DIST=0.25`）**完全不可见**，此时**唯一的近障保护层就是门控的前向停止区 `stop_front=0.7 m`**。因此这条改动与 §4.4 的门控改动**必须同时上线**，不能只改遮罩；并且 `stop_front` 不得下调到 0.55 m 以下。

### 4.4 门控（`m20_adapter/core.py`）需要新增的参数

现状：`inspect_scan(scan, now, max_age, future_tolerance, stop_front, stop_back, stop_half_width)`，矩形停止区 `-stop_back <= x <= stop_front and |y| <= stop_half_width`，且 `stop_back` 被校验为**必须为正**（`bridge.py` 中 `any(not math.isfinite(v) or v <= 0 for v in self.stop.values())` 会抛异常），所以**不能简单把 `stop_back` 设成 0**。

建议新增两个参数（默认值给出）：

```python
def inspect_scan(scan, now, max_age=0.5, future_tolerance=0.1,
                 stop_front=0.7, stop_back=0.7, stop_half_width=0.4,
                 guard_angle_min=-1.7453292519943296,   # 新增：-100 deg
                 guard_angle_max= 1.7453292519943296):  # 新增：+100 deg
```

语义：只在 `guard_angle_min <= angle <= guard_angle_max` 的 bin 上判定停止区；扇区外的点不参与门控。配套 `bridge.py` 声明同名参数。

**为什么门控要用角度而不是把 `stop_back` 调小**：把 `stop_back` 调到 0.1 m 之类仍会保留后向近距回波（实测 `x=-0.367` 恰在后向），而角度过滤是从几何上排除整片后向扇区，语义更干净，也保留了「前向近障不隐藏」的保守设计。

### 4.5 门控参数建议值

```yaml
m20_bridge:
  ros__parameters:
    dry_run: true
    commissioned: false
    aos_host: '10.21.31.103'
    aos_port: 30000
    scan_frame: 'lidar_link'      # 现场核实实际 frame_id 后确认
    max_vx: 0.3
    max_vy: 0.3
    max_wz: 0.6
    stop_front: 0.7               # 保持
    stop_back: 0.7                # 保持（由 guard_angle 限定到前向）
    stop_half_width: 0.4          # 保持
    guard_angle_min: -1.7453292519943296
    guard_angle_max:  1.7453292519943296
```

### 4.6 新增参数清单（汇总）

| 参数 | 所属节点 | 现值 | 建议值 | 依据 |
| --- | --- | --- | --- | --- |
| `angle_min` | `m20_cloud_to_scan` | `-π` | `-100°` | 前向扇区，排除后雷达主导的后向 bin |
| `angle_max` | `m20_cloud_to_scan` | `+π` | `+100°` | 同上 |
| `frame_front/back/left/right` | `robot_nexus` | `0.41/0.41/0.253/0.253` | `0.55/0.55/0.32/0.32` | 覆盖 x=±0.32028 雷达本体 + 机身 0.41/0.253 |
| `guard_angle_min/max` | `m20_bridge`（**新增**） | 无 | `±100°` | 停止区只在前向扇区判定 |
| `min_height/max_height` | `m20_cloud_to_scan` | `-0.1/0.5` | **保持，待实测 z 语义后定** | `M20_RUNBOOK.md` §7：「调试 `min_height/max_height` 前确认 z 是以机身原点而非地面为零。」 |
| `follow_distance` | `robot_nexus` | `1.2` | 保持（1.2 m） | 现场实测目标距离约 1.8~2.0 m 时可检出 |

### 4.7 命令

```bash
# GOS 环境
source /opt/robot/scripts/setup_ros2.sh
source ~/m20_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 1) 被动观察（建图期间/联调期，只跑点云→扫描，不发速度）
taskset -c 4-7 ros2 launch jie_deamon m20_mapping.launch.py

# 2) 跟随 dry-run（不改 AOS、不开发速度）
taskset -c 4-7 ros2 launch jie_deamon m20.launch.py

# 3) 观察门控状态
ros2 topic echo /m20/bridge_status
ros2 topic hz /m20/scan
ros2 topic hz /LIDAR/POINTS
```

**不要执行**：
```bash
# 不推荐：会触发「导航、定位功能将无法使用，自主充电功能部分受限」
sudo sed -i 's/send_separately: false/send_separately: true/' \
  /opt/robot/share/node_driver/config/config.yaml
sudo systemctl restart rsdriver.service
```

---

## 五、风险与不解决项

### 5.1 本选型下仍然存在的安全盲区

| 风险 | 说明 | 现状 |
| --- | --- | --- |
| 负障碍（坑、沟、下水道口） | 2D 单层切片在固定高度取一圈回波，负障碍在该高度没有回波 → 检测不到 | 本方案不解决 |
| 台阶/路沿（下台阶） | 同上，向下的落差不在切片高度内 | 本方案不解决 |
| 悬空障碍（桌面、树枝、横杆） | 高于 `max_height` 的回波被丢弃；M20 配置 `max_height: 0.5`（机身坐标） | 本方案不解决 |
| 玻璃、玻璃门 | 文档明确列为不适用场景：「大面积玻璃环境（穿透/镜像反射）」（`lidar.txt` §2.1） | 本方案不解决，需视觉/超声补盲 |
| 目标身份跟踪 | 算法取「目标圆内点云质心」，**没有身份识别**：走廊里另一个走进该圆的人会被当成目标 | 本方案不解决；实测记录也列为验收阻塞项 |
| 侧向盲区 | 机身 0.82×0.506 m，雷达在两端 → 侧向存在遮挡缺口；FOV 文档未给出 | 需实测确认 |
| 低矮 / 蹲姿 / 儿童目标 | 2D 切片是**雷达高度上的一个水平平面**（雷达 z=-0.013 机身坐标），目标矮于该平面时回波落在切片之下，与负障碍同理 | 本方案不解决，需现场按目标身高实测 |
| 单雷达失效（前雷达） | 融合话题下前雷达失效时话题仍存活，门控看不到 → **危险失效模式** | 见 5.2 缓解 |
| 自车/腿部动态回波 | 腿部在行走中扫掠，静态遮罩无法完全覆盖（配置注释即写「NOT a validated swept leg footprint」） | 需实测动态轮廓 |
| 前后雷达配准误差 | 融合点云中同一目标可能被切成两簇 → 质心漂移、横向误输出（实测 `y=+0.5 m/s` 疑似同源） | 前向扇区化可显著缓解 |
| 目标丢失后的行为 | 「没有命中当前目标时输出零速度，不再追逐上一帧残留目标」（`M20_ADAPTATION.md` §5）——停止而非搜索 | 设计如此，非缺陷 |
| 零速度 ≠ 急停 | 「软件中的零速度是减速停车请求，**不是**软急停。」（`M20_RUNBOOK.md` §8） | 运维须知 |

### 5.2 故障域缓解建议（针对「前雷达失效不可见」）

- 用 `basic_server` 的 `Type=1002, Cmd=5` 读取雷达开关状态：「DevEnable.Lidar.Front 0 / 1 / 2：前雷达 关闭/开启/启动中」（`lidar.txt` §4.1），把 `Front != 1` 作为 disarm 条件。
- 用 rsdriver 日志字段做健康判定：「status 1（0 = 未运行）」「error_code 0（非 0 = 硬件故障）」「cld_size > 0（0 = 无点云输出）」「diff_timestamp 100ms ~ 200ms（> 200ms = 时间同步异常）」「ptp 0x1（0x0 = PTP 未同步）」（`lidar.txt` §4.2）。
- 注意：本桥目前**没有**做这项判定——`M20_ADAPTATION.md` §7 明确「只有一部分有效点也会被判为有效扫描；当前没有双雷达独立健康鉴别或覆盖率诊断。」

### 5.3 文档未给出、必须现场实测的项

- 前后雷达的水平/垂直视场角、垂直角分辨率、机身遮挡造成的侧向盲区尺寸。
- `ring` 字段是否可区分前后雷达（若可区分，则「按 ring 过滤」成为第三条可选路径）。
- 点云 z 的零点语义（机身原点 vs 地面）——直接决定 `min_height/max_height` 是否正确。
- GOS 上 `pointcloud_to_laserscan` 的实际 CPU/RSS/温度与端到端延迟。
- 实际发布者节点名、QoS、`frame_id` 真实值。
- 行走状态下腿部动态回波的轮廓。
- 改 `send_separately: true` 是否只影响该主机（文档只说「配置为每个主机独立」，未说 GOS 改了就一定不破坏原厂功能）。

---

## 六、待现场实测确认项（可执行命令）

> 前置：GOS 上 `su` → `export ROS_DISTRO=foxy` → `source /opt/robot/scripts/setup_ros2.sh` → `export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp`（`M20_DOCS_DIAGNOSIS_2026-09-07.md` §Next discriminating check）。
> 所有命令均为只读/被动，不发布速度、不 arm、不碰 AOS。

### 6.1 雷达配置与话题事实

```bash
# 1. 确认 send_separately 现状（只读，不修改）
sudo cat /opt/robot/share/node_driver/config/config.yaml | grep -n send_separately

# 2. 确认 POINTS2 是否存在（默认不启用时不应出现）
ros2 topic list -t | grep -i lidar
ros2 topic info /LIDAR/POINTS -v
ros2 topic info /LIDAR/POINTS2 -v     # 预期 "Unknown topic"

# 3. 频率与点数
timeout -s INT 15s ros2 topic hz /LIDAR/POINTS

# 4. 点云字段、frame_id、点云尺寸
timeout -s INT 8s ros2 topic echo --once /LIDAR/POINTS --field header
timeout -s INT 8s ros2 topic echo --once /LIDAR/POINTS --field fields
timeout -s INT 8s ros2 topic echo --once /LIDAR/POINTS --field width

# 5. 雷达驱动健康（前后雷达是否都在正常输出）
sudo journalctl -u rsdriver.service --no-pager -n 200 | grep -Ei 'status|error_code|cld_size|diff_timestamp|ptp'
systemctl status rsdriver.service --no-pager
systemctl status multicast-relay.service --no-pager
```

### 6.2 关键判定：`ring` 能否区分前后雷达 + z 零点语义

```bash
# 一次性取一帧，导出 ring/x/z 的联合分布（只读脚本，写到 /tmp）
cat > /tmp/m20_ring_probe.py <<'PY'
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class P(Node):
    def __init__(self):
        super().__init__('m20_ring_probe')
        self.done = False
        self.create_subscription(PointCloud2, '/LIDAR/POINTS', self.cb, qos_profile_sensor_data)
    def cb(self, msg):
        pts = point_cloud2.read_points(msg, field_names=('x','y','z','ring'), skip_nans=True)
        a = np.array([tuple(p) for p in pts])
        if a.size == 0: return
        x, z, ring = a[:,0], a[:,2], a[:,3].astype(int)
        print('points=%d  frame_id=%s' % (a.shape[0], msg.header.frame_id))
        print('x  min/med/max = %.3f / %.3f / %.3f' % (x.min(), np.median(x), x.max()))
        print('z  min/med/max = %.3f / %.3f / %.3f' % (z.min(), np.median(z), z.max()))
        print('ring min/max = %d / %d  unique=%d' % (ring.min(), ring.max(), len(np.unique(ring))))
        for lo, hi in [(-99,-0.6),(-0.6,-0.2),(-0.2,0.2),(0.2,0.6),(0.6,99)]:
            m = (x>=lo)&(x<hi)
            if m.sum():
                print('x in [%6.2f,%6.2f): n=%6d  ring range %d..%d' %
                      (lo, hi, m.sum(), ring[m].min(), ring[m].max()))
        print('front-only ring set (x>0.2):', sorted(set(ring[x>0.2]))[:8], '...')
        print('rear-only  ring set (x<-0.2):', sorted(set(ring[x<-0.2]))[:8], '...')
        self.done = True
rclpy.init(); n = P()
while not n.done: rclpy.spin_once(n, timeout_sec=0.5)
rclpy.shutdown()
PY
python3 /tmp/m20_ring_probe.py
```

判定标准：
- 若 `x>0.2` 与 `x<-0.2` 的 `ring` 集合**不相交**（例如前 0..95、后 96..191）→ 可按 `ring` 过滤，出现第三条可选路径；
- 若两者**重叠**（都 0..95）→ `ring` 不可用于前后区分，只能按几何/角度过滤（即本报告推荐方案）。
- `z` 的 min/med/max 用于确认零点语义：若 `z` 中位数接近 -0.013（雷达安装高度）→ 零点在机身；若接近 0.3~0.5 → 零点在地面。

### 6.3 近距回波定位（判定 0.45 m 回波是自车回波还是真实近障）

**判据**：让机器人静止站立、四周清空，连续抓 100 帧。若近距回波**在清空后依然稳定存在**（距离几乎不变），则它是自车回波；若随环境消失，则是真实物体。同时输出每个回波到两个雷达原点的距离与角度，用来判断它由哪个雷达测得。

```bash
python3 - <<'PY'
import numpy as np, rclpy, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

FRONT, REAR = 0.32028, -0.32028   # lidar.txt §3 静态外参
class P(Node):
    def __init__(self):
        super().__init__('m20_selfecho'); self.n=0; self.hits=[]
        self.create_subscription(LaserScan, '/m20/scan', self.cb, qos_profile_sensor_data)
    def cb(self, m):
        self.n += 1
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or not (m.range_min <= r <= m.range_max): continue
            a = m.angle_min + i*m.angle_increment
            x, y = r*math.cos(a), r*math.sin(a)
            if math.hypot(x, y) < 0.7:
                df = math.hypot(x-FRONT, y); dr = math.hypot(x-REAR, y)
                self.hits.append((x, y, r, math.degrees(a),
                                  df, math.degrees(math.atan2(y, x-FRONT)),
                                  dr, math.degrees(math.atan2(y, x-REAR))))
        if self.n >= 100:
            self.destroy_node(); rclpy.shutdown()
rclpy.init(); n=P(); rclpy.spin(n)
h = np.array(n.hits) if n.hits else np.zeros((0,8))
print('frames=100  near-returns(<0.7m)=%d' % len(h))
if len(h):
    print('x range %.3f..%.3f  y range %.3f..%.3f  body-angle %.1f..%.1f deg'
          % (h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max(),h[:,3].min(),h[:,3].max()))
    print('dist-to-front-lidar %.3f..%.3f  front-lidar-angle %.1f..%.1f deg'
          % (h[:,4].min(),h[:,4].max(),h[:,5].min(),h[:,5].max()))
    print('dist-to-rear-lidar  %.3f..%.3f  rear-lidar-angle  %.1f..%.1f deg'
          % (h[:,6].min(),h[:,6].max(),h[:,7].min(),h[:,7].max()))
    # 落在机身包络内的（|x|<0.41, |y|<0.253）几乎必然是自车回波
    inside = (np.abs(h[:,0]) < 0.41) & (np.abs(h[:,1]) < 0.253)
    print('inside body envelope: %d / %d' % (inside.sum(), len(h)))
    print('sample:', np.round(h[:5], 3))
PY
```

**解读**：
- `body-angle` 集中在 `|angle| > 100°`（后向）→ 角度扇区过滤（§4.2）即可清除；
- 出现 `inside body envelope` 的点 → 是自车壳体/支架回波，必须靠 §4.3 的自车遮罩清除；
- 若 `dist-to-front-lidar < 0.2` 或 `dist-to-rear-lidar < 0.2` → 低于雷达量程下限，说明该点不可能由该雷达产生，需要重新核对坐标系与 frame。

### 6.4 低矮目标可见性（对应用户身高/蹲姿风险）

```bash
# 在正前方 1.2 m 处放置 0.3 / 0.6 / 0.9 / 1.2 m 高的纸箱，逐个观察是否被切片捕获
timeout -s INT 20s ros2 topic echo /m20/scan --field ranges | head -50
# 或用 §6.3 的脚本把阈值从 0.7 m 改成 2.0 m，按高度分层记录命中情况
```

预期：只有落在切片高度平面附近的目标才会稳定出现；矮目标会时断时续甚至完全消失——这是 2D 单层的固有盲区。

### 6.5 性能与延迟（回答「代价对比」）

```bash
# CPU / 内存：点云转扫描节点
PID=$(pgrep -f pointcloud_to_laserscan_node | head -1)
echo "pid=$PID"
pidstat -p $PID 1 30          # 无 pidstat 时用: top -b -d 1 -n 30 -p $PID
taskset -cp $PID              # 确认绑核是否生效
cat /proc/$PID/status | grep -E 'VmRSS|Threads'

# 端到端延迟：扫描时间戳 vs 系统时间
timeout -s INT 20s ros2 topic echo /m20/scan --field header.stamp
# 与 `date +%s.%N` 对比；已实测 128~259 ms（M20_GUARD_CHECK_2026-09-08.md）

# 带宽：融合点云的实际占用
PID=$(pgrep -f rsdriver | head -1); cat /proc/$PID/net/dev
# 或用 iftop/ip -s link 观察 31 网段网卡收发速率

# 全系统负载基线（对比 360° 与 200° 两种配置）
mpstat -P ALL 1 30
htop
```

### 6.6 与官方功能共存的验证

```bash
# 跟随/建图前，确认原厂导航/充电空闲、控制权已让出
systemctl status planner.service --no-pager
ros2 topic info /NAV_CMD -v
ros2 topic echo /LOCATION_STATUS --once     # total_status 应为 1（正常）
ros2 topic echo /CHARGE_STATUS --once       # state 应为 0（空闲）
ros2 topic echo /HES_STATUS --once          # data 应为 0
```

### 6.7 若最终仍要评估「独立发送」路径（需厂商支持）

```bash
# 仅记录，勿在未获厂商确认前执行：
# sudo cat /opt/robot/share/node_driver/config/config.yaml          # 备份
# sudo sed -i 's/send_separately: false/send_separately: true/' ...
# sudo systemctl restart rsdriver.service
# 然后验证：/LIDAR/POINTS 是否只剩前雷达；/LIDAR/POINTS2 是否出现；
# 以及 NOS 上 drmap/localization/charge_manager 是否仍正常（这是必须的回归项）
```

---

## 附：本报告引用的文件清单

| 文件 | 用途 |
| --- | --- |
| `D:\repos\dt_text\lidar.txt` | 双雷达硬件、外参、`send_separately`、组播转发、PTP |
| `D:\repos\dt_text\lidar_external.txt` | 外接主机 RSAIRY、组播 IP/端口、外接 SDK |
| `D:\repos\dt_text\topics.txt` | `/LIDAR/POINTS`、`/LIDAR/POINTS2`、`/NAV_CMD`、`/LOCATION_STATUS` 等 |
| `D:\repos\dt_text\hardware.txt` | 机身尺寸、传感器坐标、腿部连杆 |
| `D:\repos\dt_text\architecture.txt` | AOS/NOS/GOS 职责、`rslidar_node` 归属 |
| `D:\repos\dt_text\compute.txt` | RK3588 8 核、绑核建议、各主机配额 |
| `D:\repos\dt_text\updates.txt` | V1.1.7 GOS 点云、V1.1.8 点云修复、V1.2.0.1 |
| `config/m20.yaml` | 切片、跟随、门控参数现值 |
| `launch/m20.launch.py` / `launch/m20_mapping.launch.py` | 启动拓扑 |
| `include/lidar_tracker.hpp` | 目标质心、走廊居中、APF、速度计算 |
| `include/common_types.hpp` | 跟随距离、走廊宽度、自车排除框、速度上限、APF 距离 |
| `m20_adapter/core.py` | `Guard`、`inspect_scan` 停止区逻辑 |
| `m20_adapter/bridge.py` | 参数声明、frame 校验、dry-run/live 分支 |
| `docs/M20_ADAPTATION.md` | 适配结论、坐标关键例外、门控边界 |
| `docs/M20_DRY_RUN_2026-09-08.md` | 10 Hz、9.7~10 万点/帧、静止跟随预览 |
| `docs/M20_GUARD_CHECK_2026-09-08.md` | 150 帧中 81 帧停止区命中、0.45 m 后左回波 |
| `docs/M20_RUNBOOK.md` | 现场 SOP、z 零点告警、接管验收顺序 |
| `docs/M20_DOCS_DIAGNOSIS_2026-09-07.md` | POINTS2 未启用的判据、GOS 环境前置 |
| `docs/M20_SOURCES.md` | 文档差异与处理原则 |
