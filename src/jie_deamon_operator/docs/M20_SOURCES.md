# 文档证据与待确认点

查阅日期：2026-09-06。用户消息中的断行和中文句号已清理，按原文档 ID 访问。
读取了以下 19 篇与本次适配相关的页面，没有声称检查所有附件、图片或全部关联页面。
文本快照在 `vendor_text/`，原 URL、读取日期和文本 SHA-256 在 `vendor_text/manifest.json`。
快照为公开页面 SSR 正文的纯文本提取，图片尺寸图、流程图和 SDK 附件不在其中；保留原文错误供核查。

| 来源 | 本次使用的事实 |
| --- | --- |
| [软件系统架构](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/AR4GpnMqJzM30KZ3UklzoGBLVKe0xjE3) | AOS basic_server/rl_deploy；NOS rslidar_node/planner；GOS 二开 |
| [计算平台与资源](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/Gl6Pm2Db8D30AMZ0Se3l4Re6JxLq0Ee4) | 优先 GOS，三主机 RK3588，CPU 4~7 建议 |
| [网络配置](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/93NwLYZXWyg6Rxw6tNR75MABJkyEqBQm) | AOS .103/GOS .104/NOS .106，31 网段，AOS 不提供 SSH/VNC |
| [运行服务](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/XPwkYGxZV3RbA43bU9znqRd5WAgozOKL) | 原厂服务和监控入口 |
| [系统时间](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/P7QG4Yx2Jp7mXxnmiQa3A1DDV9dEq3XD) | NOS PTP Master，AOS/GOS/雷达 Slave |
| [对外通信](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/gvNG4YZ7JnemNxlmiNv35roGV2LD0oRE) | basic_server 与 DDS 两种通道 |
| [ROS 2 / DDS 总览](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/o14dA3GK8g5y4QGySnyegPppV9ekBD76) | Ubuntu 20.04/Foxy/DrDDS、Fast DDS 兼容、Domain 0 |
| [话题速查](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/oP0MALyR8k73gzN3iYagqwbK83bzYmDO) | PointCloud2 10 Hz，NavCmd，状态和网段可见性 |
| [激光雷达](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/20eMKjyp81RbA0YbUe21nQMZWxAZB1Gv) | 双雷达、原始组播转发、GOS 原厂解析、已变换的机身点云、配置独立 |
| [外部主机雷达接收](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/YQBnd5ExVEw5djy5UARXAoeQ8yeZqMmz) | 外接 SDK、RSAIRY、组播 IP/6691/6692，GOS 与外接电脑不同 |
| [ROS 2 运控](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/KGZLxjv9VG3OAkQOS6ALnkKGV6EDybno) | RL=17，NAV_CMD 导航模式限制，planner/充电冲突，速度区间，500 ms 超时 |
| [basic_server 运控](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/Gl6Pm2Db8D30AMZ0Se3l4yB5JxLq0Ee4) | Cmd21 归一化、Cmd25 SI 速度，BasicStatus，实际运动反馈 |
| [协议总览](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/gwva2dxOW4Kb6v3btk13Eplv8bkz3BRL) | 16 字节头、小端长度/ID、JSON、UDP/TCP、心跳、状态注册和错误码 |
| [Python 开发教程](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/dpYLaezmVNLdDRkdFgp471d68rMqPxX6) | 独立交叉核对包头示例 |
| [模式管理](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/lyQod3RxJK3d4eNdSokApxaaJkb4Mw9r) | 常规0/导航1/辅助2，切模式后重置步态 |
| [硬件参数](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/R1zknDm0WR3dN62dS94vel1YVBQEx5rG) | 身体长度0.82 m、宽度0.506 m；动态腿部运动不能由静态尺寸完全覆盖 |
| [更新说明](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/6LeBq413JAz3A9Y3CZL1A6O98DOnGvpb) | V1.1.7 GOS 点云、Sleep 整型；V1.1.8 跨版本 ROS 监听修复 |
| [软件开发指南](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/qnYMoO1rWxDL7r6LHbad2kA9W47Z3je9) | BasicStatus Direction=0 正向/1 后向，Charge/HES 等解释 |
| [硬急停](https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/KGZLxjv9VG3OAkQOS6ALr6mmV6EDybno) | 硬急停状态接口，现场急停保留 |

## 文档差异及处理

1. 架构表把雷达驱动归为 NOS，雷达专页同时描述 AOS/NOS/GOS 各自解析和独立配置。
   采用细化的数据路径，避免声称“所有点云都由 NOS 通过 DDS 发送”。实际发布者仍需现场查看。
2. 点云物理坐标是 base_link，标签却可为 lidar_link。按专页不重复变换，但严格核对固件实际输出。
3. 模式页例子把 ControlUsageMode 放在 Items 下，运控页放在 Items.BasicStatus 下。
   桥接受两种**完整**布局；只给一个模式字段的局部回包不能使能。
4. 更新说明 V1.1.7 将上报 Sleep 改为 int，旧接口示例另有 bool。桥使用新版本整型上报，缺字段或 bool 均拒绝放行。
5. ROS 运控页既写站立会自动进入 RL，又要求显式发 state=17；MotionInfo 示例两个子字段同名 data。
   不据此仿造消息或编写自动 ROS 状态推进。本次使用 basic_server 速度与状态，姿态由原厂控制端管理。
6. 软件开发指南旧表出现 0x1002 高台，细分运控专页使用 0x1003 楼梯。
   本桥按细分运控页 V1.1.7 速度表列出 4097/4099/12290/12291；遇到 4098 或其他未支持值拒绝放行。
   实机未核对前不能把这些数值当成跨版本通用常量。
7. ROS 页有 10 Hz 和 20 Hz 两种建议，统一桥发送 20 Hz；机器人本体超时停车虽有文档说明，仍需实测。
8. 组播服务命令旁没有明确执行主机，外接指南指向 NOS 转发网口 .106。
   NOS 是首先检查对象，但需 `systemctl cat` 和网卡确认实际安装，不自动假定 unit 所属。

## 验证记录

- Windows Python 3.13：25 项 unittest 已通过，包括真实本机 UDP 回环。
- Python 源码语法检查：已通过；ROS 依赖未导入运行。
- Git diff whitespace 检查：已通过。
- C++ 三项回归用例、ROS 全链路 smoke、Foxy/Humble CI：已编写，当前环境未执行。
- ARM64 构建、GOS 性能、实体运动、现场通信：未验证。

上游代码依据：`src/robot_nexus.cpp`、`include/lidar_tracker.hpp`、
`include/common_types.hpp`、`include/direct_control.hpp`、`src/web_comm.cpp`、
`src/android_comm.cpp`、构建文件及 launch。没有修改上游远程仓库，没有创建提交或发布 PR。
