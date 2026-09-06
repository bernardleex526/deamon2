# jie_deamon

## M20 Pro 离线适配

本分支新增云深处 M20 Pro 的 GOS 部署配置、点云转扫描接入、basic_server
速度桥和离线测试，默认 `dry_run=true`，不连接机器人。详细内容：

- [主机职责、数据链路与适配边界](docs/M20_ADAPTATION.md)
- [离线演练、GOS 部署和实机验收](docs/M20_RUNBOOK.md)
- [文档证据与版本差异](docs/M20_SOURCES.md)

快速运行不依赖 ROS 的测试：`python -m unittest discover -s test -v`。
M20 必须使用 `m20.launch.py`，以下原有 D1 启动流程不适用于 M20。

智元机器狗后端服务节点。基于 ROS 2 构建，集成激光雷达目标追踪、Web 可视化控制、Android App 通讯等功能。

## 介绍视频
Bilibili: [【赛博购物车】带着机器狗去办年货](https://www.bilibili.com/video/BV1GWZJBQEaz/)  
Youtube: [【赛博购物车】带着机器狗去办年货](https://www.youtube.com/watch?v=Gsfbm5OljRc)

## 特别感谢
感谢智元机器人的大力支持，可扫描如下二维码获取更多开发资料。
<div align="center">
  <img src="./media/D1_EDU.jpg" width="400">
</div><br>

## 教材书籍
《机器人操作系统（ROS2）入门与实践》
<div align="center">
  <img src="./media/book_1.jpg" width="300">
</div><br>

淘宝链接：[《机器人操作系统（ROS2）入门与实践》](https://world.taobao.com/item/820988259242.htm)


## 架构概览

```
robot_nexus (中枢节点)
├── lidar_tracker    — 激光雷达目标追踪（含卡尔曼滤波、势场避障）
├── direct_control   — 直接速度控制
├── web_comm         — Web 可视化与 WebSocket 通讯
└── android_comm     — Android App UDP 通讯
```

**ROS 2 话题：**

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/scan` | `sensor_msgs/LaserScan` | 激光雷达扫描数据 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令 |
| 发布 | `/d1_cmd` | `std_msgs/String` | 机器狗动作指令 |

**网络端口：**

| 端口 | 协议 | 用途 |
|------|------|------|
| 8080 | HTTP | Web 控制界面 |
| 8890 | WebSocket | 实时数据推送 |
| 8888 | UDP | 雷达数据发送至 App |
| 8889 | UDP | 接收 App 控制指令 |

## 部署

### 环境要求

- ROS 2 (Humble 或更高版本)
- C++17
- OpenCV

### 编译

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 使用

### 配置启动文件

编辑 `launch/start.launch.py`，将 `lidar_launch` 和 `robot_launch` 替换为你自己的雷达驱动和底盘驱动 launch 文件：

```python
# 雷达驱动 —— 修改为你的雷达 launch 文件
lidar_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("你的雷达包"), 'launch', '你的雷达.launch.py')
    )
)

# 底盘驱动 —— 修改为你的底盘 launch 文件
robot_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("你的底盘包"), 'launch', '你的底盘.launch.py')
    )
)
```

### 一键启动

启动中枢节点及所有驱动（雷达、底盘）：

```bash
ros2 launch jie_deamon start.launch.py
```

**启动参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `active` | `true` | 激活跟随功能 |
| `enable_web` | `true` | 启用 Web 可视化 |
| `enable_opencv` | `false` | 启用 OpenCV 调试窗口 |

示例 — 关闭跟随，仅使用 Web 遥控：

```bash
ros2 launch jie_deamon start.launch.py active:=false
```

### Web 控制界面

启动后访问 `http://<机器人IP>:8080`，支持：

- 跟随/直控模式切换
- 虚拟摇杆直接控制
- 机器狗动作指令下发
- 雷达点云实时可视化
- 双击设置跟随目标坐标

### 键盘控制（调试）

```bash
ros2 run jie_deamon keyboard_cmd
```

## 目录结构

```
jie_deamon/
├── src/
│   ├── robot_nexus.cpp      # 中枢节点主程序
│   ├── web_comm.cpp         # Web 通讯实现
│   ├── android_comm.cpp     # Android 通讯实现
│   └── keyboard_cmd.cpp     # 键盘控制节点
├── include/
│   ├── common_types.hpp     # 公共类型与常量
│   ├── lidar_tracker.hpp    # 雷达追踪模块
│   ├── kalman_filter.hpp    # 卡尔曼滤波器
│   ├── direct_control.hpp   # 直接控制模块
│   ├── web_comm.hpp         # Web 通讯头文件
│   ├── web_server.hpp       # HTTP/WebSocket 服务
│   └── android_comm.hpp     # Android 通讯头文件
├── web/                     # 前端静态资源
├── launch/                  # ROS 2 启动文件
├── CMakeLists.txt
└── package.xml
```

## 许可证

MIT
