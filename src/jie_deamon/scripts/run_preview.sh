#!/usr/bin/env bash
# NEW-06: M20 预览启动脚本（清洁环境，只 source /opt/ros/foxy + 新install）
# 用法(NEW-11 两终端流程，见 docs/NEW_CHASE.md):
#   终端1(root, 独立小进程, 不加载ROS环境/不daemon, 仅接点云):
#     sudo env -i HOME=/root USER=root \
#       PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
#       /home/user/new_chase/bin/drdds_cloud_export --continuous
#     (验收可用 --seconds 25; 用户自己输密码, 不把明文写进任何文件)
#   终端2(普通user, 本脚本):
#     /home/user/new_chase/src/jie_deamon/scripts/run_preview.sh
#   停止: 各自 Ctrl-C 正常析构退出，不 pkill，不清理 SHM。
#
# 默认 cloud_topic 已改为 /new_chase/points（NEW-09 已验证 export→ROS 链路：
# 25秒 rx249/write249/err0, user rclpy 20秒 count200 valid200 hz9.9967）；
# 原生 /LIDAR/POINTS 直连 ROS 尚未打通。override 保留:
#   run_preview.sh use_cloud_converter:=false          # 合成LaserScan测试
#   run_preview.sh cloud_topic:=/LIDAR/POINTS          # 原生主题（未打通）
#   NEW_CHASE_ROS_DOMAIN=42 run_preview.sh             # 指定测试域
#
# 注意：不使用 bash -lc（避免加载旧workspace），环境为最小清洁集；
# FASTRTPS_DEFAULT_PROFILES_FILE 指向只读诊断XML（主代理复核，禁止修改）。
set -euo pipefail

NEW_CHASE_ROOT=/home/user/new_chase
FOXY_SETUP=/opt/ros/foxy/setup.bash
INSTALL_SETUP=${NEW_CHASE_ROOT}/install/setup.bash

if [[ -z "${_NEW_CHASE_CLEAN:-}" ]]; then
    # 重新以清洁环境执行自身：仅保留 HOME/USER/LOGNAME/SHELL/TERM/LANG/PATH 最小集，
    # 显式传入的 ROS 变量——绝不继承用户环境的 _old/旧workspace prefix 残留
    exec env -i \
        _NEW_CHASE_CLEAN=1 \
        HOME="${HOME:-/home/user}" \
        USER="${USER:-user}" \
        LOGNAME="${LOGNAME:-user}" \
        PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
        TERM="${TERM:-xterm}" \
        LANG="${LANG:-C.UTF-8}" \
        ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${NEW_CHASE_ROS_DOMAIN:-0}}" \
        bash --noprofile --norc "$0" "$@"
fi

# NEW-06 P1: source阶段临时关闭 -u（Foxy setup脚本可能引用未导出的COLCON_TRACE等）
set +u
source "${FOXY_SETUP}"
source "${INSTALL_SETUP}"
set -u

# ROS_DOMAIN_ID 已通过清洁env透传（默认0，测试可由 NEW_CHASE_ROS_DOMAIN/ROS_DOMAIN_ID 覆盖）
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# NEW-06 P1: 显式固定DDS实现与网络模式，杜绝用户环境变量残留
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml

cd "${NEW_CHASE_ROOT}"
exec ros2 launch jie_deamon new_chase_preview.launch.py "$@"
