#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NEW-03/NEW-08 read-only acceptance probe for vendor ROS point-cloud topics on GOS 10.21.31.104.

This is an ACCEPTANCE PROBE, not a production bridge:
  * it only SUBSCRIBES (no publishers / services / clients / forwards);
  * it never publishes velocity, heartbeat or anything else;
  * like any ROS 2 node it necessarily creates one DDS participant, but only
    with the profile approved for the selected --transport mode (udp: UDP-only,
    no SHM transport; local-shm: UDP + one 50 MiB SHM transport descriptor);
  * it performs NO direct deletion or cleanup of SHM/DDS shared files and never
    kills processes; the middleware may allocate and release its own segments.

Interface:
    python3 -B probe_factory_topics.py --seconds 20
      --seconds   finite float, 1..30 inclusive, default 20.
      --transport udp | local-shm, default udp.
                  udp:       FASTRTPS_DEFAULT_PROFILES_FILE must be exactly
                             /home/user/new_chase/config/diagnostic_udp_only.xml
                  local-shm: FASTRTPS_DEFAULT_PROFILES_FILE must be exactly
                             /home/user/new_chase/config/diagnostic_local_shm.xml
                             AND the process must run as root (euid==0).

Exit codes:
    0  PASS - at least one cloud topic received valid AND fresh data
    1  FAIL - ran to completion but no cloud topic had valid+fresh data,
              or the probe was interrupted early (SIGINT) => interrupted=true
    2  ENV/EXECUTION FAULT - environment mismatch, bad arguments, exception,
       or failed resource cleanup (cleanup_errors non-empty). Tracebacks are
       preserved; exceptions are never swallowed into exit 0.

The caller must provide the environment (this script never sets it):
    ROS_DOMAIN_ID=0  ROS_LOCALHOST_ONLY=0  RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml   (udp)
    FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_local_shm.xml (local-shm, root)

Approved run commands (main agent, real run; run the redirect INSIDE the remote
shell so the evidence log is written on 10.21.31.104, keep the true exit code,
do not use bash -lc, do not use timeout):
    udp (user):
    ssh 104 "env -i HOME=/home/user USER=user \
      PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
      bash --noprofile --norc -c \
      'source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_udp_only.xml; python3 -B /home/user/new_chase/test/probe_factory_topics.py --transport udp --seconds 20' \
      > /home/user/new_chase/evidence/NEW-03-probe.log 2>&1; echo exit=\$?"
    local-shm (root, password typed interactively at the sudo prompt, never
    stored anywhere):
    ssh 104 "sudo -p '[sudo] password for user: ' env -i HOME=/root USER=root \
      PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
      bash --noprofile --norc -c \
      'source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/new_chase/config/diagnostic_local_shm.xml; python3 -B /home/user/new_chase/test/probe_factory_topics.py --transport local-shm --seconds 20' \
      > /home/user/new_chase/evidence/NEW-08-probe.log 2>&1; echo exit=\$?"
"""

import argparse
import json
import math
import os
import signal
import sys
import time
import traceback
import xml.etree.ElementTree as ET

APPROVED_PROFILES = {
    "udp": "/home/user/new_chase/config/diagnostic_udp_only.xml",
    "local-shm": "/home/user/new_chase/config/diagnostic_local_shm.xml",
}
# Backwards-compatible alias for the default (udp) mode.
APPROVED_PROFILE = APPROVED_PROFILES["udp"]
APPROVED_WHITELIST = {"127.0.0.1", "10.21.31.104"}
# NEW-08: local-shm profile must carry EXACTLY one SHM descriptor with this
# segment size (50 MiB, NOT the vendor 500000000 and NOT multiple segments).
SHM_SEGMENT_SIZE = 52428800
SHM_TRANSPORT_ID = "shm_transport"
UDP_TRANSPORT_ID = "udp_transport"
CLOUD_TOPICS = ["/LIDAR/POINTS", "/LIDAR_POINTS", "/lidar_points", "/LIDAR/POINTS2",
                "/new_chase/points"]
MOTION_TOPIC = "/MOTION_INFO"
ALL_TOPICS = CLOUD_TOPICS + [MOTION_TOPIC]
AGE_MIN_OK = -0.1
AGE_MAX_OK = 0.5
FRESH_WINDOW = 0.5

_STOP = {"flag": False}


def _sigint_handler(signum, frame):
    # Graceful: stop the loop, let finally-block cleanup and reporting run.
    _STOP["flag"] = True


def fail_env(reasons, mode=None):
    """Emit a JSON report and exit 2 for environment/argument faults."""
    report = {
        "task": "NEW-03/NEW-08",
        "result": "FAIL",
        "exit_code": 2,
        "stage": "environment_check",
        "errors": reasons,
        "mode": mode,
        "profile": APPROVED_PROFILES.get(mode),
        "euid": os.geteuid(),
        "env": {k: os.environ.get(k) for k in (
            "FASTRTPS_DEFAULT_PROFILES_FILE", "ROS_DOMAIN_ID",
            "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY")},
    }
    print(json.dumps(report, indent=2, default=str))
    return 2


def validate_profile_xml(raw, mode):
    """Pure structural validation of one FastDDS profile XML string.

    Returns [] when the profile exactly matches the approved shape for
    `mode`; otherwise returns a list of reasons. No file or ROS access.
    """
    errs = []
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        return ["profile XML parse error: %s" % exc]
    # SHM must not appear in any actual transport config of the udp profile.
    # The approved files' comments mention SHM only to document their origin;
    # comments are not config and are not checked here.
    config_text = ET.tostring(root, encoding="unicode").upper()
    transports = root.findall(".//{*}transport_descriptor")
    if not transports:
        errs.append("profile has no transport_descriptor")
    ttypes = [(t.findtext("{*}type") or "").strip() for t in transports]
    udp_types = [t for t in ttypes if t == "UDPv4"]
    shm_types = [t for t in ttypes if t == "SHM"]
    if mode == "udp":
        if "SHM" in config_text:
            errs.append("SHM transport present in profile config; only UDPv4 "
                        "is approved")
        if any(t != "UDPv4" for t in ttypes):
            errs.append("non-UDPv4 transport type in profile: %r"
                        % sorted({t for t in ttypes if t != "UDPv4"}))
    elif mode == "local-shm":
        if len(transports) != 2:
            errs.append("local-shm profile must contain exactly 2 transport "
                        "descriptors, got %d" % len(transports))
        if len(udp_types) != 1:
            errs.append("local-shm profile must contain exactly one UDPv4 "
                        "descriptor, got %d" % len(udp_types))
        if len(shm_types) != 1:
            errs.append("local-shm profile must contain exactly one SHM "
                        "descriptor, got %d" % len(shm_types))
        if len(shm_types) == 1:
            shm_desc = [t for t in transports
                        if (t.findtext("{*}type") or "").strip() == "SHM"][0]
            seg_text = (shm_desc.findtext("{*}segment_size") or "").strip()
            try:
                seg = int(seg_text)
            except ValueError:
                errs.append("SHM segment_size not an integer: %r" % seg_text)
            else:
                if seg != SHM_SEGMENT_SIZE:
                    errs.append("SHM segment_size must be exactly %d, got %d"
                                % (SHM_SEGMENT_SIZE, seg))
    else:  # pragma: no cover - argparse choices already restrict this
        errs.append("unknown transport mode: %r" % (mode,))
    if mode in ("udp", "local-shm"):
        whitelist = [a.text.strip() for a in
                     root.findall(".//{*}interfaceWhiteList/{*}address")]
        if set(whitelist) != APPROVED_WHITELIST:
            errs.append("interfaceWhiteList %r != approved %r"
                        % (sorted(whitelist), sorted(APPROVED_WHITELIST)))
        ubt = root.findall(".//{*}useBuiltinTransports")
        if not ubt:
            errs.append("profile missing useBuiltinTransports")
        for e in ubt:
            if (e.text or "").strip().lower() != "false":
                errs.append("useBuiltinTransports must be false "
                            "(no builtin SHM)")
        participants = root.findall(".//{*}participant")
        if not participants:
            errs.append("profile has no participant profile")
        if len(participants) != 1:
            errs.append("profile must contain exactly one participant, got %d"
                        % len(participants))
        for p in participants:
            default = (p.get("is_default_profile") or "").strip().lower()
            if default != "true":
                errs.append("participant profile is not the default profile "
                            "(is_default_profile=%r)" % p.get("is_default_profile"))
        if mode == "local-shm":
            # userTransports must reference exactly both transports.
            ut_refs = [e.text.strip() for e in
                       root.findall(".//{*}userTransports/{*}transport_id")]
            if sorted(ut_refs) != sorted([UDP_TRANSPORT_ID, SHM_TRANSPORT_ID]):
                errs.append("userTransports %r must reference exactly %r"
                            % (ut_refs, [UDP_TRANSPORT_ID, SHM_TRANSPORT_ID]))
        elif mode == "udp":
            ut_refs = [e.text.strip() for e in
                       root.findall(".//{*}userTransports/{*}transport_id")]
            if ut_refs != [UDP_TRANSPORT_ID]:
                errs.append("udp profile userTransports must be exactly "
                            "[%r], got %r" % (UDP_TRANSPORT_ID, ut_refs))
    return errs


def check_environment(mode):
    """Verify caller-provided env for the selected transport mode; never
    modifies the environment. Pure validation, executed BEFORE any ROS
    import. Returns [] on ok."""
    errs = []
    approved = APPROVED_PROFILES.get(mode)
    if approved is None:
        errs.append("unknown transport mode: %r" % (mode,))
        return errs
    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") != approved:
        errs.append(
            "FASTRTPS_DEFAULT_PROFILES_FILE for mode %r must be exactly %r, "
            "got %r" % (mode, approved,
                        os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")))
        return errs  # no point parsing an unapproved path
    if not os.path.isfile(approved):
        errs.append("approved profile file missing: %s" % approved)
        return errs
    try:
        with open(approved, "rb") as f:
            raw = f.read().decode("utf-8", "replace")
        errs.extend(validate_profile_xml(raw, mode))
    except OSError as exc:
        errs.append("cannot read profile: %s" % exc)

    if os.environ.get("ROS_DOMAIN_ID") != "0":
        errs.append("ROS_DOMAIN_ID must be 0, got %r"
                    % os.environ.get("ROS_DOMAIN_ID"))
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        errs.append("RMW_IMPLEMENTATION must be rmw_fastrtps_cpp, got %r"
                    % os.environ.get("RMW_IMPLEMENTATION"))
    if os.environ.get("ROS_LOCALHOST_ONLY") != "0":
        errs.append("ROS_LOCALHOST_ONLY must be 0, got %r"
                    % os.environ.get("ROS_LOCALHOST_ONLY"))
    # NEW-08: local SHM reception requires root; without it the middleware
    # cannot use the same SHM segments as the root SDK. UDP mode has no uid
    # requirement.
    if mode == "local-shm" and os.geteuid() != 0:
        errs.append("local-shm mode requires root (os.geteuid()==0), got %d"
                    % os.geteuid())
    return errs


def shm_snapshot():
    """Read-only snapshot of /dev/shm names + usage. Never deletes anything."""
    try:
        names = sorted(os.listdir("/dev/shm"))
    except OSError as exc:
        names = []
        list_err = str(exc)
    else:
        list_err = None
    try:
        st = os.statvfs("/dev/shm")
        usage = {
            "free_bytes": st.f_bavail * st.f_frsize,
            "total_bytes": st.f_blocks * st.f_frsize,
        }
        usage_err = None
    except OSError as exc:
        usage = None
        usage_err = str(exc)
    return {"names": names, "usage": usage,
            "errors": [e for e in (list_err, usage_err) if e]}


def cloud_frame_validity(msg):
    """Return list of reasons why this PointCloud2 frame is not valid (empty=valid).

    Pure function over the message attributes; safe to unit-test with a
    types.SimpleNamespace stand-in (no ROS import required).
    """
    reasons = []
    field_names = {f.name for f in msg.fields}
    if not {"x", "y", "z"}.issubset(field_names):
        reasons.append("missing_xyz_fields")
    if msg.width * msg.height <= 0:
        reasons.append("width_height_not_positive")
    else:
        if msg.point_step <= 0:
            reasons.append("point_step_not_positive")
        if msg.row_step < msg.point_step * msg.width:
            reasons.append("row_step_smaller_than_point_step_times_width")
        if len(msg.data) < msg.row_step * msg.height:
            reasons.append("data_len_smaller_than_row_step_times_height")
    if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
        reasons.append("stamp_zero")
    return reasons


def init_cloud_stats():
    """Fresh per-topic stats dict. last_* fields always describe the LATEST
    frame; last_valid distinguishes valid from invalid latest frames."""
    return {
        "count": 0,
        "valid_count": 0,
        "invalid_count": 0,
        "last_valid": False,
        "last_reasons": [],
        "first_rx_monotonic": None,
        "last_rx_monotonic": None,
        "last_valid_rx_monotonic": None,
        "age_min": None,       # receive-time stamp age over valid frames
        "age_max": None,
        "age_last": None,      # receive-time stamp age of latest VALID frame;
                               # reset to None whenever the latest frame is invalid
        "last_frame_id": None,
        "width": None,
        "height": None,
        "bytes": None,
        "point_step": None,
        "row_step": None,
        "fields": None,
        "stamp_last_sec": None,   # source stamp of latest VALID frame
        "stamp_last_nsec": None,
    }


def make_cloud_callback(stats, topic):
    """Return a STRICT single-parameter callback bound to `topic`.

    rclpy (Foxy) inspects the callback signature: a 2-parameter callback is
    treated as WITH_MESSAGE_INFO and receives (msg, MessageInfo). Binding the
    topic via a closure (not a default-arg lambda) keeps the callback
    single-argument so it is called as plain (msg).
    """
    def on_cloud(msg):
        st = stats[topic]
        now_mono = time.monotonic()
        st["count"] += 1
        if st["first_rx_monotonic"] is None:
            st["first_rx_monotonic"] = now_mono
        st["last_rx_monotonic"] = now_mono
        st["width"] = int(msg.width)
        st["height"] = int(msg.height)
        st["bytes"] = int(len(msg.data))
        st["point_step"] = int(msg.point_step)
        st["row_step"] = int(msg.row_step)
        st["fields"] = [{"name": f.name, "datatype": int(f.datatype)}
                        for f in msg.fields]
        st["last_frame_id"] = str(msg.header.frame_id)
        st["stamp_last_sec"] = int(msg.header.stamp.sec)
        st["stamp_last_nsec"] = int(msg.header.stamp.nanosec)

        reasons = cloud_frame_validity(msg)
        if reasons:
            # invalid frame: invalidate the "latest" view, reset staleness
            st["invalid_count"] += 1
            st["last_valid"] = False
            st["last_reasons"] = reasons[:5]  # bounded, no payload stored
            st["age_last"] = None
            return

        st["valid_count"] += 1
        st["last_valid"] = True
        st["last_reasons"] = []
        st["last_valid_rx_monotonic"] = now_mono
        stamp_age = time.time() - (msg.header.stamp.sec +
                                   msg.header.stamp.nanosec * 1e-9)
        st["age_last"] = stamp_age
        st["age_min"] = (stamp_age if st["age_min"] is None
                         else min(st["age_min"], stamp_age))
        st["age_max"] = (stamp_age if st["age_max"] is None
                         else max(st["age_max"], stamp_age))

    return on_cloud


def cloud_summary(st, now_mono, now_wall):
    """Pure per-topic summary/judgment. Freshness is decided HERE, at
    judgment time, from the latest VALID frame's saved source stamp
    (source_stamp_age_at_check = now_wall - stamp) plus the receiver-side age
    (receiver_age_at_check = now_mono - last_valid_rx_monotonic). Both must
    pass; a stale-in-either-sense or invalid latest frame can never PASS."""
    res = {
        "count": st["count"],
        "valid_count": st["valid_count"],
        "invalid_count": st["invalid_count"],
        "last_frame_valid": st["last_valid"],
        "last_reasons": list(st["last_reasons"]),
        "first_rx_monotonic": st["first_rx_monotonic"],
        "last_rx_monotonic": st["last_rx_monotonic"],
        "age_min": st["age_min"],
        "age_max": st["age_max"],
        "age_last": st["age_last"],
        "last_frame_id": st["last_frame_id"],
        "width": st["width"],
        "height": st["height"],
        "bytes": st["bytes"],
        "point_step": st["point_step"],
        "row_step": st["row_step"],
        "fields": st["fields"],
        "stamp_last_sec": st["stamp_last_sec"],
        "stamp_last_nsec": st["stamp_last_nsec"],
    }
    if (st["count"] >= 2
            and st["first_rx_monotonic"] is not None
            and st["last_rx_monotonic"] is not None
            and st["last_rx_monotonic"] > st["first_rx_monotonic"]):
        res["hz"] = ((st["count"] - 1)
                     / (st["last_rx_monotonic"] - st["first_rx_monotonic"]))
    else:
        res["hz"] = None

    src_age = None
    recv_age = None
    if st["last_valid"] and st["last_valid_rx_monotonic"] is not None \
            and st["stamp_last_sec"] is not None:
        src_age = now_wall - (st["stamp_last_sec"]
                              + st["stamp_last_nsec"] * 1e-9)
        recv_age = now_mono - st["last_valid_rx_monotonic"]
    res["source_stamp_age_at_check"] = src_age
    res["receiver_age_at_check"] = recv_age

    res["valid"] = st["valid_count"] > 0
    res["fresh"] = bool(
        st["last_valid"]
        and src_age is not None
        and AGE_MIN_OK <= src_age <= AGE_MAX_OK
        and recv_age is not None
        and recv_age <= FRESH_WINDOW)
    return res


def main():
    parser = argparse.ArgumentParser(
        description="NEW-03/NEW-08 read-only ROS point-cloud acceptance probe")
    parser.add_argument("--seconds", type=float, default=20.0,
                        help="probe duration in seconds, 1..30 (default 20)")
    parser.add_argument("--transport", choices=["udp", "local-shm"],
                        default="udp",
                        help="transport profile to require (default: udp)")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not (1.0 <= args.seconds <= 30.0):
        return fail_env(["--seconds must be a finite float in [1, 30], got %r"
                         % (args.seconds,)], args.transport)

    env_errs = check_environment(args.transport)
    if env_errs:
        return fail_env(env_errs, args.transport)

    # ---- ROS imports only AFTER env verification (no participants yet) ----
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                           QoSReliabilityPolicy, QoSProfile)
    from sensor_msgs.msg import PointCloud2
    from drdds.msg import MotionInfo

    probe_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    shm_before = shm_snapshot()
    ctx = None
    node = None
    executor = None
    subscriptions = []
    exec_traceback = None
    cleanup_errors = []
    sigint_received = False

    stats = {t: init_cloud_stats() for t in CLOUD_TOPICS}
    stats[MOTION_TOPIC] = {"count": 0, "state": None, "gait": None}
    discovery = {"topic_names_and_types": None, "publishers": {},
                 "errors": []}

    def _cleanup_step(desc, fn):
        try:
            fn()
        except Exception as exc:
            cleanup_errors.append("%s: %r" % (desc, exc))

    def on_motion(msg):
        st = stats[MOTION_TOPIC]
        st["count"] += 1
        try:
            st["state"] = int(msg.data.motion_state.state)
            st["gait"] = int(msg.data.gait_state.gait)
        except AttributeError as exc:
            st["state"] = "attr_error:%s" % exc

    duration = None
    try:
        ctx = Context()
        ctx.init()
        node = Node(
            "new_chase_probe_factory_topics",
            context=ctx,
            start_parameter_services=False,
            enable_rosout=False,
        )
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        for topic in CLOUD_TOPICS:
            subscriptions.append(node.create_subscription(
                PointCloud2, topic, make_cloud_callback(stats, topic),
                probe_qos))
        subscriptions.append(node.create_subscription(
            MotionInfo, MOTION_TOPIC, on_motion, probe_qos))

        signal.signal(signal.SIGINT, _sigint_handler)

        start_mono = time.monotonic()
        deadline = start_mono + args.seconds
        while not _STOP["flag"] and time.monotonic() < deadline:
            # bounded spin; time.monotonic keeps the loop well-defined
            executor.spin_once(timeout_sec=0.1)
        duration = time.monotonic() - start_mono
        sigint_received = _STOP["flag"]

        # Discovery mapping: single query set, still inside the live-context
        # window (before finally destroys node/context), no ros2 daemon used.
        try:
            discovery["topic_names_and_types"] = [
                {"topic": t, "types": list(types)}
                for t, types in node.get_topic_names_and_types()
            ]
        except Exception as exc:
            discovery["errors"].append(
                "get_topic_names_and_types: %r" % (exc,))
        for topic in ALL_TOPICS:
            try:
                infos = node.get_publishers_info_by_topic(topic)
                discovery["publishers"][topic] = [{
                    "node": str(i.node_name),
                    "node_namespace": str(i.node_namespace),
                    "topic_type": str(i.topic_type),
                    "endpoint_gid": str(i.endpoint_gid),
                    "qos": str(i.qos_profile),
                } for i in infos]
            except Exception as exc:
                discovery["errors"].append(
                    "get_publishers_info_by_topic(%s): %r" % (topic, exc))
    except KeyboardInterrupt:
        sigint_received = True
        duration = None
    except Exception:
        exec_traceback = traceback.format_exc()
        duration = None
    finally:
        # Release everything; failures are RECORDED (never silently dropped)
        # and force exit 2. No shared/SHM files are ever removed here.
        if executor is not None and node is not None:
            _cleanup_step("executor.remove_node",
                          lambda: executor.remove_node(node))
        if executor is not None:
            _cleanup_step("executor.shutdown", executor.shutdown)
        if node is not None:
            _cleanup_step("node.destroy_node", node.destroy_node)
        if ctx is not None:
            try:
                if ctx.ok():
                    _cleanup_step("context.shutdown", ctx.shutdown)
            except Exception as exc:
                cleanup_errors.append("context.ok(): %r" % (exc,))

    shm_after = shm_snapshot()
    shm_added = sorted(set(shm_after["names"]) - set(shm_before["names"]))
    shm_note = ("no direct deletion; middleware may allocate/release; "
                "nothing was removed by this probe")

    now_mono = time.monotonic()
    now_wall = time.time()
    cloud_results = {t: cloud_summary(stats[t], now_mono, now_wall)
                     for t in CLOUD_TOPICS}
    pass_topic = None
    for topic in CLOUD_TOPICS:
        if cloud_results[topic]["fresh"] and cloud_results[topic]["valid"]:
            pass_topic = topic
            break

    motion_summary = {
        "count": stats[MOTION_TOPIC]["count"],
        "state": stats[MOTION_TOPIC]["state"],
        "gait": stats[MOTION_TOPIC]["gait"],
    }

    fault = exec_traceback is not None or bool(cleanup_errors)
    # An interrupted run never counts as a full-duration PASS.
    passed = pass_topic is not None and not fault and not sigint_received
    exit_code = 2 if fault else (0 if passed else 1)

    report = {
        "task": "NEW-03/NEW-08",
        "script": os.path.abspath(__file__) if "__file__" in globals() else
                  "/home/user/new_chase/test/probe_factory_topics.py",
        "mode": args.transport,
        "profile": APPROVED_PROFILES.get(args.transport),
        "euid": os.geteuid(),
        "result": "PASS" if passed else "FAIL",
        "pass_topic": pass_topic,
        "seconds_requested": args.seconds,
        "duration_monotonic": duration,
        "sigint_graceful_exit": sigint_received,
        "interrupted": sigint_received,
        "interrupted_note": "early SIGINT exit cannot count as a full "
                            "duration PASS; report is FAIL with exit 1"
                            if sigint_received else None,
        "execution_fault": bool(exec_traceback),
        "cleanup_errors": cleanup_errors,
        "cloud_topics": cloud_results,
        "motion_info": motion_summary,
        "motion_note": "motion_state.state is a vendor motion-state enum, "
                       "NOT a whole-machine safety clearance.",
        "discovery": discovery,
        "shm": {
            "before": shm_before,
            "after": shm_after,
            "added_segments": shm_added,
            "note": shm_note,
        },
        "exit_code": exit_code,
    }

    if exec_traceback:
        sys.stderr.write(exec_traceback)
    if cleanup_errors:
        sys.stderr.write("cleanup errors:\n" + "\n".join(cleanup_errors)
                         + "\n")

    print(json.dumps(report, indent=2, default=str))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
