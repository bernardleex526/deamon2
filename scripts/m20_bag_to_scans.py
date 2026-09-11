#!/usr/bin/env python3
"""Convert a local cloud bag with the installed ROS converter, localhost only."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    # Never discover the robot domain or publish its topic names.
    os.environ['ROS_DOMAIN_ID'] = '187'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(key, None)
    import rclpy
    import rosbag2_py
    from ament_index_python.packages import get_package_prefix
    from rclpy.serialization import deserialize_message
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, LaserScan
    from sensor_msgs_py import point_cloud2
    import numpy as np
    import yaml
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from m20_adapter.cloud_points import xyz_rows

    args.output.mkdir(parents=True, exist_ok=True)
    config_path = Path(__file__).resolve().parents[1] / 'config/m20.yaml'
    config = yaml.safe_load(config_path.read_text())['m20_cloud_to_scan']['ros__parameters']
    command = [get_package_prefix('pointcloud_to_laserscan') +
               '/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node', '--ros-args',
               '-r', '__node:=offline_cloud_to_scan', '-r', 'cloud_in:=/offline/cloud',
               '-r', 'scan:=/offline/scan']
    for k,v in config.items():
        command += ['-p', k + ':=' + ("''" if v == '' else str(v).lower() if isinstance(v,bool) else str(v))]
    proc = None
    rclpy.init()
    node = rclpy.create_node('offline_bag_scan_capture')
    pub = node.create_publisher(PointCloud2, '/offline/cloud', qos_profile_sensor_data)
    received = {}
    def cb(m):
        received[(m.header.stamp.sec,m.header.stamp.nanosec)] = m
    sub = node.create_subscription(LaserScan,'/offline/scan',cb,qos_profile_sensor_data)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag),storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr','cdr'))
    frames = 0
    try:
        with (args.output/'converter.log').open('w') as log, (args.output/'scans.jsonl').open('w') as out:
            proc = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                    start_new_session=True)
            deadline=time.monotonic()+20
            while pub.get_subscription_count()==0:
                if proc.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Converter unavailable; see converter.log')
                rclpy.spin_once(node,timeout_sec=.1)
            while reader.has_next():
                topic,data,t = reader.read_next()
                if topic != '/LIDAR/POINTS': continue
                cloud=deserialize_message(data,PointCloud2)
                key=(cloud.header.stamp.sec,cloud.header.stamp.nanosec)
                if frames==0:
                    print('CLOUD_FIELDS',[(f.name,f.offset,f.datatype) for f in cloud.fields],flush=True)
                    points=point_cloud2.read_points(cloud,field_names=['x','y','z'],skip_nans=True)
                    xyz=np.asarray(xyz_rows(points, cloud.fields), dtype=float).reshape(-1, 3)
                    np.save(args.output/'first_cloud_xyz.npy',xyz)
                pub.publish(cloud)
                deadline=time.monotonic()+10
                retry=time.monotonic()+1
                while key not in received:
                    if time.monotonic()>deadline:
                        raise RuntimeError('No matching converted scan: '+str(key))
                    rclpy.spin_once(node,timeout_sec=.05)
                    if time.monotonic()>retry and key not in received:
                        pub.publish(cloud)
                        retry=time.monotonic()+1
                s=received.pop(key)
                out.write(json.dumps(dict(bag_ns=t,stamp_sec=s.header.stamp.sec,
                    stamp_nsec=s.header.stamp.nanosec,frame=s.header.frame_id,
                    angle_min=s.angle_min,angle_increment=s.angle_increment,
                    range_min=s.range_min,range_max=s.range_max,
                    ranges=[float(r) if np.isfinite(r) else None for r in s.ranges]),allow_nan=False)+'\n')
                frames+=1
                if frames%100==0: print('CONVERTED',frames,flush=True)
            print('TOTAL_CONVERTED',frames,flush=True)
            (args.output/'conversion.json').write_text(json.dumps(dict(frames=frames,
                bag=str(args.bag),configuration=config,domain=187,localhost_only=True),indent=2))
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try: proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
