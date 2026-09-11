"""Validate the coupled M20 controller/guard configuration before launch."""
import math


def validate_follow_profile(config, live=False):
    tracker = config.get('robot_nexus', {}).get('ros__parameters', {})
    bridge = config.get('m20_bridge', {}).get('ros__parameters', {})
    if not tracker.get('gait_aware_follow', False):
        if live:
            raise ValueError('M20 live requires the gait-aware, latched-target profile')
        return
    if bridge.get('expected_gait') != 4097 or bridge.get('require_tracking_status') is not True:
        raise ValueError('Basic-gait controller requires gait 4097 and tracking status interlock')
    if tracker.get('minimum_target_points', 0) < 3:
        raise ValueError('M20 requires at least three target scan points')
    age = tracker.get('tracking_timeout', .3)
    if type(age) not in (float, int) or not math.isfinite(age) or not 0 < age <= .5:
        raise ValueError('Tracking freshness must be in (0, 0.5] seconds')
    if age != bridge.get('tracking_timeout', .3):
        raise ValueError('Tracker and bridge must use the same acquisition-age budget')
    for axis, low in [('vx', .20), ('wz', .50)]:
        values = [tracker.get('min_follow_' + axis), tracker.get('max_follow_' + axis),
                  bridge.get('max_' + axis)]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError('Missing or invalid speed configuration: ' + axis)
        minimum, maximum, limit = values
        if not low <= minimum <= maximum <= limit:
            raise ValueError('Controller speed outside the basic-gait/bridge limits: ' + axis)
