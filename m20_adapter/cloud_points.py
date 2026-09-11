"""Normalize ROS cloud-reader rows without depending on its ROS-version container."""


def xyz_rows(points, fields):
    """Accept Humble structured arrays or Foxy generators in field-offset order."""
    selected = sorted((f for f in fields if f.name in ('x', 'y', 'z')),
                      key=lambda f: f.offset)
    if len(selected) != 3 or {f.name for f in selected} != {'x', 'y', 'z'} or any(
            f.count != 1 for f in selected):
        raise ValueError('Cloud must contain scalar x, y and z fields')
    if getattr(getattr(points, 'dtype', None), 'names', None):
        return [(float(p['x']), float(p['y']), float(p['z'])) for p in points.reshape(-1)]
    indices = [next(i for i, f in enumerate(selected) if f.name == name) for name in ('x', 'y', 'z')]
    return [tuple(float(p[i]) for i in indices) for p in points]
