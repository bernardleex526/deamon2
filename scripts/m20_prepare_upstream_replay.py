#!/usr/bin/env python3
"""Mechanical offline normalization of upstream geometry; never edits checkout."""
import argparse
from pathlib import Path
import re
import shutil
import subprocess

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('checkout',type=Path)
p.add_argument('output',type=Path)
a=p.parse_args()
if a.output.exists():
    p.error('output must be a new directory')
a.output.mkdir(parents=True)
for name in ('lidar_tracker.hpp','common_types.hpp','kalman_filter.hpp'):
    shutil.copy2(a.checkout/'include'/name,a.output/name)
path=a.output/'common_types.hpp'
text=path.read_text()
for key,value in dict(FOLLOW_DIST='1.2',RECTANGLE_WIDTH='0.8',ROBOT_FRAME_FRONT='0.41',
                      ROBOT_FRAME_BACK='0.41',ROBOT_FRAME_LEFT='0.253',ROBOT_FRAME_RIGHT='0.253').items():
    text,count=re.subn(r'(constexpr double '+key+r'\s*=\s*)[0-9.]+',lambda m:m[1]+value,text)
    if count!=1: raise RuntimeError('Unknown upstream constant: '+key)
path.write_text(text)
sha=subprocess.check_output(['git','-C',str(a.checkout),'rev-parse','HEAD'],text=True).strip()
(a.output/'NORMALIZATION.txt').write_text('Upstream '+sha+'\nGeometry and follow distance normalized only.\n'
    'Replay adds pi to scan.angle_min to cancel upstream negated XY.\n')
print(sha)
