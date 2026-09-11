#!/usr/bin/env python3
"""Evaluate production tracker on reconstructed real scans, without ROS or UDP."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20_adapter.core import Guard, inspect_scan


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scans',type=Path)
    p.add_argument('--tracker',type=Path,required=True)
    p.add_argument('--upstream',type=Path)
    p.add_argument('--target',nargs=2,type=float,required=True)
    p.add_argument('--gait-aware', action='store_true', help='Replay the fixed M20 basic-gait profile')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    scans=[json.loads(line) for line in a.scans.read_text().splitlines()]
    base=scans[0]['bag_ns'];payload=[]
    for s in scans:
        payload.append(' '.join(map(str,[(s['bag_ns']-base)/1e9,s['angle_min'],s['angle_increment'],
            s['range_min'],s['range_max'],len(s['ranges'])]+[-1 if r is None else r for r in s['ranges']])))
    summaries={};all_reports={}
    for label,exe in [('minimal',a.tracker),('upstream',a.upstream)]:
        if exe is None:continue
        command = [str(exe), *map(str, a.target)]
        if a.gait_aware and label == 'minimal':
            command.append('--m20')
        completed=subprocess.run(command,input='\n'.join(payload)+'\n',
                                 text=True,capture_output=True,check=True,timeout=30)
        columns=['t','target_x','target_y','raw_x','raw_y','raw_yaw']
        if label=='minimal':columns+=['target_points','corridor','apf_x','apf_y']
        rows=[dict(zip(columns,map(float,line.split(',')))) for line in completed.stdout.splitlines()]
        assert len(rows)==len(scans),(len(rows),len(scans))
        now=[0.];g=Guard(clock=lambda:now[0], require_tracking=a.gait_aware and label=='minimal',
                         tracking_timeout=.5 if a.gait_aware else .3)
        unarmed=Guard(clock=lambda:now[0])
        status=dict(MotionState=17,Gait=4097 if a.gait_aware else 12290,Charge=0,HES=0,ControlUsageMode=1,Sleep=0,Direction=0)
        for i,(r,s) in enumerate(zip(rows,scans)):
            now[0]=r['t'];arrival=s['bag_ns']/1e9
            scan=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                sec=s['stamp_sec'],nanosec=s['stamp_nsec'])),angle_min=s['angle_min'],
                angle_increment=s['angle_increment'],range_min=s['range_min'],range_max=s['range_max'],
                ranges=[math.inf if v is None else v for v in s['ranges']])
            valid,clear=inspect_scan(scan,arrival)
            # Apply elapsed-time watchdog before fresh inputs, preserving a latched fault.
            if i:g.output()
            for gate in (g,unarmed):gate.update_status(status);gate.update_scan(valid,clear)
            if a.gait_aware and label == 'minimal':
                age = max(0., arrival-s['stamp_sec']-s['stamp_nsec']/1e9)
                g.update_tracking(r['target_points'] >= 3, 1, age=age)
            if i==0:g.arm()
            raw=tuple(r[k] for k in ['raw_x','raw_y','raw_yaw'])
            g.update_command(raw);unarmed.update_command(raw)
            out=g.output();assert unarmed.output()==(0.,0.,0.)
            r.update(scan_valid=valid,scan_clear=clear,guard_x=out[0],guard_y=out[1],
                     guard_yaw=out[2],guard_reason=g.reason,armed=g.armed,
                     age_ms=(arrival-s['stamp_sec']-s['stamp_nsec']/1e9)*1000)
        with (a.output/(label+'.csv')).open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        summary=dict(frames=len(rows),duration=rows[-1]['t'],gait_aware=a.gait_aware,
            scan_invalid=sum(not r['scan_valid'] for r in rows),
            obstacle_frames=sum(r['scan_valid'] and not r['scan_clear'] for r in rows),
            initial_target=a.target,initial_arm=rows[0]['armed'],
            last_guard_reason=rows[-1]['guard_reason'],
            raw_ranges={axis:[min(r['raw_'+axis] for r in rows),max(r['raw_'+axis] for r in rows)]
                        for axis in ('x','y','yaw')},
            guarded_nonzero=sum(any(r['guard_'+k]!=0 for k in ('x','y','yaw')) for r in rows))
        if label=='minimal':
            summary['lost_frames']=sum(r['target_points']==0 for r in rows)
            summary['apf_abs_max']=max(max(abs(r['apf_x']),abs(r['apf_y'])) for r in rows)
            summary['centered_lateral_gt_0p2']=sum(abs(r['target_y'])<.1 and abs(r['raw_y'])>.2 for r in rows)
        summary['bins']=[]
        for begin in range(0,math.ceil(rows[-1]['t']),5):
            subset=[r for r in rows if begin<=r['t']<begin+5]
            summary['bins'].append(dict(start=begin,**{k:round(float(np.median([r[k] for r in subset])),4)
                for k in ('target_x','target_y','raw_x','raw_y','raw_yaw')}))
        summaries[label]=summary;all_reports[label]=rows
    (a.output/'summary.json').write_text(json.dumps(summaries,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(13,10),sharex=True)
    for label,rows in all_reports.items():
        ts=[r['t'] for r in rows]
        axes[0].plot(ts,[r['target_y'] for r in rows],label=label+' tracked Y')
        for ax,k in zip(axes[1:],('x','y','yaw')):
            ax.plot(ts,[r['raw_'+k] for r in rows],label=label+' raw',alpha=.8)
            if label=='minimal':ax.plot(ts,[r['guard_'+k] for r in rows],label='mock armed guard',color='black')
            ax.set_ylabel(k+(' rad/s' if k=='yaw' else ' m/s'))
    axes[0].set_ylabel('Tracked Y (m)')
    for ax in axes:ax.grid();ax.legend(loc='upper right')
    axes[-1].set_xlabel('Recorded time (s)');fig.suptitle('REAL cloud bag replay; no robot control; seed is not identity proof')
    fig.tight_layout();fig.savefig(a.output/'comparison.png');plt.close(fig)
    fig,axes=plt.subplots(3,4,figsize=(13,10))
    for ax,seconds in zip(axes.flat,np.linspace(0,rows[-1]['t'],12)):
        i=int(np.argmin([abs((s['bag_ns']-base)/1e9-seconds) for s in scans]));s=scans[i]
        rs=np.array([np.nan if v is None else v for v in s['ranges']]);angs=s['angle_min']+np.arange(len(rs))*s['angle_increment']
        x=rs*np.cos(angs);y=rs*np.sin(angs)
        ax.scatter(y,x,s=2,c='gray')
        for label,reports in all_reports.items():ax.scatter(reports[i]['target_y'],reports[i]['target_x'],s=35,label=label)
        ax.set_xlim(4,-4);ax.set_ylim(-1,5);ax.set_aspect('equal');ax.set_title(f'{seconds:.1f}s');ax.grid()
    axes.flat[0].legend(fontsize=7);fig.suptitle('Scan XY snapshots (forward up, robot left to image left); dots are tracker estimates')
    fig.tight_layout();fig.savefig(a.output/'scan_snapshots.png');plt.close(fig)
    print(json.dumps(summaries,indent=2))


if __name__=='__main__':main()
