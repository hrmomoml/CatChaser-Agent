#!/usr/bin/env python3
"""Export reference joint angles from the existing RPO MuJoCo controller, not torques."""
import argparse
from contextlib import redirect_stdout
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import yaml


def export(sequence,source,repo,out,cfg):
    sys.path.insert(0,str(source.parent))
    spec=importlib.util.spec_from_file_location('rpo_source_cat_play',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    commands=module.parse_command(sequence)
    if not isinstance(commands,list) or not commands:raise ValueError('Expected a motion sequence')
    lower=any(c.posture is not None for c in commands)
    demo=module.Demo(repo,.15,2.1,10.)
    names=[demo.m.joint(int(j)).name for j in demo.joints]
    if names!=cfg['joint_names']:raise ValueError('Source actuator order differs from pinned logical order')
    times=[];refs=[];actual=[]
    demo.enqueue(commands)
    with (out.with_suffix('.log')).open('w') as log,redirect_stdout(log):
        while demo.d.time<180.:
            demo.step()
            if int(round(demo.d.time/demo.m.opt.timestep))%10==0:
                times.append(float(demo.d.time));refs.append(demo.ref.copy());actual.append(demo.d.qpos[demo.q].copy())
            if not demo.pending and demo.active_action is None and demo.d.time>demo.next_action_time+1.:break
    if demo.failure or demo.pending or demo.active_action is not None:raise RuntimeError('Simulation did not complete')
    t=np.array(times);t-=t[0];q=np.array(refs);qa=np.array(actual)
    upper=q[:,13:].copy()
    # Keep the inactive left arm at the official policy default, rather than zeroing it.
    upper[:,:5]=np.asarray(cfg['default_position'])[13:18]
    bounds=np.asarray(cfg['joint_limits']).reshape(23,2)
    full_bad=np.any((q<bounds[:,0])|(q>bounds[:,1]),axis=0)
    upper_bad=np.any((upper<bounds[13:,0])|(upper>bounds[13:,1]),axis=0)
    if upper_bad.any():raise ValueError('Exported upper-body path exceeds hardware-profile limits')
    np.savez_compressed(out,time_s=t,joint_names=np.array(names),upper_joint_names=np.array(names[13:]),
                        q_ref_full_source=q,q_actual_source=qa,upper_ref=upper,
                        upper_velocity=np.gradient(upper,t,axis=0))
    info=dict(sequence=sequence,format_version=1,rate_hz=50,duration_s=float(t[-1]),
              source_controller_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              source_model_sha256=hashlib.sha256((repo/'mjcf/rpo.xml').read_bytes()).hexdigest(),
              arrays='References are absolute radian positions; MuJoCo torques/gains are not exported.',
              lower_body_requested=lower,upper_only_compatible=not lower,
              full_reference_limit_violations=[names[i] for i in np.flatnonzero(full_bad)],
              hardware_validated=False,self_collision_validated=False,
              whole_body_hardware_replay_allowed=False,
              notes=['Known shin/foot mesh intersections remain unresolved.',
                     'Upper-body references are candidates for slow supported-robot trials, not hardware validation.',
                     'Legs and waist must remain owned by the interrupt policy for the upper-body path.'])
    out.with_suffix('.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n')
    return info


def main():
    p=argparse.ArgumentParser(description=__doc__)
    root=Path(__file__).resolve().parents[1]
    p.add_argument('--source',type=Path,default=root/'sim_source/cat_play.py')
    p.add_argument('--repo',type=Path,default=root.parent/'rpo_description')
    p.add_argument('--profile',type=Path,default=root/'config/rpo_23.yaml')
    p.add_argument('--output',type=Path,default=root/'artifacts')
    p.add_argument('--sequence');p.add_argument('--library',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    cfg=yaml.safe_load(a.profile.read_text())
    if not a.library and not a.sequence:p.error('Use --library or --sequence')
    sequences=[str(n) for n in range(3,9)] if a.library else [a.sequence]
    for sequence in sequences:
        filename='motion_'+sequence.replace('+','_plus_').replace(' ','_then_')+'.npz'
        info=export(sequence,a.source,a.repo,a.output/filename,cfg)
        print(filename,'upper_only=',info['upper_only_compatible'],'full_limit_violations=',info['full_reference_limit_violations'],flush=True)

if __name__=='__main__':main()
