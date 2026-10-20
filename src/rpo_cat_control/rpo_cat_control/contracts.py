"""Pure checks for the pinned RPO ten-arm-joint command contract."""
from pathlib import Path
import numpy as np
import yaml


def profile(path):
    data=yaml.safe_load(Path(path).read_text())
    if len(data['joint_names'])!=23 or len(set(data['joint_names']))!=23:
        raise ValueError('RPO requires 23 unique logical joint names')
    if data['upper_joint_names']!=data['joint_names'][13:]:
        raise ValueError('Upper joints must be the last 10 logical joints')
    return data


def check_upper(values, cfg):
    q=np.asarray(values,dtype=float)
    if q.shape!=(10,) or not np.isfinite(q).all():
        raise ValueError('Expected 10 finite absolute joint angles in radians')
    bounds=np.asarray(cfg['joint_limits']).reshape(23,2)[13:]
    if np.any(q<bounds[:,0]) or np.any(q>bounds[:,1]):
        raise ValueError('Upper joint target exceeds the pinned RPO limits')
    return q


def load_motion(path,cfg):
    with np.load(path,allow_pickle=False) as data:
        names=data['upper_joint_names'].tolist()
        times=np.array(data['time_s'],dtype=float)
        values=np.array(data['upper_ref'],dtype=float)
    if names!=cfg['upper_joint_names']:
        raise ValueError('Motion uses a different joint order')
    if times.ndim!=1 or len(times)<2 or values.shape!=(len(times),10):
        raise ValueError('Malformed trajectory arrays')
    if not np.isfinite(times).all() or times[0]!=0. or np.any(np.diff(times)<=0):
        raise ValueError('Time must be finite, start at zero, and increase strictly')
    for q in values:check_upper(q,cfg)
    return times,values


def interpolate(times,values,t):
    return np.array([np.interp(t,times,values[:,i]) for i in range(10)])


def rate_step(previous,target,dt,limit):
    if not (0.<dt<=.1 and np.isfinite(dt) and 0.<limit<=2.):
        raise ValueError('Invalid control interval or rate limit')
    return previous+np.clip(target-previous,-limit*dt,limit*dt)


def check_sdk_config(path):
    data=yaml.safe_load(Path(path).read_text())
    motors=data.get('motors',{});robot=data.get('robot',{})
    if robot.get('type')!='rpo' or len(motors.get('motor_id',[]))!=23 or sum(motors.get('motor_num',[]))!=23:
        raise ValueError('SDK config must describe the 23-motor RPO')
    if sorted(robot.get('urdf2motor',[]))!=list(range(23)):
        raise ValueError('SDK logical-to-motor map must be a 23-entry permutation')
    return data
