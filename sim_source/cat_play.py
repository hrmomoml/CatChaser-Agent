"""RPO posture and arm channels: 1 crouches, 2 stands up, plus combines simultaneous motions."""
import argparse
from collections import deque
import queue
import threading
import json
import hashlib
import os
import select
import sys
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from motion_primitives import ACTION_NAMES, Command, single_command, displacement, parse_command, smoothstep


def build(repo, floating=True, self_collision=False, collision_geometry="hull"):
    source = repo / 'mjcf/rpo.xml'
    root = ET.parse(source).getroot()
    root.set('model', 'rpo-cat-play')
    root.find('compiler').set('meshdir', str((repo/'meshes').resolve()))
    root.find('option').set('timestep', '0.002')
    root.find('option').set('integrator', 'implicitfast')
    root.find('option').set('solver', 'Newton')
    base = root.find(".//body[@name='base_link']")
    if not floating:
        base.remove(base.find("joint[@type='free']"))
    floor = root.find(".//geom[@name='ground']")
    floor.set('name', 'cat_play_floor')
    for geom in root.findall(".//geom[@type='mesh']"):
        geom.set('group', '2')
        geom.set('name', geom.get('mesh')+'_collision')
        if self_collision:
            geom.set('contype', '1')
            geom.set('conaffinity', '1')
    for side in ('left', 'right'):
        foot = root.find(f".//body[@name='{side}_ankle_roll_link']/geom")
        foot.set('name', side+'_foot_collision')
    hand = root.find(".//body[@name='right_elbow_yaw_link']")
    ET.SubElement(hand, 'geom', name='toy_wand', type='capsule', fromto='.10 0 0 .39 0 0',
                  size='.006', mass='.015', contype='0', conaffinity='0', group='0', rgba='1 .65 .1 1')
    ET.SubElement(hand, 'site', name='toy_tip', pos='.39 0 0', size='.022', rgba='1 .15 .1 1')
    if self_collision:
        hand.find("geom[@name='toy_wand']").set('contype', '1')
        hand.find("geom[@name='toy_wand']").set('conaffinity', '1')
    if self_collision and collision_geometry == 'refined':
        directory = Path(__file__).resolve().parent/'collision_meshes_refined'
        manifest = json.loads((directory/'manifest.json').read_text())
        for name, entry in manifest['meshes'].items():
            if hashlib.sha256((repo/'meshes'/(name+'.STL')).read_bytes()).hexdigest() != entry['source_sha256']:
                raise ValueError('Refined collision mesh is stale: '+name)
            body = root.find(f".//body[@name='{name}']")
            original = body.find('geom')
            original.set('contype','0')
            original.set('conaffinity','0')
            prefix = original.get('name')
            for index, part in enumerate(entry['parts']):
                mesh_name = name+'_part_'+str(index)
                ET.SubElement(root.find('asset'), 'mesh', name=mesh_name,
                              file=str((directory/part['file']).resolve()))
                ET.SubElement(body, 'geom', name=prefix+'__part_'+str(index),
                              type='mesh', mesh=mesh_name, contype='1', conaffinity='1',
                              mass='0', group='3', rgba='.9 .35 .15 .35')
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


class Demo:
    def __init__(self, repo, amplitude, frequency, duration, floating=True, self_collision=False, collision_geometry="hull"):
        self.m = build(repo, floating, self_collision, collision_geometry)
        self.collision_geometry = collision_geometry
        self.self_collision_enabled = self_collision
        self.self_contact_details = {}
        self.floating = floating
        self.d = mujoco.MjData(self.m)
        self.ik = mujoco.MjData(self.m)
        self.amplitude, self.frequency, self.duration = amplitude, frequency, duration
        self.joints = self.m.actuator_trnid[:, 0]
        self.q = self.m.jnt_qposadr[self.joints]
        self.v = self.m.jnt_dofadr[self.joints]
        self.lo = self.m.jnt_range[self.joints, 0] + .03
        self.hi = self.m.jnt_range[self.joints, 1] - .03
        arm_names = ('right_arm_pitch_joint', 'right_arm_roll_joint',
                     'right_arm_yaw_joint', 'right_elbow_pitch_joint')
        self.arm = np.array([int(np.where(self.joints == self.m.joint(name).id)[0][0]) for name in arm_names])
        self.home = np.zeros(self.m.nu)
        for name, value in [('left_arm_roll_joint', .30),
                            ('left_thigh_pitch_joint', -1.40), ('left_knee_joint', 1.55),
                            ('left_ankle_pitch_joint', -.50), ('right_thigh_pitch_joint', -1.40),
                            ('right_knee_joint', 1.55), ('right_ankle_pitch_joint', -.50),
                            ('right_arm_pitch_joint', -.567772910),
                            ('right_arm_roll_joint', .086392360),
                            ('right_arm_yaw_joint', -.088924371), ('right_elbow_pitch_joint', .270439864)]:
            self.home[np.where(self.joints == self.m.joint(name).id)[0][0]] = value
        self.d.qpos[self.q] = 0.
        mujoco.mj_forward(self.m, self.d)
        # Start with soles 1 mm above the floor; floating mode has no pelvis forces.
        floor_penetration = max([0.] + [-c.dist for c in self.d.contact
                                           if c.geom1 == self.m.geom('cat_play_floor').id or c.geom2 == self.m.geom('cat_play_floor').id])
        if floating:
            self.d.qpos[2] += floor_penetration + .001
        else:
            self.m.body_pos[self.m.body('base_link').id, 2] += floor_penetration + .001
        mujoco.mj_forward(self.m, self.d)
        self.site = self.m.site('toy_tip').id
        self.center = self.d.site_xpos[self.site].copy()
        self.ref = np.zeros(self.m.nu)
        self.refvel = np.zeros(self.m.nu)
        self.motion_start = 9.5
        self.center_locked = False
        self.phase = "standing"
        self.active_mode = None
        self.active_start = self.motion_start
        self.blend = 0.
        self.kp = np.full(self.m.nu, 1200.)
        self.kd = np.full(self.m.nu, 35.)
        for i, j in enumerate(self.joints):
            if any(word in self.m.joint(j).name for word in ('arm_', 'elbow')):
                self.kp[i], self.kd[i] = 180., 1.5
        self.m.dof_damping[self.v] = self.kd
        self.ik.qpos[:] = self.d.qpos
        self.initial_height = float(self.d.xpos[self.m.body('base_link').id, 2])
        self.balance_samples = deque(maxlen=10000)
        self.joint_samples = deque(maxlen=10000)
        self.failure = None
        self.fall_detected = False
        self.initial_com = self.d.subtree_com[0].copy()
        self.previous_pelvis = self.d.xpos[self.m.body("base_link").id].copy()
        self.max_pelvis_step_m = 0.
        self.self_contact_pairs = set()
        self.pending = deque()
        self.active_action = None
        self.crouched = False
        self.records = deque(maxlen=1000)
        self.action_rows = []
        self.action_start = 0.
        self.action_start_height = self.initial_height
        self.stop_start = None
        self.next_action_time = 1.5
        self.phase = 'idle_standing'
        self.max_tilt_deg = 0.
        self.min_com_margin_m = float('inf')
        self.arm_prepared = False
        self.command_duration = 0.
        self.pose_duration = 0.
        self.pose_total = 0.
        self.pose_from = 0.
        self.pose_goal = 0.
        self.arm_start = 0.
        self.arm_duration = 0.
        self.prep_start = 0.
        self.prep_duration = 0.
        self.prep_offset = np.zeros(3)
        self.action_heights = []
        self.action_blends = []
        self.overlap_seconds = 0.
        self.moving_overlap_seconds = 0.

    def enqueue(self, actions):
        actions = [single_command(action) if isinstance(action, int) else action for action in actions]
        if any(not isinstance(action, Command) for action in actions):
            raise ValueError('需要动作编号或姿态+摆臂组合。')
        if len(self.pending)+len(actions)>100:
            raise ValueError('动作队列过长，请等待当前动作完成。')
        # A single arm key may join a posture that is running or waiting to start.
        if len(actions)==1 and actions[0].posture is None:
            arm = actions[0].arm
            if self.active_action is not None and self.active_action.posture is not None and self.active_action.arm is None:
                self.active_action = Command(self.active_action.posture, arm)
                self.prepare_arm()
                elapsed = self.d.time-self.action_start
                self.command_duration = max(self.command_duration, elapsed+self.prep_duration+max(self.duration, 6. if arm == 8 else 0.))
                self.arm_duration = self.action_start+self.command_duration-self.arm_start
                print(f'同时执行：{self.active_action}', flush=True)
                return 'joined'
            if self.pending and self.pending[-1].posture is not None and self.pending[-1].arm is None:
                self.pending[-1] = Command(self.pending[-1].posture, arm)
                print(f'已合并同时指令：{self.pending[-1]}', flush=True)
                return 'joined'
        self.pending.extend(actions)
        return 'queued'

    def prepare_arm(self):
        self.prep_start = self.d.time
        self.prep_duration = 0. if self.arm_prepared else 1.5
        root = self.d.xpos[self.m.body('base_link').id]
        self.prep_offset = self.d.site_xpos[self.site].copy()-root
        self.arm_start = self.d.time+self.prep_duration

    def stop(self):
        self.pending.clear()
        if self.active_action is None:
            print('队列已清空，保持当前姿态。', flush=True)
            return
        if self.stop_start is None:
            self.stop_start = self.d.time
            elapsed = self.d.time-self.action_start
            self.command_duration = max(self.pose_total, elapsed+.8)
            print('摆臂平滑停止；完成当前蹲下/站起过渡后保持姿态。', flush=True)

    def finish_action(self, cancelled=False):
        command = self.active_action
        record = dict(command=str(command), name=command.name, posture=command.posture, arm=command.arm,
                      start_s=float(self.action_start), end_s=float(self.d.time),
                      completed=not cancelled, cancelled=cancelled,
                      pelvis_height_start_m=self.action_start_height,
                      pelvis_height_end_m=float(self.d.xpos[self.m.body('base_link').id, 2]),
                      pelvis_height_range_m=[min(self.action_heights), max(self.action_heights)] if self.action_heights else [],
                      posture_blend_range=[min(self.action_blends), max(self.action_blends)] if self.action_blends else [],
                      simultaneous_seconds=self.overlap_seconds,
                      moving_posture_arm_seconds=self.moving_overlap_seconds)
        if self.action_rows:
            rows = np.array(self.action_rows)
            relative_actual = rows[:, 4:7]-rows[:, 7:10]
            relative_target = rows[:, 1:4]-rows[:, 7:10]
            record.update(tip_peak_to_peak_m=np.ptp(rows[:, 4:7], axis=0).tolist(),
                          relative_tip_peak_to_peak_m=np.ptp(relative_actual, axis=0).tolist(),
                          relative_target_peak_to_peak_m=np.ptp(relative_target, axis=0).tolist(),
                          tracking_rms_m=float(np.sqrt(np.mean(np.sum((rows[:, 1:4]-rows[:, 4:7])**2, axis=1)))),
                          max_tracking_error_m=float(np.max(np.linalg.norm(rows[:, 1:4]-rows[:, 4:7], axis=1))))
            if command.arm in (3, 4, 5):
                axis = 2 if command.arm == 3 else 1
                record['frequency_hz'] = min(self.frequency, 1.2) if command.arm == 5 else self.frequency
                signal = relative_actual[:, axis]
                mid = .5*(signal.min()+signal.max())
                crossings = rows[np.flatnonzero((signal[:-1]<mid)&(signal[1:]>=mid))+1, 0]
                record['measured_frequency_hz'] = float(1/np.mean(np.diff(crossings))) if len(crossings)>1 else None
            if command.arm == 5:
                radial = np.linalg.norm(relative_actual[:, 1:3], axis=1)
                record['mean_circle_radius_m'] = float(radial.mean())
                record['circle_radius_rms_error_m'] = float(np.sqrt(np.mean((radial-self.amplitude)**2)))
        self.records.append(record)
        print(('已停止 ' if cancelled else '完成 ')+str(command)+'：'+command.name, flush=True)
        self.active_action = None
        self.action_rows = []
        self.action_heights = []
        self.action_blends = []
        self.stop_start = None
        self.next_action_time = self.d.time+.5

    def start_next(self):
        if self.active_action is not None or not self.pending or self.d.time < self.next_action_time:
            return
        self.active_action = self.pending.popleft()
        self.action_start = self.d.time
        self.action_start_height = float(self.d.xpos[self.m.body('base_link').id, 2])
        self.action_rows = []
        self.action_heights = []
        self.action_blends = []
        self.overlap_seconds = 0.
        self.moving_overlap_seconds = 0.
        self.pose_from = self.blend
        posture = self.active_action.posture
        self.pose_goal = (1. if posture == 1 else 0.) if posture is not None else self.blend
        changing = abs(self.pose_goal-self.pose_from)>.001
        self.pose_duration = 6. if changing else .5
        self.pose_total = self.pose_duration+(2. if changing else 0.) if posture is not None else 0.
        self.command_duration = self.pose_total
        if self.active_action.arm is not None:
            self.prepare_arm()
            self.command_duration = max(self.command_duration, self.prep_duration+max(self.duration, 6. if self.active_action.arm == 8 else 0.))
            self.arm_duration = self.command_duration-self.prep_duration
        print('开始 '+str(self.active_action)+'：'+self.active_action.name, flush=True)

    def solve_ik(self, target):
        jp = np.zeros((3, self.m.nv))
        rest = self.home[self.arm].copy()
        rest[1] = -.20  # Keep the upper arm away from the side of the torso.
        upper = self.hi[self.arm].copy()
        upper[1] = min(upper[1], .20)
        for iteration in range(40):
            mujoco.mj_forward(self.m, self.ik)
            error = target-self.ik.site_xpos[self.site]
            if np.linalg.norm(error) < 1e-5 and iteration > 0:
                break
            mujoco.mj_jacSite(self.m, self.ik, jp, None, self.site)
            jac = jp[:, self.v[self.arm]]
            inverse = jac.T@np.linalg.inv(jac@jac.T+1e-5*np.eye(3))
            current = self.ik.qpos[self.q[self.arm]]
            dq = inverse@error
            if iteration == 0:
                secondary = (np.eye(len(self.arm))-inverse@jac)@(.06*(rest-current))
                dq += np.clip(secondary, -.002, .002)
            self.ik.qpos[self.q[self.arm]] = np.clip(
                current+np.clip(dq, -.04, .04), self.lo[self.arm], upper)

    def step(self):
        self.start_next()
        t = self.d.time
        elapsed = t-self.action_start
        command = self.active_action
        previous_blend = self.blend
        if command is not None and command.posture is not None:
            self.blend = self.pose_from+(self.pose_goal-self.pose_from)*float(smoothstep(elapsed/self.pose_duration))
        self.crouched = self.blend>.99
        self.ik.qpos[:] = self.d.qpos
        self.ik.qpos[self.q] = self.home*self.blend
        if command is not None:
            self.phase = str(command)
            self.action_heights.append(float(self.d.xpos[self.m.body('base_link').id, 2]))
            self.action_blends.append(float(self.blend))
        else:
            self.phase = 'idle_crouched' if self.crouched else 'idle_standing'
        arm = command.arm if command is not None else None
        if self.arm_prepared or arm is not None:
            self.ik.qpos[self.q[self.arm]] = self.ref[self.arm]
            root = self.d.xpos[self.m.body('base_link').id]
            # The front workspace follows pelvis height throughout the posture change.
            offset = np.array([.43+.08*self.blend, -.22+.10*self.blend, -.02])
            if not self.arm_prepared:
                progress = float(smoothstep((t-self.prep_start)/self.prep_duration))
                offset = (1.-progress)*self.prep_offset+progress*offset
                if progress >= 1.:
                    self.arm_prepared = True
            self.center = root+offset
            delta = np.zeros(3)
            arm_elapsed = t-self.arm_start
            if arm is not None and arm_elapsed >= 0.:
                delta = displacement(arm, arm_elapsed, self.arm_duration, self.amplitude, self.frequency)
                if self.stop_start is not None:
                    delta *= 1.-smoothstep((t-self.stop_start)/.8)
                if command.posture is not None:
                    self.overlap_seconds += self.m.opt.timestep
                    if abs(self.blend-previous_blend)>1e-8:
                        self.moving_overlap_seconds += self.m.opt.timestep
            target = self.center+delta
            self.solve_ik(target)
        else:
            mujoco.mj_forward(self.m, self.ik)
            target = self.ik.site_xpos[self.site].copy()
        new_ref = self.ik.qpos[self.q].copy()
        refvel = (new_ref - self.ref) / self.m.opt.timestep
        refacc = np.clip((refvel-self.refvel)/self.m.opt.timestep, -400., 400.)
        self.refvel = refvel.copy()
        self.ref = new_ref
        mujoco.mj_forward(self.m, self.d)
        # Inertia feedforward follows the rapid reference without excessive stiffness.
        desired_acc = np.zeros(self.m.nv)
        desired_acc[self.v[self.arm]] = refacc[self.arm]
        inertia_feedforward = np.zeros(self.m.nv)
        mujoco.mj_mulM(self.m, self.d, inertia_feedforward, desired_acc)
        # Joint damping is integrated implicitly by MuJoCo.
        torque = (self.kp*(self.ref-self.d.qpos[self.q])
                  + self.kd*refvel + self.d.qfrc_bias[self.v]+inertia_feedforward[self.v])
        # Ankle feedback counters torso pitch/roll during fast arm motion.
        R = self.d.xmat[self.m.body('base_link').id].reshape(3, 3)
        pitch = np.arctan2(-R[2, 0], R[2, 2])
        roll = np.arctan2(R[2, 1], R[2, 2])
        omega = self.d.qvel[3:6] if self.floating else np.zeros(3)
        for side in ['left', 'right']:
            ai = self.m.actuator(side+'_ankle_pitch_joint').id
            torque[ai] += 100.*(pitch-.35*self.blend)+18.*omega[1]
            ai = self.m.actuator(side+'_ankle_roll_joint').id
            torque[ai] += 100.*roll+18.*omega[0]
        self.d.ctrl[:] = np.clip(torque, self.m.actuator_ctrlrange[:, 0], self.m.actuator_ctrlrange[:, 1])
        mujoco.mj_step(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        if not np.isfinite(self.d.qpos).all() or np.any(self.d.warning.number):
            raise RuntimeError('Simulation became unstable')
        if np.any(self.d.qpos[self.q] < self.lo-.02) or np.any(self.d.qpos[self.q] > self.hi+.02):
            raise RuntimeError('Joint limit exceeded')
        # Check physical falls every step, independently of arm tracking.
        base_id = self.m.body('base_link').id
        self.max_pelvis_step_m = max(self.max_pelvis_step_m, float(np.linalg.norm(self.d.xpos[base_id]-self.previous_pelvis)))
        self.previous_pelvis = self.d.xpos[base_id].copy()
        tilt = np.arccos(np.clip(self.d.xmat[base_id].reshape(3, 3)[2, 2], -1., 1.))
        floor = self.m.geom('cat_play_floor').id
        for c_index, c in enumerate(self.d.contact):
            if floor not in (c.geom1, c.geom2):
                pair = tuple(sorted((self.m.geom(c.geom1).name, self.m.geom(c.geom2).name)))
                self.self_contact_pairs.add(pair)
                force = np.zeros(6)
                mujoco.mj_contactForce(self.m, self.d, c_index, force)
                info = self.self_contact_details.setdefault(pair, dict(
                    first_time_s=float(self.d.time), min_distance_m=0., max_normal_force_n=0.))
                info['min_distance_m'] = min(info['min_distance_m'], float(c.dist))
                info['max_normal_force_n'] = max(info['max_normal_force_n'], float(abs(force[0])))
                if self.self_collision_enabled and (c.dist < -.001 or abs(force[0]) > 10.):
                    self.failure = 'Self collision: '+' / '.join(pair)
            if floor in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == floor else c.geom1
                if 'foot_' not in self.m.geom(other).name:
                    self.failure = 'Non-foot body touched the floor'
                    self.fall_detected = True
        if tilt > np.radians(55) or self.d.xpos[base_id, 2] < .30:
            self.failure = 'Excessive body tilt or pelvis height loss'
            self.fall_detected = True
        if self.failure:
            raise RuntimeError(self.failure)
        # Slower diagnostics do not affect the physics timestep.
        if int(round(self.d.time/self.m.opt.timestep)) % 10 == 0:
            from balance import balance
            snapshot = balance(self)
            snapshot.update(time_s=float(self.d.time), phase=self.phase)
            self.max_tilt_deg = max(self.max_tilt_deg, snapshot['lean_deg'])
            self.min_com_margin_m = min(self.min_com_margin_m, snapshot['com_margin'])
            self.balance_samples.append(snapshot)
            self.joint_samples.append(self.d.qpos[self.q[self.arm]].copy())
        actual = self.d.site_xpos[self.site].copy()
        if command is not None and command.arm is not None:
            arm_elapsed = t-self.arm_start
            if .8 < arm_elapsed < self.arm_duration-.8:
                self.action_rows.append([self.d.time, *target, *actual, *self.center])
        if command is not None and elapsed >= self.command_duration:
            self.finish_action(cancelled=self.stop_start is not None)
        return target, actual



HELP = """
RPO 模型指令控制
1 蹲下并保持    2 站起并保持
3 上下摆动    4 左右摆动    5 画圆
6 快速前伸—停顿—慢速收回    7 从左到右画正弦波
8 左右快扫（两侧各停 2 秒）
加号同时执行：1+3（边蹲边上下摆棒）、2+5（边站起边画圆）。
空格依次执行：1 3 2 4。一次只选一种姿态和一种摆臂。
姿态动作进行时直接按 3～8，也可加入一种摆臂动作。
0 清队列并平滑停止；q 退出；h 帮助。
"""


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sequence', help='Batch sequence, e.g. "1+3 2+5"')
    p.add_argument('--motion', choices=['vertical', 'horizontal', 'both'], help='Legacy automatic demo')
    p.add_argument('--amplitude', type=float, default=.15, help='One-sided tip amplitude, meters')
    p.add_argument('--frequency', type=float, default=2.1, help='Cycles per second')
    p.add_argument('--duration', type=float, default=10., help='Seconds per arm action')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--self-collision', action='store_true',
                   help='Enable robot/wand physical self-collisions and stop at >1 mm penetration or >10 N contact')
    p.add_argument('--collision-geometry', choices=['hull','refined'], default='refined',
                   help='Collision geometry used with --self-collision; refined uses local convex covers of original triangles')
    p.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[1])
    p.add_argument('--report', type=Path)
    a = p.parse_args()
    if not (.005 <= a.amplitude <= .15 and .1 <= a.frequency <= 2.1 and 3 <= a.duration <= 120):
        p.error('Use amplitude .005–.15 m, frequency .1–2.1 Hz, duration 3–120 s')
    if a.sequence and a.motion:
        p.error('--sequence and --motion cannot be combined')
    batch = a.sequence is not None or a.motion is not None
    if a.headless and not batch:
        p.error('--headless requires --sequence or --motion')
    demo = Demo(a.repo, a.amplitude, a.frequency, a.duration, self_collision=a.self_collision, collision_geometry=a.collision_geometry)
    if a.sequence is not None:
        try:
            actions = parse_command(a.sequence)
            if not isinstance(actions, list) or not actions:
                raise ValueError('批处理需要至少一个动作编号。')
        except ValueError as error:
            p.error(str(error))
        demo.enqueue(actions)
    elif a.motion:
        demo.enqueue(parse_command({'vertical': '1+3', 'horizontal': '1+4', 'both': '1+3 2+4'}[a.motion]))
    events = queue.Queue(maxsize=256)
    def emit(value):
        try:
            events.put_nowait(value)
        except queue.Full:
            print('输入过快，请等待队列执行。', flush=True)
    input_shutdown = threading.Event()
    input_thread = None
    def read_input():
        # Poll the file descriptor so window-close can stop and join this thread.
        buffered = b''
        print('动作> ', end='', flush=True)
        try:
            descriptor = sys.stdin.fileno()
            while not input_shutdown.is_set():
                readable, _, _ = select.select([descriptor], [], [], .2)
                if not readable:
                    continue
                chunk = os.read(descriptor, 4096)
                if not chunk:
                    if buffered:
                        emit(buffered.decode('utf-8', errors='replace'))
                    return
                buffered += chunk
                while b'\n' in buffered:
                    line, buffered = buffered.split(b'\n', 1)
                    command = line.decode('utf-8', errors='replace').strip()
                    emit(command)
                    if command.lower() in ('q', 'quit', 'exit'):
                        return
                    print('动作> ', end='', flush=True)
        except (OSError, ValueError):
            return
    def keyboard(key):
        if 49 <= key <= 56:
            emit(chr(key))
        elif key == 48:
            emit('0')
        elif key in (81, 113):
            emit('q')
    if not batch:
        print(HELP, flush=True)
        input_thread = threading.Thread(target=read_input, daemon=True)
        input_thread.start()
    quit_requested = False
    wall_start = time.monotonic()
    phases = []
    previous_phase = None
    def run(viewer=None):
        nonlocal quit_requested, previous_phase
        playback_start = time.monotonic()
        steps = 0
        previous_hud = None
        while not quit_requested and (viewer is None or viewer.is_running()):
            while not events.empty():
                try:
                    command = parse_command(events.get_nowait())
                    if command == 'quit':
                        quit_requested = True
                    elif command == 'stop':
                        demo.stop()
                    elif command == 'help':
                        print(HELP, flush=True)
                    elif command:
                        result = demo.enqueue(command)
                        if result != 'joined':
                            print('已排队：'+' → '.join(map(str, command)), flush=True)
                except ValueError as error:
                    print(error, flush=True)
            if quit_requested:
                break
            try:
                demo.step()
            except RuntimeError as error:
                demo.failure = str(error)
                print(f'Stopped: {error}', flush=True)
                break
            if demo.phase != previous_phase:
                phases.append(dict(phase=demo.phase, time_s=float(demo.d.time)))
                if len(phases)>1000:
                    phases.pop(0)
                previous_phase = demo.phase
            steps += 1
            if viewer is not None and steps % 8 == 0:
                hud = (demo.active_action, demo.phase, tuple(demo.pending))
                if hud != previous_hud:
                    active = str(demo.active_action) if demo.active_action is not None else 'Idle'
                    pending = ' '.join(map(str, list(demo.pending)[:12])) or '-'
                    viewer.set_texts((mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                        ('RPO | Self collision: '+('ON' if demo.self_collision_enabled else 'OFF')+'\n1  Crouch   2  Stand up\n3  Up/down  4  Left/right\n5  Circle   6  Reach/pause/return\n7  Sine sweep   8  Sweep/pause\nUse 1+3 or 2+5 in terminal\n0  Stop    Q  Quit\nActive: '+active+'\nQueued: '+pending), ''))
                    previous_hud = hud
                viewer.sync()
                # Digits also toggle MuJoCo geometry groups; keep this app visible.
                with viewer.lock():
                    viewer.opt.geomgroup[:] = (1, 0, 1, 0, 0, 0)
                time.sleep(max(0., demo.d.time-(time.monotonic()-playback_start)))
            if batch and not demo.pending and demo.active_action is None and demo.d.time>demo.next_action_time+1.:
                break
    if a.headless:
        run()
    else:
        import mujoco.viewer
        threads_before_viewer = set(threading.enumerate())
        with mujoco.viewer.launch_passive(demo.m, demo.d, key_callback=keyboard) as viewer:
            viewer_threads = set(threading.enumerate())-threads_before_viewer
            viewer.cam.lookat[:] = [.2, 0, .6]
            viewer.cam.distance = 2.3
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -15
            run(viewer)
        # Let native rendering tear down before Python's GLFW atexit handler.
        for thread in viewer_threads:
            thread.join(timeout=5.)
    input_shutdown.set()
    if input_thread is not None:
        input_thread.join(timeout=1.)
    report = dict(robot='RPO', model_file=str((a.repo/'mjcf/rpo.xml').resolve()),
                  model_sha256=hashlib.sha256((a.repo/'mjcf/rpo.xml').read_bytes()).hexdigest(),
                  actuated_joints=int(demo.m.nu), frequency_hz=a.frequency, amplitude_m=a.amplitude, base_mode='floating',
                  completed=not demo.pending and demo.active_action is None and not demo.failure and
                      all(record['completed'] for record in demo.records),
                  simulated_seconds=float(demo.d.time), wall_seconds=time.monotonic()-wall_start,
                  phases=phases, fall_detected=demo.fall_detected, failure=demo.failure,
                  initial_pelvis_height_m=demo.initial_height, max_pelvis_step_m=demo.max_pelvis_step_m,
                  self_collision_enabled=demo.self_collision_enabled,
                  collision_geometry=demo.collision_geometry if demo.self_collision_enabled else "source_masks",
                  self_contact_details=[dict(pair=list(pair), **info) for pair, info in sorted(demo.self_contact_details.items())],
                  self_contact_pairs=sorted(demo.self_contact_pairs), target_center_m=demo.center.tolist(),
                  max_body_tilt_deg=demo.max_tilt_deg,
                  min_com_footprint_margin_m=demo.min_com_margin_m if np.isfinite(demo.min_com_margin_m) else None,
                  active_action=str(demo.active_action) if demo.active_action is not None else None,
                  pending_actions=list(map(str, demo.pending)),
                  actions=list(demo.records))
    if batch:
        print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    else:
        print(f'已退出。完成 {sum(record["completed"] for record in demo.records)} 个动作。', flush=True)
    if a.report:
        a.report.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    if demo.failure:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
