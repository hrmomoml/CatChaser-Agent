import unittest
import sys
from pathlib import Path
import tempfile
import yaml
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rpo_cat_control'))
from rpo_cat_control.contracts import profile,check_upper,load_motion,rate_step,check_sdk_config


class Contracts(unittest.TestCase):
    def setUp(self):self.cfg=profile(ROOT/'config/rpo_23.yaml')
    def test_reject_malformed_and_nonfinite_commands(self):
        for value in [[0.]*9,[0.]*23,[float('nan')]*10,[float('inf')]*10]:
            with self.assertRaises(ValueError):check_upper(value,self.cfg)
    def test_enforce_hardware_joint_bounds(self):
        q=self.cfg['default_position'][13:];check_upper(q,self.cfg)
        q[6]=1.
        with self.assertRaises(ValueError):check_upper(q,self.cfg)
    def test_wrong_robot_sdk_configuration_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'robot.yaml'
            good=dict(robot=dict(type='rpo',urdf2motor=list(range(23))),motors=dict(motor_id=list(range(1,24)),motor_num=[6,7,5,5]))
            p.write_text(yaml.safe_dump(good));check_sdk_config(p)
            good['robot']['type']='rp1';p.write_text(yaml.safe_dump(good))
            with self.assertRaises(ValueError):check_sdk_config(p)
            good['robot']['type']='rpo';good['motors']['motor_id'].append(24);p.write_text(yaml.safe_dump(good))
            with self.assertRaises(ValueError):check_sdk_config(p)

    def test_rate_limiter_prevents_initial_jump(self):
        previous=np.zeros(10);target=np.ones(10)
        q=rate_step(previous,target,.02,.5)
        np.testing.assert_allclose(q,np.full(10,.01))
    def test_reject_reordered_motion_and_bad_timestamps(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'motion.npz'
            for names,times in [(list(reversed(self.cfg['upper_joint_names'])),[0.,1.]),(self.cfg['upper_joint_names'],[0.,0.])]:
                np.savez(p,upper_joint_names=np.array(names),time_s=times,upper_ref=np.zeros((2,10)))
                with self.assertRaises(ValueError):load_motion(p,self.cfg)

if __name__=='__main__':unittest.main()
