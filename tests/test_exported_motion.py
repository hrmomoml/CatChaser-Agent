import json
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rpo_cat_control'))
from rpo_cat_control.contracts import profile,load_motion


class ExportedMotion(unittest.TestCase):
    @unittest.skipUnless((ROOT/'artifacts/motion_8.npz').exists(),'Export motion library first')
    def test_all_exported_upper_references_match_contract(self):
        cfg=profile(ROOT/'config/rpo_23.yaml')
        for action in range(3,9):
            path=ROOT/f'artifacts/motion_{action}.npz'
            times,values=load_motion(path,cfg)
            self.assertGreater(times[-1],10.)
            np.testing.assert_allclose(values[:,:5],np.tile(cfg['default_position'][13:18],(len(values),1)))
            meta=json.loads(path.with_suffix('.json').read_text())
            self.assertTrue(meta['upper_only_compatible'])
            self.assertFalse(meta['hardware_validated'])
            self.assertFalse(meta['whole_body_hardware_replay_allowed'])
    @unittest.skipUnless((ROOT/'artifacts/motion_1_plus_7_then_2_plus_8.json').exists(),'Export whole-body analysis sample first')
    def test_squat_sample_stays_blocked(self):
        meta=json.loads((ROOT/'artifacts/motion_1_plus_7_then_2_plus_8.json').read_text())
        self.assertFalse(meta['upper_only_compatible'])
        self.assertTrue(meta['lower_body_requested'])
        self.assertEqual(meta['full_reference_limit_violations'],[])
        self.assertFalse(meta['whole_body_hardware_replay_allowed'])

if __name__=='__main__':unittest.main()
