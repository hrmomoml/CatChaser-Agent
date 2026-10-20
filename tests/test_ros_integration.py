"""Local ROS 2 integration: real DDS messages, no physical SDK or robot."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import numpy as np
import yaml
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[1]


class RosIntegration(unittest.TestCase):
    def test_command_packet_guard_and_disconnect_fallback(self):
        cfg=yaml.safe_load((ROOT/'config/rpo_23.yaml').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp);default=np.array(cfg['default_position'][13:]);target=default.copy();target[5]+=.1
            np.savez(path/'motion_7.npz',time_s=np.array([0.,.5,1.]),upper_joint_names=np.array(cfg['upper_joint_names']),upper_ref=np.array([default,target,default]))
            (path/'motion_7.json').write_text(json.dumps(dict(upper_only_compatible=True,lower_body_requested=False)))
            env=dict(os.environ);env['PYTHONPATH']=str(ROOT/'src/rpo_cat_control')+':'+env.get('PYTHONPATH','')
            logs=[];processes=[]
            rclpy.init();node=rclpy.create_node('rpo_integration_test')
            packets=[];statuses=[];mock=[]
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(JointState,'/rpo_mock/joint_ref_states',lambda m:packets.append(np.array(m.position)),qos)
            node.create_subscription(String,'/rpo/status',lambda m:statuses.append(json.loads(m.data)),10)
            node.create_subscription(String,'/rpo_mock/status',lambda m:mock.append(json.loads(m.data)),10)
            pub=node.create_publisher(String,'/rpo/command',10)
            bad=node.create_publisher(JointState,'/rpo_mock/joint_ref_states',qos)
            def spin_until(predicate,timeout=8.):
                deadline=time.monotonic()+timeout
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.05)
                    if predicate():return True
                return False
            try:
                for module,extra in [('mock_robot',[]),('player',['-p','library:='+str(path),'-p','time_scale:=1.0'])]:
                    log=(path/(module+'.log')).open('w');logs.append(log)
                    processes.append(subprocess.Popen([sys.executable,'-c',f'from rpo_cat_control.{module} import main; main()',
                        '--ros-args','-p','profile:='+str(ROOT/'config/rpo_23.yaml'),*extra],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT))
                self.assertTrue(spin_until(lambda:len(mock)>2 and len(statuses)>2))
                msg=String();msg.data='1+7';pub.publish(msg)
                self.assertTrue(spin_until(lambda:any(s.get('error') and 'Only arm actions' in s['error'] for s in statuses)))
                msg.data='7';pub.publish(msg)
                self.assertTrue(spin_until(lambda:any(s.get('interrupt') and s.get('active')=='7' for s in statuses)))
                self.assertTrue(spin_until(lambda:any(s.get('mode')=='holding' for s in statuses),12.))
                self.assertGreater(len(packets),10)
                self.assertTrue(all(q.shape==(10,) and np.isfinite(q).all() for q in packets))
                self.assertGreater(max(q[5] for q in packets),default[5]+.08)
                for p in processes:p.poll();self.assertIsNone(p.returncode)
                # Stop the sender; malformed packets must not keep Interrupt alive.
                processes[1].terminate();processes[1].wait(timeout=5)
                malformed=JointState();malformed.position=[0.]*9
                for _ in range(8):bad.publish(malformed);rclpy.spin_once(node,timeout_sec=.1)
                self.assertTrue(spin_until(lambda:mock and not mock[-1]['interrupt'],3.))
            finally:
                for p in processes:
                    if p.poll() is None:p.terminate();p.wait(timeout=5)
                node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
                for log in logs:log.close()
                for f in path.glob('*.log'):
                    text=f.read_text()
                    self.assertNotIn('Traceback',text,f.name+' '+text)

    def test_supported_23_joint_bridge_mock(self):
        sys.path.insert(0,str(ROOT/'src/rpo_cat_control'))
        from rpo_cat_control.sdk_bridge import SupportedBridge
        from std_srvs.srv import Trigger
        from rclpy.executors import SingleThreadedExecutor
        rclpy.init(args=['--ros-args','-p','profile:='+str(ROOT/'config/rpo_23.yaml')])
        bridge=SupportedBridge();client_node=rclpy.create_node('supported_bridge_test')
        executor=SingleThreadedExecutor();executor.add_node(bridge);executor.add_node(client_node)
        states=[]
        client_node.create_subscription(JointState,'/rpo_sdk/joint_states',lambda m:states.append(list(m.position)),1)
        pub=client_node.create_publisher(JointState,'/rpo_sdk/joint_target',1)
        initialize=client_node.create_client(Trigger,'/rpo_sdk/initialize')
        def until(predicate,timeout=3.):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.02)
                if predicate():return True
            return False
        try:
            self.assertTrue(initialize.wait_for_service(timeout_sec=2.))
            future=initialize.call_async(Trigger.Request());self.assertTrue(until(future.done));self.assertTrue(future.result().success)
            cfg=yaml.safe_load((ROOT/'config/rpo_23.yaml').read_text())
            q=list(cfg['default_position']);q[3]+=.05
            msg=JointState();msg.name=cfg['joint_names'];msg.position=q;pub.publish(msg)
            self.assertTrue(until(lambda:states and states[-1][3]>.345))
            self.assertEqual(len(states[-1]),23)
            self.assertIsNone(bridge.robot)
            self.assertNotIn('robot_py',sys.modules)
            # A larger target without refresh must time out before reaching it.
            q[3]=.65;msg.position=q;pub.publish(msg)
            self.assertTrue(until(lambda:states and states[-1][3]>.38))
            start=time.monotonic()
            until(lambda:time.monotonic()-start>.7,2.)
            self.assertLess(states[-1][3],.5)
        finally:
            executor.remove_node(bridge);executor.remove_node(client_node)
            bridge.destroy_node();client_node.destroy_node();executor.shutdown();rclpy.shutdown()

if __name__=='__main__':unittest.main()
