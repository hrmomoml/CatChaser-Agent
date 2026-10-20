"""Experimental 23-joint ROS-to-SDK bridge for a mechanically supported robot.

This is not a balancing controller. Use it instead of the official inference
process, never concurrently. The default backend is a hardware-free mock.
"""
import importlib
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from .contracts import profile,check_sdk_config


class SupportedBridge(Node):
    def __init__(self):
        super().__init__('rpo_supported_sdk_bridge')
        for key,value in [('profile','config/rpo_23.yaml'),('backend','mock'),('hardware_enabled',False),
                          ('supported_fixture',False),('sdk_config',''),('control_dt',.004)]:self.declare_parameter(key,value)
        self.cfg=profile(self.get_parameter('profile').value)
        self.backend=self.get_parameter('backend').value
        self.dt=float(self.get_parameter('control_dt').value)
        if self.backend not in ('mock','sdk') or not (.004<=self.dt<=.02):raise ValueError('Invalid backend/control period')
        if self.backend=='sdk' and not (self.get_parameter('hardware_enabled').value and self.get_parameter('supported_fixture').value):
            raise ValueError('SDK backend requires hardware_enabled and supported_fixture; no balancing controller is provided')
        self.robot=None;self.initialized=False;self.q=np.array(self.cfg['default_position'],dtype=float)
        self.sent=self.q.copy();self.target=self.q.copy();self.last_packet=0.
        self.bounds=np.array(self.cfg['joint_limits']).reshape(23,2)
        self.create_service(Trigger,'/rpo_sdk/initialize',self.initialize)
        self.create_service(Trigger,'/rpo_sdk/deinitialize',self.deinitialize)
        self.create_subscription(JointState,'/rpo_sdk/joint_target',self.command,1)
        self.pub=self.create_publisher(JointState,'/rpo_sdk/joint_states',1)
        self.create_timer(self.dt,self.tick)

    def validate(self,values):
        q=np.array(values,dtype=float)
        if q.shape!=(23,) or not np.isfinite(q).all():raise ValueError('Need 23 finite logical joint positions')
        if np.any(q<self.bounds[:,0]) or np.any(q>self.bounds[:,1]):raise ValueError('Target outside configured joint limits')
        return q

    def initialize(self,request,response):
        if self.initialized:
            response.success=True;response.message='Already initialized';return response
        try:
            if self.backend=='sdk':
                if any('inference' in name for name in self.get_node_names()):
                    raise ValueError('An inference node is present; stop the official CAN owner first')
                path=self.get_parameter('sdk_config').value
                if not path:raise ValueError('sdk_config must be the verified RPO robot.yaml')
                check_sdk_config(path)
                sdk=importlib.import_module('robot_py')
                self.robot=sdk.RobotInterface(path,self.dt)
                self.robot.init_motors();self.robot.refresh_joints()
                self.q=self.validate(self.robot.get_joint_q())
            self.sent=self.q.copy();self.target=self.q.copy();self.initialized=True
            response.success=True;response.message='Initialized '+self.backend+'; supported pose control only'
        except Exception as error:
            self.initialized=False
            if self.robot is not None:
                try:self.robot.deinit_motors()
                except Exception:pass
                self.robot=None
            response.success=False;response.message=str(error)
        return response

    def deinitialize(self,request,response):
        self.initialized=False
        if self.robot is not None:self.robot.deinit_motors();self.robot=None
        response.success=True;response.message='Deinitialized';return response

    def command(self,msg):
        try:
            if not self.initialized:raise ValueError('Initialize the supported bridge first')
            if msg.name and list(msg.name)!=self.cfg['joint_names']:raise ValueError('Wrong logical joint order')
            self.target=self.validate(msg.position);self.last_packet=time.monotonic()
        except ValueError as error:self.get_logger().warning(str(error))

    def tick(self):
        if not self.initialized:return
        try:
            # On command timeout hold the last sent position; do not continue the motion.
            if time.monotonic()-self.last_packet>.5:self.target=self.sent.copy()
            self.sent+=np.clip(self.target-self.sent,-.2*self.dt,.2*self.dt)
            if self.robot is not None:
                self.robot.apply_action(self.sent.tolist())
                self.robot.read_joints();self.q=np.array(self.robot.get_joint_q())
                if self.q.shape!=(23,) or not np.isfinite(self.q).all():raise RuntimeError('Invalid SDK feedback')
            else:self.q=self.sent.copy()
            msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=self.cfg['joint_names'];msg.position=self.q.tolist();self.pub.publish(msg)
        except Exception as error:
            self.initialized=False
            self.get_logger().error('SDK control stopped: '+str(error))
            if self.robot is not None:
                try:self.robot.deinit_motors()
                finally:self.robot=None


def main(args=None):
    rclpy.init(args=args);node=SupportedBridge()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except _rclpy.RCLError:
        if rclpy.ok():raise
    finally:
        if node.robot is not None:node.robot.deinit_motors()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
