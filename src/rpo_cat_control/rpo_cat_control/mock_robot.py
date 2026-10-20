"""A ROS-only RPO interface double. Never imports a motor SDK."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from sensor_msgs.msg import JointState,Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .contracts import profile,check_upper,rate_step


def main(args=None):
    rclpy.init(args=args)
    node=MockRobot()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except _rclpy.RCLError:
        if rclpy.ok():raise
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


class MockRobot(Node):
    def __init__(self):
        super().__init__('rpo_mock_robot')
        self.declare_parameter('profile','config/rpo_23.yaml')
        self.cfg=profile(self.get_parameter('profile').value)
        self.q=np.array(self.cfg['default_position'],dtype=float)
        self.target=self.q[13:].copy();self.enabled=False;self.last_packet=0.
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState,'/rpo_mock/joint_ref_states',self.command,qos)
        self.pub=self.create_publisher(JointState,'/rpo_mock/joint_states',qos)
        self.imu=self.create_publisher(Imu,'/rpo_mock/imu',qos)
        self.status=self.create_publisher(String,'/rpo_mock/status',10)
        self.create_service(SetBool,'/rpo_mock/set_interrupt',self.enable)
        self.create_timer(.01,self.tick)

    def command(self,msg):
        try:self.target=check_upper(msg.position,self.cfg)
        except ValueError as e:self.get_logger().warning(str(e));return
        self.last_packet=time.monotonic()

    def enable(self,request,response):
        if request.data and time.monotonic()-self.last_packet>.5:
            response.success=False;response.message='No fresh valid 10-joint target';return response
        self.enabled=request.data;response.success=True;response.message='Mock interrupt '+str(self.enabled)
        return response

    def tick(self):
        if time.monotonic()-self.last_packet>.5:self.enabled=False
        target=self.target if self.enabled else np.array(self.cfg['default_position'][13:])
        self.q[13:]=rate_step(self.q[13:],target,.01,.5)
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=self.cfg['joint_names'];msg.position=self.q.tolist();msg.velocity=[0.]*23
        self.pub.publish(msg)
        imu=Imu();imu.header.stamp=msg.header.stamp;imu.orientation.w=1.;self.imu.publish(imu)
        s=String();s.data=json.dumps(dict(interrupt=self.enabled,hardware=False));self.status.publish(s)
