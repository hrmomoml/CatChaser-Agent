"""Publish existing upper-body motions to mock or patched official Interrupt ROS 2 APIs."""
from collections import deque
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger
from .contracts import profile,check_upper,load_motion,interpolate,rate_step


class MotionPlayer(Node):
    def __init__(self):
        super().__init__('rpo_motion_player')
        for key,value in [('profile','config/rpo_23.yaml'),('library','artifacts'),('backend','mock'),
                          ('hardware_enabled',False),('time_scale',.25)]:self.declare_parameter(key,value)
        self.cfg=profile(self.get_parameter('profile').value)
        self.library=Path(self.get_parameter('library').value)
        self.backend=self.get_parameter('backend').value
        self.scale=float(self.get_parameter('time_scale').value)
        if self.backend not in ('mock','official_interrupt'):raise ValueError('Unsupported backend')
        if not (0.<self.scale<=1.):raise ValueError('time_scale must be in (0,1]')
        if self.backend=='official_interrupt':
            if not self.get_parameter('hardware_enabled').value:
                raise ValueError('Real output requires explicit hardware_enabled:=true')
            if self.scale>.25:raise ValueError('Initial real-robot time_scale must be <=0.25')
        prefix='/rpo_mock' if self.backend=='mock' else ''
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher=self.create_publisher(JointState,prefix+'/joint_ref_states',qos)
        self.create_subscription(JointState,prefix+'/joint_states',self.feedback,qos)
        self.client=self.create_client(SetBool,prefix+'/set_interrupt')
        self.create_subscription(String,'/rpo/command',self.command,10)
        self.status_pub=self.create_publisher(String,'/rpo/status',10)
        self.create_service(Trigger,'/rpo/stop',self.stop_service)
        self.pending=deque();self.active=None;self.elapsed=0.;self.last_feedback=0.
        self.current=None;self.enabled=False;self.future=None;self.mode='waiting_for_feedback';self.error=None
        self.timer_time=time.monotonic();self.create_timer(.02,self.tick)
        self.get_logger().info('Backend: '+self.backend+'; control only 10 arm joints; posture 1/2 unavailable')

    def feedback(self,msg):
        if len(msg.position)!=23:return
        try:q=check_upper(msg.position[13:],self.cfg)
        except ValueError:return
        self.last_feedback=time.monotonic()
        if self.current is None or not self.enabled:self.current=q.copy()

    def request_mode(self,enabled):
        if not self.client.service_is_ready():
            self.error='set_interrupt service unavailable; apply the pinned upstream patch';return False
        request=SetBool.Request();request.data=enabled
        self.future=self.client.call_async(request);self.requested_mode=enabled
        return True

    def command(self,msg):
        value=msg.data.strip()
        if value=='0':self.stop();return
        tokens=value.split()
        if not tokens or any(t not in ['3','4','5','6','7','8'] for t in tokens):
            self.error='Only arm actions 3..8 are available. 1/2 and posture combinations need a validated lower-body controller.';return
        if len(self.pending)+len(tokens)>20:self.error='Queue full';return
        try:
            items=[]
            for token in tokens:
                path=self.library/f'motion_{token}.npz'
                metadata=json.loads(path.with_suffix('.json').read_text())
                if not metadata.get('upper_only_compatible') or metadata.get('lower_body_requested'):
                    raise ValueError('Whole-body file rejected by upper-body backend')
                times,values=load_motion(path,self.cfg);items.append((token,times,values))
        except (OSError,ValueError,KeyError) as error:self.error=str(error);return
        self.error=None;self.pending.extend(items)

    def stop(self):
        self.pending.clear();self.active=None;self.enabled=False;self.mode='stopped'
        if self.future is not None and not self.future.done():
            # Queue the disable after the in-flight enable completes in tick().
            self.disable_pending=True
        else:self.request_mode(False)

    def stop_service(self,request,response):
        self.stop();response.success=True;response.message='Motion queue cleared; requested policy ownership of arms';return response

    def tick(self):
        now=time.monotonic();dt=min(.04,max(.001,now-self.timer_time));self.timer_time=now
        if self.future is not None and self.future.done():
            try:
                reply=self.future.result();self.enabled=bool(reply.success and self.requested_mode)
                if not reply.success:self.error=reply.message;self.pending.clear();self.active=None
            except Exception as e:self.error=str(e);self.enabled=False
            self.future=None
            if getattr(self,'disable_pending',False):
                self.disable_pending=False;self.enabled=False;self.request_mode(False)
        fresh=self.current is not None and now-self.last_feedback<.5
        if not fresh:
            if self.enabled:self.stop()
            self.mode='feedback_timeout';self.publish_status();return
        if self.pending and self.active is None:
            self.active=self.pending.popleft();self.elapsed=0.;self.mode='preparing'
        if self.active is not None and not self.enabled and self.future is None:
            # Publish a valid measured target before enabling the patched receiver.
            self.publish_target(self.current)
            if not self.request_mode(True):self.mode='waiting_for_service'
            self.publish_status();return
        if self.active is not None and self.enabled:
            number,times,values=self.active
            if self.mode=='preparing':
                desired=values[0]
                self.current=rate_step(self.current,desired,dt,self.cfg['max_upper_velocity_rad_s'])
                if np.max(np.abs(self.current-desired))<1e-5:self.mode='playing'
            else:
                increment=dt*self.scale
                candidate=interpolate(times,values,min(times[-1],self.elapsed+increment))
                distance=np.max(np.abs(candidate-self.current))
                ratio=min(1.,self.cfg['max_upper_velocity_rad_s']*dt/max(distance,1e-12))
                self.elapsed=min(times[-1],self.elapsed+increment*ratio)
                desired=interpolate(times,values,self.elapsed)
                self.current=rate_step(self.current,desired,dt,self.cfg['max_upper_velocity_rad_s'])
                if self.elapsed>=times[-1] and np.max(np.abs(self.current-values[-1]))<1e-5:
                    self.get_logger().info('Completed '+number);self.active=None;self.mode='holding'
            self.publish_target(self.current)
        elif self.enabled or self.future is not None:
            self.publish_target(self.current)
        self.publish_status()

    def publish_target(self,q):
        values=check_upper(q,self.cfg)
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=self.cfg['upper_joint_names'];msg.position=values.tolist()
        self.publisher.publish(msg)

    def publish_status(self):
        msg=String();msg.data=json.dumps(dict(backend=self.backend,mode=self.mode,interrupt=self.enabled,
                    active=self.active[0] if self.active else None,queued=len(self.pending),error=self.error),ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=MotionPlayer()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except _rclpy.RCLError:
        if rclpy.ok():raise
    finally:
        if rclpy.ok():
            node.stop();rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
