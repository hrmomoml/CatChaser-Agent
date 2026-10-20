#!/usr/bin/env python3
"""Read-only host/robot prerequisite report; never constructs SDK hardware objects."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import platform
import resource
import shutil


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--role',choices=['host','robot'],default='host');p.add_argument('--output',type=Path)
 a=p.parse_args()
 modules={m:importlib.util.find_spec(m) is not None for m in ['rclpy','sensor_msgs','std_msgs','std_srvs','numpy','yaml']}
 report=dict(role=a.role,os=platform.system(),architecture=platform.machine(),kernel=platform.release(),
             wsl='microsoft' in platform.release().lower(),ros_distro=os.getenv('ROS_DISTRO'),
             ros_domain_id=os.getenv('ROS_DOMAIN_ID','0'),ros_localhost_only=os.getenv('ROS_LOCALHOST_ONLY','unset'),
             rmw=os.getenv('RMW_IMPLEMENTATION','default'),modules=modules,
             tools={x:bool(shutil.which(x)) for x in ['git','colcon','cmake','g++','ros2','ssh','gh']},
             ready_for_host_mock=all(modules.values()) and bool(shutil.which('colcon')),
             hardware_validation='NOT_PERFORMED',cross_machine_dds='NOT_TESTED')
 if a.role=='robot':
  report.update(sdk_module_available=importlib.util.find_spec('robot_py') is not None,
                realtime_priority_limit=resource.getrlimit(resource.RLIMIT_RTPRIO)[0],
                can_interfaces=[p.name for p in Path('/sys/class/net').glob('can*')],
                imu_default_device_exists=Path('/dev/ttyUSB0').exists())
 text=json.dumps(report,ensure_ascii=False,indent=2)+'\n';print(text,end='')
 if a.output:a.output.write_text(text)

if __name__=='__main__':main()
