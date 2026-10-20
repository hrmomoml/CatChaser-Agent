#!/usr/bin/env python3
"""Generate the reviewed adapter patch against the exact pinned inference source."""
from pathlib import Path
import argparse,difflib

GUARD='''// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
inline bool valid_external_arm_target(const std::vector<double>& q,
                                      const std::vector<double>& limits) {
    if (q.size() != 10 || limits.size() != 46) return false;
    for (size_t i = 0; i < 10; ++i) {
        const size_t j = 13 + i;
        if (!std::isfinite(q[i]) || q[i] < limits[2*j] || q[i] > limits[2*j+1]) return false;
    }
    return true;
}
inline float limit_external_arm_step(float previous, float target, float max_delta) {
    return previous + std::clamp(target - previous, -max_delta, max_delta);
}
'''

def replace_once(text,before,after):
    if text.count(before)!=1:raise ValueError('Pinned source anchor changed: '+before[:70])
    return text.replace(before,after,1)


def generate(source,output):
    originals={p:(source/p).read_text() for p in ['src/inference_node.hpp','src/ros_interface.cpp','src/inference_node.cpp']}
    changed=dict(originals)
    s=changed['src/inference_node.hpp']
    s=replace_once(s,'#include <std_srvs/srv/trigger.hpp>','#include <std_srvs/srv/trigger.hpp>\n#include <std_srvs/srv/set_bool.hpp>\n#include "external_arm_guard.hpp"')
    anchor='        inference_thread_ = std::thread(&InferenceNode::inference, this);'
    s=replace_once(s,anchor,'''        set_interrupt_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_interrupt", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                std::unique_lock<std::mutex> lock(interrupt_mutex_);
                const bool fresh = interrupt_packet_received_ &&
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_interrupt_packet_).count() < 0.5;
                if (req->data && (!supports_interrupt() || !is_running_.load() || !robot_->is_init_.load() || !fresh)) {
                    res->success = false;
                    res->message = "Require interrupt policy, initialized motors, running inference, and fresh 10-joint target";
                    return;
                }
                is_interrupt_.store(req->data);
                res->success = true;
                res->message = req->data ? "External arms enabled" : "Policy owns arms";
            });
'''+anchor)
    s=replace_once(s,'    std::vector<float> act_, last_act_, cmd_vel_, interrupt_action_, perception_obs_buffer_;',
'''    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_interrupt_service_;
    bool interrupt_packet_received_ = false;
    std::chrono::steady_clock::time_point last_interrupt_packet_{};
    std::vector<float> act_, last_act_, cmd_vel_, interrupt_action_, perception_obs_buffer_;''')
    changed['src/inference_node.hpp']=s
    s=changed['src/ros_interface.cpp']
    old='''    if(supports_interrupt() && is_interrupt_.load()){
        std::unique_lock<std::mutex> lock(interrupt_mutex_);
        for(size_t i = 0; i < interrupt_action_.size(); i++){
            interrupt_action_[i] = msg->position[i];
        }
    }'''
    new='''    if (!supports_interrupt() || !valid_external_arm_target(msg->position, joint_limits_)) {
        RCLCPP_WARN(this->get_logger(), "Rejected external arm packet: need 10 finite absolute angles within limits");
        return;
    }
    std::unique_lock<std::mutex> lock(interrupt_mutex_);
    for (size_t i = 0; i < interrupt_action_.size(); ++i) {
        interrupt_action_[i] = static_cast<float>(msg->position[i]);
    }
    last_interrupt_packet_ = std::chrono::steady_clock::now();
    interrupt_packet_received_ = true;'''
    changed['src/ros_interface.cpp']=replace_once(s,old,new)
    s=changed['src/inference_node.cpp']
    anchor='''    {
        std::unique_lock<std::mutex> lock(act_mutex_);
        float alpha = act_alpha_;'''
    s=replace_once(s,anchor,'''    bool limit_external_arms = false;
    {
        std::unique_lock<std::mutex> lock(interrupt_mutex_);
        limit_external_arms = interrupt_packet_received_;
        if (is_interrupt_.load() && (!interrupt_packet_received_ ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() - last_interrupt_packet_).count() > 0.5)) {
            is_interrupt_.store(false);
        }
    }
'''+anchor)
    s=replace_once(s,'''            last_act_[i] = alpha * act_[i] + (1 - alpha) * last_act_[i];''',
'''            const float target = alpha * act_[i] + (1 - alpha) * last_act_[i];
            last_act_[i] = (limit_external_arms && i + 10 >= act_.size())
                ? limit_external_arm_step(last_act_[i], target, 0.5f * dt_) : target;''')
    changed['src/inference_node.cpp']=s
    changed['src/external_arm_guard.hpp']=GUARD
    patches=[]
    for name,content in changed.items():
        old=originals.get(name,'')
        patches.extend(difflib.unified_diff(old.splitlines(True),content.splitlines(True),
                       fromfile='a/'+name if name in originals else '/dev/null',tofile='b/'+name))
    output.write_text(''.join(patches))
    output.with_name('external_arm_guard.hpp').write_text(GUARD)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();generate(a.source,a.output)
