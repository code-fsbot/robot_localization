/*
 * Copyright (c) 2017 Simon Gene Gottlieb
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Nodelet 是一种在同一进程中加载多个节点的机制，减少进程间通信的开销
 */

#include "robot_localization/navsat_transform.h"
// 用于实现 ROS Nodelet 和插件机制。
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
// 引入 C++ 的智能指针库，用于管理资源
#include <memory>

namespace RobotLocalization
{

  class NavSatTransformNodelet : public nodelet::Nodelet
  {
  private:
    // 使用智能指针管理 NavSatTransform 对象的生命周期
    std::unique_ptr<RobotLocalization::NavSatTransform> trans;

  public:
    // Nodelet 的初始化函数，在 Nodelet 加载时调用
    virtual void onInit()
    {
      NODELET_DEBUG("Initializing nodelet...");
      // 获取公有和私有的 NodeHandle。
      ros::NodeHandle nh = getNodeHandle();
      ros::NodeHandle nh_priv = getPrivateNodeHandle();
      // 创建并初始化 NavSatTransform 对象，将其存储在智能指针 trans 中
      trans = std::make_unique<RobotLocalization::NavSatTransform>(nh, nh_priv);
    }
  };

} // namespace RobotLocalization

PLUGINLIB_EXPORT_CLASS(RobotLocalization::NavSatTransformNodelet, nodelet::Nodelet);
