///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, Stefan Kohlbrecher, TU Darmstadt
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of TU Darmstadt nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <rtt/TaskContext.hpp>

#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <ros/ros.h>
#include <hw_interface_example/hw_interface_example.h>
#include <controller_manager/controller_manager.h>


using namespace RTT;

class RttRosControlExample : public RTT::TaskContext{
private:

  // The (example) hardware interface
  boost::shared_ptr<ros_control_example::HwInterfaceExample> hw_interface_;

  // The controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // For saving last update time, so period can be handed to controller manager
  ros::Time last_update_time_;


public:
  RttRosControlExample(const std::string& name):
    TaskContext(name)
  {
  }

  ~RttRosControlExample(){}

private:

  bool configureHook(){

    // @TODO: What´s the proper way of initializing ROS node?

    ros::NodeHandle nh;

    hw_interface_.reset(new ros_control_example::HwInterfaceExample);

    controller_manager_.reset(new controller_manager::ControllerManager(hw_interface_.get(), nh));
  }

  void updateHook(){

    // Get current time and period since last update @TODO: Have to consider real-time clock
    // issues here?
    ros::Time now (ros::Time::now());
    ros::Duration period = now - last_update_time_;
    last_update_time_ = now;

    hw_interface_->read();
    controller_manager_->update(now, period);
    hw_interface_->write();

  }

};
ORO_CREATE_COMPONENT(RttRosControlExample)
