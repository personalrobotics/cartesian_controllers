////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_twist_controller.cpp
 *
 * \author  Ethan K. Gordon <ekgordon@cs.uw.edu>
 * \date    2024/01/15
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_twist_controller/cartesian_twist_controller.h>

#include <algorithm>
#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

namespace cartesian_twist_controller
{
CartesianTwistController::CartesianTwistController() : Base::CartesianControllerBase() {}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianTwistController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianTwistController::init(
  const std::string & controller_name)
{
  const auto ret = Base::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  return controller_interface::return_type::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianTwistController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  m_twist_subscr = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    get_node()->get_name() + std::string("/twist_cmd"), 3,
    std::bind(&CartesianTwistController::twistCallback, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianTwistController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);

  // Start where we are
  m_twist = ctrl::Vector6D::Zero();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianTwistController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
controller_interface::return_type CartesianTwistController::update(const rclcpp::Time & time,
                                                                    const rclcpp::Duration & period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianTwistController::update()
#endif
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Turn Cartesian error into joint motion
  m_simulated_joint_motion = Base::m_ik_solver->getJointControlCmds(period, m_twist);

  Base::m_ik_solver->updateKinematics();

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

void CartesianTwistController::twistCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr target)
{
  if (std::isnan(target->twist.linear.x) || std::isnan(target->twist.linear.y) ||
      std::isnan(target->twist.linear.z) || std::isnan(target->twist.angular.x) ||
      std::isnan(target->twist.angular.y) || std::isnan(target->twist.angular.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in twist. Ignoring input.");
    return;
  }

  if (target->header.frame_id != Base::m_robot_base_link)
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), clock, 3000,
                         "Got twist in wrong reference frame. Expected: %s but got %s",
                         Base::m_robot_base_link.c_str(), target->header.frame_id.c_str());
    return;
  }

  m_twist(0) = target->twist.linear.x;
  m_twist(1) = target->twist.linear.y;
  m_twist(2) = target->twist.linear.z;
  m_twist(3) = target->twist.angular.x;
  m_twist(4) = target->twist.angular.y;
  m_twist(5) = target->twist.angular.z;
}

}  // namespace cartesian_twist_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_twist_controller::CartesianTwistController,
                       controller_interface::ControllerInterface)
