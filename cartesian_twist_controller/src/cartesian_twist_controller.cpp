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

  auto_declare<std::string>("wrench_threshold.topic", "");
  auto_declare<double>("wrench_threshold.timeout", 0.0);
  auto_declare<double>("wrench_threshold.fMag", 0.0);
  auto_declare<double>("wrench_threshold.fx", 0.0);
  auto_declare<double>("wrench_threshold.fy", 0.0);
  auto_declare<double>("wrench_threshold.fz", 0.0);
  auto_declare<double>("wrench_threshold.tMag", 0.0);
  auto_declare<double>("wrench_threshold.tx", 0.0);
  auto_declare<double>("wrench_threshold.ty", 0.0);
  auto_declare<double>("wrench_threshold.tz", 0.0);

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

  auto_declare<std::string>("wrench_threshold.topic", "");
  auto_declare<double>("wrench_threshold.timeout", 0.0);
  auto_declare<double>("wrench_threshold.fMag", 0.0);
  auto_declare<double>("wrench_threshold.fx", 0.0);
  auto_declare<double>("wrench_threshold.fy", 0.0);
  auto_declare<double>("wrench_threshold.fz", 0.0);
  auto_declare<double>("wrench_threshold.tMag", 0.0);
  auto_declare<double>("wrench_threshold.tx", 0.0);
  auto_declare<double>("wrench_threshold.ty", 0.0);
  auto_declare<double>("wrench_threshold.tz", 0.0);

  return controller_interface::return_type::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianTwistController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);

  m_wrench_topic_ = get_node()->get_parameter("wrench_threshold.topic").as_string();
  updateWrenchFromParams();

  callback_handle_ = get_node()->add_on_set_parameters_callback(
            std::bind(&CartesianTwistController::parametersCallback, this, std::placeholders::_1));

  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  m_twist_subscr = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    get_node()->get_name() + std::string("/twist_cmd"), 3,
    std::bind(&CartesianTwistController::twistCallback, this, std::placeholders::_1));

  // Wrench thresholding
  rt_wrench_stamped_.writeFromNonRT(nullptr);
  if (m_wrench_topic_ != "")
  {
    wrench_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      m_wrench_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { 
        rt_wrench_stamped_.writeFromNonRT(msg);
      });
  }

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

  // Force Gate
  if (!check_wrench_threshold(time)) {
    m_twist = ctrl::Vector6D::Zero();
  }

  // Turn Cartesian error into joint motion
  m_simulated_joint_motion = Base::m_ik_solver->getJointControlCmds(period, m_twist);

  Base::m_ik_solver->updateKinematics();

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

bool CartesianTwistController::check_wrench_threshold(const rclcpp::Time & time)
{
  // Check if enabled
  if (m_wrench_topic_ == "")
  {
    return true;
  }

  // Read stamped wrench
  auto wrench_stamped = *rt_wrench_stamped_.readFromRT();
  if (!wrench_stamped) {
    RCLCPP_WARN(get_node()->get_logger(), "WrenchStamped not received.");
    return false;
  }
  
  // Check all relevant tolerances
  if (wrench_tolerances_.timeout != rclcpp::Duration(0, 0)) {
    if (time - wrench_tolerances_.timeout > wrench_stamped->header.stamp) {
      RCLCPP_WARN(get_node()->get_logger(), "WrenchStamped timeout.");
      return false;
    }
  }

  double forceSumSq = 0.0;
  if (wrench_tolerances_.forceVec[0] != 0.0) {
    if (wrench_stamped->wrench.force.x > wrench_tolerances_.forceVec[0]) {
      RCLCPP_WARN(get_node()->get_logger(), "Wrench: Fx violation.");
      return false;
    }
  }
  forceSumSq += wrench_stamped->wrench.force.x * wrench_stamped->wrench.force.x;
  if (wrench_tolerances_.forceVec[1] != 0.0) {
    if (wrench_stamped->wrench.force.y > wrench_tolerances_.forceVec[1]) {
      RCLCPP_WARN(get_node()->get_logger(), "Wrench: Fy violation.");
      return false;
    }
  }
  forceSumSq += wrench_stamped->wrench.force.y * wrench_stamped->wrench.force.y;
  if (wrench_tolerances_.forceVec[2] != 0.0) {
    if (wrench_stamped->wrench.force.z > wrench_tolerances_.forceVec[2]) {
      RCLCPP_WARN(get_node()->get_logger(), "Wrench: Fz violation.");
      return false;
    }
  }
  forceSumSq += wrench_stamped->wrench.force.z * wrench_stamped->wrench.force.z;
  if (wrench_tolerances_.forceTotal != 0.0) {
    if (forceSumSq > wrench_tolerances_.forceTotal*wrench_tolerances_.forceTotal) {
      RCLCPP_WARN(get_node()->get_logger(), "Wrench: ||F|| violation.");
      return false;
    }
  }

  double torqueSumSq = 0.0;
  if (wrench_tolerances_.torqueVec[0] != 0.0) {
    if (wrench_stamped->wrench.torque.x > wrench_tolerances_.torqueVec[0]) {
      RCLCPP_WARN(get_node()->get_logger(), "Wrench: Tx violation.");
      return false;
    }
  }
  torqueSumSq += wrench_stamped->wrench.torque.x * wrench_stamped->wrench.torque.x;
  if (wrench_tolerances_.torqueVec[1] != 0.0) {
    if (wrench_stamped->wrench.torque.y > wrench_tolerances_.torqueVec[1]) {
      RCLCPP_WARN(get_node()->get_logger(), "Wrench: Ty violation.");
      return false;
    }
  }
  torqueSumSq += wrench_stamped->wrench.torque.y * wrench_stamped->wrench.torque.y;
  if (wrench_tolerances_.torqueVec[2] != 0.0) {
    if (wrench_stamped->wrench.torque.z > wrench_tolerances_.torqueVec[2]) {
      RCLCPP_WARN(get_node()->get_logger(), "Wrench: Tz violation.");
      return false;
    }
  }
  torqueSumSq += wrench_stamped->wrench.torque.z * wrench_stamped->wrench.torque.z;
  if (wrench_tolerances_.torqueTotal != 0.0) {
    if (torqueSumSq > wrench_tolerances_.torqueTotal*wrench_tolerances_.torqueTotal) {
      RCLCPP_WARN(get_node()->get_logger(), "Wrench: ||T|| violation.");
      return false;
    }
  }

  return true;
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

void CartesianTwistController::updateWrenchFromParams()
{
  wrench_tolerances_.timeout = rclcpp::Duration::from_seconds(get_node()->get_parameter("wrench_threshold.timeout").as_double());
  wrench_tolerances_.forceTotal = get_node()->get_parameter("wrench_threshold.fMag").as_double();
  wrench_tolerances_.forceVec[0] = get_node()->get_parameter("wrench_threshold.fx").as_double();
  wrench_tolerances_.forceVec[1] = get_node()->get_parameter("wrench_threshold.fy").as_double();
  wrench_tolerances_.forceVec[2] = get_node()->get_parameter("wrench_threshold.fz").as_double();
  wrench_tolerances_.torqueTotal = get_node()->get_parameter("wrench_threshold.tMag").as_double();
  wrench_tolerances_.torqueVec[0] = get_node()->get_parameter("wrench_threshold.tx").as_double();
  wrench_tolerances_.torqueVec[1] = get_node()->get_parameter("wrench_threshold.ty").as_double();
  wrench_tolerances_.torqueVec[2] = get_node()->get_parameter("wrench_threshold.tz").as_double();
  RCLCPP_INFO(get_node()->get_logger(), "Updating Wrench Thresholds, fMag %f", wrench_tolerances_.forceTotal);
}

rcl_interfaces::msg::SetParametersResult CartesianTwistController::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  updateWrenchFromParams();
  return result;
}

}  // namespace cartesian_twist_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_twist_controller::CartesianTwistController,
                       controller_interface::ControllerInterface)
