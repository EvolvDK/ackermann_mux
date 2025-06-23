// Copyright 2020 PAL Robotics S.L.
// Copyright 2024 Elouarn de KERROS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
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

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 * @author Jeremie Deray
 * @author Elouarn de Kerros
 */

#include <ackermann_mux/ackermann_mux.hpp>
#include <ackermann_mux/topic_handle.hpp>
#include <ackermann_mux/ackermann_mux_diagnostics.hpp>
#include <ackermann_mux/ackermann_mux_diagnostics_status.hpp>
#include <ackermann_mux/utils.hpp>
#include <ackermann_mux/params_helpers.hpp>

#include <list>
#include <memory>
#include <string>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: speed or steering angle
 * @param old_cmd Old command
 * @param new_cmd New command
 * @return true is any of the absolute components has increased
 */
bool hasIncreasedAbsVelocity(
  const ackermann_msgs::msg::AckermannDriveStamped & old_cmd,
  const ackermann_msgs::msg::AckermannDriveStamped & new_cmd)
{
  const auto old_speed = std::abs(old_cmd.drive.speed);
  const auto new_speed = std::abs(new_cmd.drive.speed);

  const auto old_acceleration = std::abs(old_cmd.drive.acceleration);
  const auto new_acceleration = std::abs(new_cmd.drive.acceleration);

  const auto old_jerk = std::abs(old_cmd.drive.jerk);
  const auto new_jerk = std::abs(new_cmd.drive.jerk);

  const auto old_steering = std::abs(old_cmd.drive.steering_angle);
  const auto new_steering = std::abs(new_cmd.drive.steering_angle);

  return (old_speed < new_speed) || 
         (old_acceleration < new_acceleration) ||
         (old_jerk < new_jerk) ||
         (old_steering < new_steering);
}

namespace ackermann_mux
{
// see e.g. https://stackoverflow.com/a/40691657
constexpr std::chrono::duration<int64_t> AckermannMux::DIAGNOSTICS_PERIOD;

AckermannMux::AckermannMux()
: Node("ackermann_mux", "",
    rclcpp::NodeOptions().allow_undeclared_parameters(
      true).automatically_declare_parameters_from_overrides(true))
{
}

void AckermannMux::init()
{
  bool use_stamped = true;
  auto nh = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
  try {
      fetch_param(nh, "use_stamped", use_stamped);
  } catch (const ParamsHelperException & e) {
    RCLCPP_WARN(get_logger(), "Parameter 'use_stamped' not found, using the default value: %s",
                use_stamped ? "true":"false");
  }

  /// Get topics and locks:
  if(use_stamped){  
    ackermann_stamped_hs_ = std::make_shared<ackermann_stamped_topic_container>();
    getTopicHandles("topics", *ackermann_stamped_hs_);
  }else{
    ackermann_hs_ = std::make_shared<ackermann_topic_container>();
    getTopicHandles("topics", *ackermann_hs_);
  }
  lock_hs_ = std::make_shared<lock_topic_container>();
  getTopicHandles("locks", *lock_hs_);

  if(use_stamped){
    cmd_pub_stamped_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd",rclcpp::QoS(rclcpp::KeepLast(1)));
  }
  else{
    cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("ackermann_cmd",rclcpp::QoS(rclcpp::KeepLast(1)));
  }

  /// Diagnostics:
  diagnostics_ = std::make_shared<diagnostics_type>(this);
  status_ = std::make_shared<status_type>();
  status_->ackermann_hs = ackermann_hs_;
  status_->ackermann_stamped_hs = ackermann_stamped_hs_;
  status_->lock_hs = lock_hs_;
  status_->use_stamped = use_stamped;

  diagnostics_timer_ = this->create_wall_timer(
    DIAGNOSTICS_PERIOD, [this]() -> void {
      updateDiagnostics();
    });
}

void AckermannMux::updateDiagnostics()
{
  status_->priority = getLockPriority();
  diagnostics_->updateStatus(status_);
}

void AckermannMux::publishAckermann(
  const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr & msg)
{
  cmd_pub_->publish(*msg);
}

void AckermannMux::publishAckermannStamped(
  const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr & msg)
{
  cmd_pub_stamped_->publish(*msg);
}

template<typename T>
void AckermannMux::getTopicHandles(const std::string & param_name, std::list<T> & topic_hs)
{
  RCLCPP_DEBUG(get_logger(), "getTopicHandles: %s", param_name.c_str());

  rcl_interfaces::msg::ListParametersResult list = list_parameters({param_name}, 10);

  try {
    for (auto prefix : list.prefixes) {
      RCLCPP_DEBUG(get_logger(), "Prefix: %s", prefix.c_str());

      std::string topic;
      double timeout = 0;
      int priority = 0;

      auto nh = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

      fetch_param(nh, prefix + ".topic", topic);
      fetch_param(nh, prefix + ".timeout", timeout);
      fetch_param(nh, prefix + ".priority", priority);

      RCLCPP_DEBUG(get_logger(), "Retrieved topic: %s", topic.c_str());
      RCLCPP_DEBUG(get_logger(), "Listed prefix: %.2f", timeout);
      RCLCPP_DEBUG(get_logger(), "Listed prefix: %d", priority);

      topic_hs.emplace_back(prefix, topic, std::chrono::duration<double>(timeout), priority, this);
    }
  } catch (const ParamsHelperException & e) {
    RCLCPP_FATAL(get_logger(), "Error parsing params '%s':\n\t%s", param_name.c_str(), e.what());
    throw e;
  }
}

int AckermannMux::getLockPriority()
{
  LockTopicHandle::priority_type priority = 0;

  /// max_element on the priority of lock topic handles satisfying
  /// that is locked:
  for (const auto & lock_h : *lock_hs_) {
    if (lock_h.isLocked()) {
      auto tmp = lock_h.getPriority();
      if (priority < tmp) {
        priority = tmp;
      }
    }
  }


  RCLCPP_DEBUG(get_logger(), "Priority = %d.", static_cast<int>(priority));

  return priority;
}

bool AckermannMux::hasPriority(const AckermannTopicHandle & ackermann)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string ackermann_name = "NULL";

  /// max_element on the priority of ackermann topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto & ackermann_h : *ackermann_hs_) {
    if (!ackermann_h.isMasked(lock_priority)) {
      const auto ackermann_priority = ackermann_h.getPriority();
      if (priority < ackermann_priority) {
        priority = ackermann_priority;
        ackermann_name = ackermann_h.getName();
      }
    }
  }

  return ackermann.getName() == ackermann_name;
}

bool AckermannMux::hasPriorityStamped(const AckermannStampedTopicHandle & ackermann)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string ackermann_name = "NULL";

  /// max_element on the priority of ackermann topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto & ackermann_stamped_h : *ackermann_stamped_hs_) {
    if (!ackermann_stamped_h.isMasked(lock_priority)) {
      const auto ackermann_priority = ackermann_stamped_h.getPriority();
      if (priority < ackermann_priority) {
        priority = ackermann_priority;
        ackermann_name = ackermann_stamped_h.getName();
      }
    }
  }

  return ackermann.getName() == ackermann_name;
}

}  // namespace ackermann_mux
