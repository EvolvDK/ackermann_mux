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
 * @author Jeremie Deray
 * @author Brighten Lee
 * @author Elouarn de Kerros
 */

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

class AckermannMarker
{
public:
  AckermannMarker(std::string & frame_id, double scale, double z)
  : frame_id_(frame_id), scale_(scale), z_(z)
  {
    // ID and type:
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::ARROW;

    // Frame ID:
    marker_.header.frame_id = frame_id_;

    // Pre-allocate points for setting the arrow with the ackermann command:
    marker_.points.resize(2);

    // Vertical position:
    marker_.pose.position.z = z_;

    // Scale:
    marker_.scale.x = 0.05 * scale_;
    marker_.scale.y = 2 * marker_.scale.x;

    // Color:
    marker_.color.a = 1.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;

    // Error when all points are zero:
    marker_.points[1].z = 0.01;
  }

  void update(const ackermann_msgs::msg::AckermannDrive & ackermann)
  {
    using std::abs;

    marker_.points[1].x = ackermann.speed;
    marker_.points[1].y = ackermann.steering_angle;
  }

  const visualization_msgs::msg::Marker & getMarker()
  {
    return marker_;
  }

private:
  visualization_msgs::msg::Marker marker_;

  std::string frame_id_;
  double scale_;
  double z_;
};

class AckermannMarkerPublisher : public rclcpp::Node
{
public:
  AckermannMarkerPublisher()
  : Node("ackermann_marker")
  {
    std::string frame_id;
    double scale;
    bool use_stamped;
    double z;

    this->declare_parameter("frame_id", "base_footprint");
    this->declare_parameter("scale", 1.0);
    this->declare_parameter("use_stamped", true);
    this->declare_parameter("vertical_position", 2.0);

    this->get_parameter<std::string>("frame_id", frame_id);
    this->get_parameter<double>("scale", scale);
    this->get_parameter<bool>("use_stamped", use_stamped);
    this->get_parameter<double>("vertical_position", z);

    marker_ = std::make_shared<AckermannMarker>(frame_id, scale, z);


    if (use_stamped)
    {
      sub_stamped_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "ackermann", rclcpp::SystemDefaultsQoS(),
        std::bind(&AckermannMarkerPublisher::callback_stamped, this, std::placeholders::_1));
    }
    else
    {
      sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "ackermann", rclcpp::SystemDefaultsQoS(),
        std::bind(&AckermannMarkerPublisher::callback, this, std::placeholders::_1));
    }
    pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
      "marker",
      rclcpp::QoS(rclcpp::KeepLast(1)));
  }

  void callback(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr ackermann)
  {
    marker_->update(*ackermann);

    pub_->publish(marker_->getMarker());
  }

  void callback_stamped(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr ackermann)
  {
    marker_->update(ackermann->drive);

    pub_->publish(marker_->getMarker());
  }

private:
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_stamped_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;

  std::shared_ptr<AckermannMarker> marker_ = nullptr;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto ackermann_marker_node = std::make_shared<AckermannMarkerPublisher>();

  rclcpp::spin(ackermann_marker_node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
