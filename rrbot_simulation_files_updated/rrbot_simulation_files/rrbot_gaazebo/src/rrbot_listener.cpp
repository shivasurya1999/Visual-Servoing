// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("rrbot_frame_listener")
  {

    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("link1/pose", 1);
    // Call on_timer function every second
    timer_ = this->create_wall_timer(1s, std::bind(&FrameListener::on_timer, this));


    target_frame_1 = this->declare_parameter<std::string>("target_frame_1", "link1");
    tf_buffer_1 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_1 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1);
    publisher_1 = this->create_publisher<geometry_msgs::msg::TransformStamped>("link2/pose", 1);
    // Call on_timer function every second
    timer_1 = this->create_wall_timer(1s, std::bind(&FrameListener::on_timer1, this));

  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel1 = "link1";
    geometry_msgs::msg::TransformStamped msg;
    msg = tf_buffer_->lookupTransform(toFrameRel1, fromFrameRel,tf2::TimePointZero);
    publisher_->publish(msg);
  } 

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;

private:
  void on_timer1()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel1 = target_frame_1.c_str();
    std::string toFrameRel2 = "link2";
    geometry_msgs::msg::TransformStamped msg1;
    msg1 = tf_buffer_1->lookupTransform(toFrameRel2, fromFrameRel1,tf2::TimePointZero);
    publisher_1->publish(msg1);
  } 

  rclcpp::TimerBase::SharedPtr timer_1{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_1{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_1;
  std::string target_frame_1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
