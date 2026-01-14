// Copyright (c) 2026, Bento Robotics
// Copyright (c) 2014, JSK Lab.
// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/********************************************************************
* image_crop_node.cpp
* this is modified version of image_rotate.
* And now crops images instead
*********************************************************************/

#include "image_crop/image_crop_node.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_crop
{

ImageCropNode::ImageCropNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ImageCropNode", options)
{
  // Set runtime parameters BEFORE add_on_set_parameters_callback
  config_.target_height = this->declare_parameter("target_height", 400);
  config_.target_width = this->declare_parameter("target_width", 800);
  config_.crop_start_x = this->declare_parameter("crop_start_x", 100);
  config_.crop_start_y = this->declare_parameter("crop_start_y", 100);

  auto reconfigureCallback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name() == "target_height") {
          config_.target_width = parameter.as_int();
          RCLCPP_INFO(get_logger(), "Set target_height to '%i'", config_.target_height);
        } else if (parameter.get_name() == "target_width") {
          config_.target_height = parameter.as_int();
          RCLCPP_INFO(get_logger(), "Set target_width to '%i'", config_.target_width);
        }else if (parameter.get_name() == "crop_start_x") {
          config_.crop_start_x = parameter.as_int();
          RCLCPP_INFO(get_logger(), "Set crop_start_x to '%i'", config_.crop_start_x);
        } else if (parameter.get_name() == "crop_start_y") {
          config_.crop_start_y = parameter.as_int();
          RCLCPP_INFO(get_logger(), "Set crop_start_y to '%i'", config_.crop_start_y);
        }
      }

      return result;
    };
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(reconfigureCallback);

  // Set other parameters AFTER add_on_set_parameters_callback
  config_.use_camera_info = this->declare_parameter("use_camera_info", true);
  config_.output_image_size = this->declare_parameter("output_image_size", 2.0);

  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");

  onInit();
}

// const std::string ImageCropNode::frameWithDefault(
//   const std::string & frame,
//   const std::string & image_frame)
// {
//   if (frame.empty()) {
//     return image_frame;
//   }
//   return frame;
// }

void ImageCropNode::imageCallbackWithInfo(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  (void) cam_info;
  do_work(msg);
}

void ImageCropNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  do_work(msg); 
}

void ImageCropNode::do_work(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{  
  if (config_.crop_start_y < 0 || (uint) abs(config_.crop_start_y + config_.target_height) > msg->height ||
      config_.crop_start_x < 0 || (uint) abs(config_.crop_start_x + config_.target_width)  > msg->width  ){
        RCLCPP_ERROR(get_logger(), "Crop startpoint at (%i, %i) is invalid", config_.crop_start_x, config_.crop_start_y);
        return;
  }
  if (config_.target_height <= 0 ||
      config_.target_width  <= 0 ){ 
        RCLCPP_ERROR(get_logger(), "Crop size of %i x %i is invalid", config_.target_width, config_.target_height);
        return;
  }
  try {
    // Convert the image into something opencv can handle.
    cv::Mat in_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

    // Do the crop
    cv::Rect crop_region(config_.crop_start_x, config_.crop_start_y, config_.target_width, config_.target_height);
    cv::Mat out_image = in_image(crop_region);

    // Publish the image.
    sensor_msgs::msg::Image::SharedPtr out_img =
    cv_bridge::CvImage(msg->header, msg->encoding, out_image).toImageMsg();
    img_pub_.publish(out_img);

  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Image processing error: %s %s %s %i",
      e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
  }
}

void ImageCropNode::onInit()
{
  rclcpp::Clock::SharedPtr clock = this->get_clock();

  // Create publisher with connect callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo &)
    {
      if (img_pub_.getNumSubscribers() == 0) {
        RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
        img_sub_.shutdown();
        cam_sub_.shutdown();
      } else {
        // For compressed topics to remap appropriately, we need to pass a
        // fully expanded and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string topic_name = node_base->resolve_topic_or_service_name("image", false);
        RCLCPP_INFO(get_logger(), "Subscribing to %s topic.", topic_name.c_str());

        // Check that remapping appears to be correct
        auto topics = this->get_topic_names_and_types();
        if (topics.find(topic_name) == topics.end()) {
          RCLCPP_WARN(
            get_logger(),
            "Topic %s is not connected! Typical command-line usage:\n"
            "\t$ ros2 run image_crop image_crop --ros-args -r image:=<image topic>",
            topic_name.c_str());
        }

        // This will check image_transport parameter to get proper transport
        image_transport::TransportHints transport_hint(this, "raw");

        if (config_.use_camera_info) {
          auto custom_qos = rmw_qos_profile_system_default;
          custom_qos.depth = 3;
          cam_sub_ = image_transport::create_camera_subscription(
            this,
            topic_name,
            std::bind(
              &ImageCropNode::imageCallbackWithInfo, this,
              std::placeholders::_1, std::placeholders::_2),
            transport_hint.getTransport(),
            custom_qos);
        } else {
          auto custom_qos = rmw_qos_profile_system_default;
          custom_qos.depth = 3;
          img_sub_ = image_transport::create_subscription(
            this,
            topic_name,
            std::bind(&ImageCropNode::imageCallback, this, std::placeholders::_1),
            transport_hint.getTransport(),
            custom_qos);
        }
      }
    };

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  std::string topic = node_base->resolve_topic_or_service_name("cropped/out", false);

  // Allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  img_pub_ = image_transport::create_publisher(this, topic, rmw_qos_profile_default, pub_options);
}

}  // namespace image_crop

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(image_crop::ImageCropNode)
