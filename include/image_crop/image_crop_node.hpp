// Copyright (c) 2026, Bento Robotics
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

#ifndef IMAGE_CROP__IMAGE_CROP_NODE_HPP_
#define IMAGE_CROP__IMAGE_CROP_NODE_HPP_

#include <string>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_crop/visibility.h"

namespace image_crop
{

struct ImageCropConfig
{
  int target_height;
  int target_width;
  int crop_start_x;
  int crop_start_y;
  bool use_camera_info;
  double max_angular_rate;
  double output_image_size;
};

class ImageCropNode : public rclcpp::Node
{
public:
  IMAGE_CROP_PUBLIC ImageCropNode(const rclcpp::NodeOptions & options);

private:
  const std::string frameWithDefault(const std::string & frame, const std::string & image_frame);
  void imageCallbackWithInfo(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void do_work(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void onInit();

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  image_crop::ImageCropConfig config_;

  image_transport::Publisher img_pub_;

  // Subscriber - only one is used at a time - depends on use_camera_info
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

};
}  // namespace image_crop

#endif  // IMAGE_CROP__IMAGE_CROP_NODE_HPP_
