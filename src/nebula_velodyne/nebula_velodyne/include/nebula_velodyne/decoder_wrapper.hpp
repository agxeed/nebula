// Copyright 2024 TIER IV, Inc.
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

#pragma once

#include "nebula_core_ros/parameter_descriptors.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_velodyne_common/velodyne_common.hpp>
#include <nebula_velodyne_decoders/velodyne_driver.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace nebula::ros
{

class VelodyneDecoderWrapper
{
public:
  using PointCloudCallback = std::function<void(
    nebula::drivers::NebulaPointCloudPtr,
    double)>;

  VelodyneDecoderWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config,
    PointCloudCallback pointcloud_callback);

  void process_cloud_packet(
    std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config);

  nebula::Status status();

private:
  std::shared_ptr<nebula::drivers::VelodyneCalibrationConfiguration>
  get_calibration_data(const std::string & calibration_file_path);

  rclcpp::Logger logger_;

  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> sensor_cfg_;

  std::string calibration_file_path_{};

  std::shared_ptr<nebula::drivers::VelodyneCalibrationConfiguration>
    calibration_cfg_ptr_{};

  std::shared_ptr<nebula::drivers::VelodyneDriver> driver_ptr_{};

  PointCloudCallback pointcloud_callback_;

  std::mutex mtx_driver_ptr_;
};

}  // namespace nebula::ros