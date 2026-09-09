// Copyright 2024 TIER IV, Inc.

#include "nebula_velodyne/decoder_wrapper.hpp"

#include <nebula_core_common/util/string_conversions.hpp>
#include <rclcpp/time.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

namespace nebula::ros
{

VelodyneDecoderWrapper::VelodyneDecoderWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config,
  PointCloudCallback pointcloud_callback)
: logger_(parent_node->get_logger().get_child("VelodyneDecoder")),
  sensor_cfg_(config),
  pointcloud_callback_(std::move(pointcloud_callback))
{
  if (!config) {
    throw std::runtime_error(
      "VelodyneDecoderWrapper cannot be instantiated without a valid config!");
  }


  calibration_file_path_ =
    parent_node->declare_parameter<std::string>(
      "calibration_file", param_read_only());

  RCLCPP_INFO_STREAM(
    logger_,
    "Calibration file: '" << calibration_file_path_ << "'");

  calibration_cfg_ptr_ =
    get_calibration_data(calibration_file_path_);

  RCLCPP_INFO_STREAM(
    logger_,
    "Using calibration data from "
      << calibration_cfg_ptr_->calibration_file);

  RCLCPP_INFO(logger_, "Starting Decoder");

  driver_ptr_ = std::make_shared<drivers::VelodyneDriver>(config, calibration_cfg_ptr_);

  const auto status = driver_ptr_->get_status();

  if (status != Status::OK) {
    throw std::runtime_error(
      "Error instantiating decoder: " + util::to_string(status));
  }
}

void VelodyneDecoderWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);

  driver_ptr_ =
    std::make_shared<drivers::VelodyneDriver>(new_config, calibration_cfg_ptr_);

  sensor_cfg_ = new_config;
}

std::shared_ptr<nebula::drivers::VelodyneCalibrationConfiguration>
VelodyneDecoderWrapper::get_calibration_data(
  const std::string & calibration_file_path)
{
  if (!std::filesystem::exists(calibration_file_path)) {
    throw std::runtime_error(
      "No calibration data found at '" +
      calibration_file_path + "'");
  }

  auto calibration =
    std::make_shared<
      nebula::drivers::VelodyneCalibrationConfiguration>();

  const auto status =
    calibration->load_from_file(calibration_file_path);

  if (status != Status::OK) {
    throw std::runtime_error(
      "Could not load calibration file at '" +
      calibration_file_path +
      "': " +
      util::to_string(status));
  }

  calibration->calibration_file =
    calibration_file_path;

  return calibration;
}

void VelodyneDecoderWrapper::process_cloud_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts;

  {
    std::lock_guard lock(mtx_driver_ptr_);

    pointcloud_ts = driver_ptr_->parse_cloud_packet(
      packet_msg->data,
      rclcpp::Time(packet_msg->stamp).seconds());
  }

  auto pointcloud = std::get<0>(pointcloud_ts);

  if (!pointcloud) {
    return;
  }

  if (pointcloud_callback_) {
    pointcloud_callback_(pointcloud, std::get<1>(pointcloud_ts));
  }
}

nebula::Status VelodyneDecoderWrapper::status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->get_status();
}

}  // namespace nebula::ros