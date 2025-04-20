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

#include "nebula_ros/continental/continental_ars548_ros_wrapper.hpp"

#include <nebula_common/util/string_conversions.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"

namespace nebula::ros
{
ContinentalARS548RosWrapper::ContinentalARS548RosWrapper(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(
    "continental_ars548_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  declare_parameter<std::string>("sensor_model", param_read_only());
  declare_parameter<std::string>("host_ip", param_read_only());
  declare_parameter<std::string>("sensor_ip", param_read_only());
  declare_parameter<std::string>("multicast_ip", param_read_only());
  declare_parameter<std::string>("frame_id", param_read_write());
  declare_parameter<std::string>("base_frame", param_read_write());
  declare_parameter<std::string>("object_frame", param_read_write());
  declare_parameter<uint16_t>("data_port", param_read_only());
  declare_parameter<uint16_t>("configuration_host_port", param_read_only());
  declare_parameter<uint16_t>("configuration_sensor_port", param_read_only());
  declare_parameter<bool>("use_sensor_time", param_read_write());
  declare_parameter<double>("configuration_vehicle_length", param_read_write());
  declare_parameter<double>("configuration_vehicle_width", param_read_write());
  declare_parameter<double>("configuration_vehicle_height", param_read_write());
  declare_parameter<double>("configuration_vehicle_wheelbase", param_read_write());
  declare_parameter<bool>("launch_hw", param_read_only());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ContinentalARS548RosWrapper::on_configure(const rclcpp_lifecycle::State &)
{
  wrapper_status_ = declare_and_get_sensor_config_params();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error("Sensor configuration invalid: " + util::to_string(wrapper_status_));
  }

  RCLCPP_INFO_STREAM(get_logger(), "Sensor Configuration: " << *config_ptr_);

  launch_hw_ = get_parameter("launch_hw").as_bool();

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, config_ptr_);
  }

  decoder_wrapper_.emplace(this, config_ptr_, launch_hw_);

  // Register parameter callback after all params have been declared. Otherwise it would be called
  // once for each declaration
  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&ContinentalARS548RosWrapper::on_parameter_change, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ContinentalARS548RosWrapper::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_logger(), "Starting stream");
  RCLCPP_INFO(get_logger(), "Activating ContinentalARS548RosWrapper");

  if (launch_hw_) {
    hw_interface_wrapper_->hw_interface()->register_packet_callback(
      std::bind(
        &ContinentalARS548RosWrapper::receive_packet_callback, this, std::placeholders::_1));
    stream_start();
  } else {
    packets_sub_ = create_subscription<nebula_msgs::msg::NebulaPackets>(
      "nebula_packets", rclcpp::SensorDataQoS(),
      std::bind(
        &ContinentalARS548RosWrapper::receive_packets_callback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Hardware connection disabled, listening for packets on " << packets_sub_->get_topic_name());
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ContinentalARS548RosWrapper::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating ContinentalARS548RosWrapper");

  if (launch_hw_) {
    hw_interface_wrapper_->hw_interface()->deregister_packet_callback();
  } else {
    packets_sub_.reset();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ContinentalARS548RosWrapper::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up ContinentalARS548RosWrapper");

  hw_interface_wrapper_.reset();
  decoder_wrapper_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ContinentalARS548RosWrapper::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down ContinentalARS548RosWrapper");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

nebula::Status ContinentalARS548RosWrapper::declare_and_get_sensor_config_params()
{
  nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration config;

  config.sensor_model =
    nebula::drivers::sensor_model_from_string(get_parameter("sensor_model").as_string());
  config.host_ip = get_parameter("host_ip").as_string();
  config.sensor_ip = get_parameter("sensor_ip").as_string();
  config.multicast_ip = get_parameter("multicast_ip").as_string();
  config.frame_id = get_parameter("frame_id").as_string();
  config.base_frame = get_parameter("base_frame").as_string();
  config.object_frame = get_parameter("object_frame").as_string();
  config.data_port = get_parameter("data_port").as_int();
  config.configuration_host_port = get_parameter("configuration_host_port").as_int();
  config.configuration_sensor_port = get_parameter("configuration_sensor_port").as_int();
  config.use_sensor_time = get_parameter("use_sensor_time").as_bool();
  config.configuration_vehicle_length =
    static_cast<float>(get_parameter("configuration_vehicle_length").as_double());
  config.configuration_vehicle_width =
    static_cast<float>(get_parameter("configuration_vehicle_width").as_double());
  config.configuration_vehicle_height =
    static_cast<float>(get_parameter("configuration_vehicle_height").as_double());
  config.configuration_vehicle_wheelbase =
    static_cast<float>(get_parameter("configuration_vehicle_wheelbase").as_double());

  if (config.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  auto new_config_ptr = std::make_shared<
    const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration>(config);
  return validate_and_set_config(new_config_ptr);
}

Status ContinentalARS548RosWrapper::validate_and_set_config(
  std::shared_ptr<const drivers::continental_ars548::ContinentalARS548SensorConfiguration> &
    new_config_ptr)
{
  if (new_config_ptr->sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }

  if (new_config_ptr->frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (hw_interface_wrapper_) {
    hw_interface_wrapper_->on_config_change(new_config_ptr);
  }
  if (decoder_wrapper_) {
    decoder_wrapper_->on_config_change(new_config_ptr);
  }

  config_ptr_ = new_config_ptr;
  return Status::OK;
}

void ContinentalARS548RosWrapper::receive_packets_callback(
  std::unique_ptr<nebula_msgs::msg::NebulaPackets> packets_msg_ptr)
{
  if (hw_interface_wrapper_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring NebulaPackets. Launch with launch_hw:=false to enable NebulaPackets "
      "replay.");
    return;
  }

  for (auto & packet : packets_msg_ptr->packets) {
    auto nebula_packet_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
    nebula_packet_ptr->stamp = packet.stamp;
    nebula_packet_ptr->data = std::move(packet.data);

    decoder_wrapper_->process_packet(std::move(nebula_packet_ptr));
  }
}

void ContinentalARS548RosWrapper::receive_packet_callback(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> msg_ptr)
{
  if (!decoder_wrapper_ || decoder_wrapper_->status() != Status::OK) {
    return;
  }

  decoder_wrapper_->process_packet(std::move(msg_ptr));
}

Status ContinentalARS548RosWrapper::get_status()
{
  return wrapper_status_;
}

Status ContinentalARS548RosWrapper::stream_start()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }

  if (hw_interface_wrapper_->status() != Status::OK) {
    return hw_interface_wrapper_->status();
  }

  hw_interface_wrapper_->sensor_interface_start();

  return hw_interface_wrapper_->status();
}

rcl_interfaces::msg::SetParametersResult ContinentalARS548RosWrapper::on_parameter_change(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  std::scoped_lock lock(mtx_config_);

  RCLCPP_INFO(get_logger(), "OnParameterChange");

  drivers::continental_ars548::ContinentalARS548SensorConfiguration new_config(*config_ptr_);

  bool got_any =
    get_param(p, "frame_id", new_config.frame_id) |
    get_param(p, "base_frame", new_config.base_frame) |
    get_param(p, "object_frame", new_config.object_frame) |
    get_param(p, "configuration_vehicle_length", new_config.configuration_vehicle_length) |
    get_param(p, "configuration_vehicle_width", new_config.configuration_vehicle_width) |
    get_param(p, "configuration_vehicle_height", new_config.configuration_vehicle_height) |
    get_param(p, "configuration_vehicle_wheelbase", new_config.configuration_vehicle_wheelbase) |
    get_param(p, "configuration_host_port", new_config.configuration_host_port) |
    get_param(p, "configuration_sensor_port", new_config.configuration_sensor_port);

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  auto new_config_ptr = std::make_shared<
    const nebula::drivers::continental_ars548::ContinentalARS548SensorConfiguration>(new_config);
  auto status = validate_and_set_config(new_config_ptr);

  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = "Invalid configuration: " + util::to_string(status);
    return result;
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

RCLCPP_COMPONENTS_REGISTER_NODE(ContinentalARS548RosWrapper)
}  // namespace nebula::ros
