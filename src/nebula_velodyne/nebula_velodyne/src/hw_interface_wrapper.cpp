// Copyright 2024 TIER IV, Inc.

#include "nebula_velodyne/hw_interface_wrapper.hpp"

#include "nebula_core_ros/rclcpp_logger.hpp"

#include <nebula_core_common/util/string_conversions.hpp>
#include <nebula_core_hw_interfaces/connections/tcp.hpp>

#include <chrono>
#include <memory>

namespace nebula::ros
{

VelodyneHwInterfaceWrapper::VelodyneHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & config, bool use_udp_only)
: hw_interface_(
    std::make_shared<drivers::VelodyneHwInterface>(
      drivers::loggers::RclcppLogger(parent_node->get_logger()).child("HwInterface"))),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(Status::NOT_INITIALIZED),
  setup_sensor_(false),
  use_udp_only_(use_udp_only),
  retry_hw_(true),
  sensor_configuration_(config)
{
  using namespace std::chrono_literals;

  setup_sensor_ = parent_node->declare_parameter<bool>("setup_sensor", param_read_only());
  retry_hw_ = parent_node->declare_parameter<bool>("retry_hw", true, param_read_only());

  status_ = hw_interface_->initialize_sensor_configuration(config);

  if (status_ != Status::OK) {
    throw std::runtime_error("Could not initialize HW interface: " + util::to_string(status_));
  }

  if (use_udp_only_) {
    // Do not initialize http client
    return;
  }

  status_ = hw_interface_->init_http_client();

  if (status_ != Status::OK) {
    throw std::runtime_error("Could not initialize HTTP client: " + util::to_string(status_));
  }

  if (setup_sensor_) {
    RCLCPP_INFO_STREAM(logger_, "Setting sensor configuration");
    while (true) {
      status_ = hw_interface_->set_sensor_configuration(config);
      if (status_ == Status::OK) {
        reconnect_configuration_applied_ = true;
        break;
      }
      if (!retry_hw_) {
        break;
      }

      RCLCPP_WARN_STREAM(
        logger_, "Waiting for Velodyne LiDAR at " << config->sensor_ip << " before configuring.");
      rclcpp::sleep_for(5000ms);
    }

    if (status_ != Status::OK) {
      throw std::runtime_error("Could not set sensor configuration: " + util::to_string(status_));
    }

    if (retry_hw_) {
      reconnect_monitor_timer_ =
        parent_node->create_wall_timer(1s, [this]() { monitor_sensor_reconnection(); });
    }
  }

  status_ = Status::OK;
}

void VelodyneHwInterfaceWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config)
{
  sensor_configuration_ = new_config;
  hw_interface_->initialize_sensor_configuration(new_config);
  if (use_udp_only_) {
    return;
  }
  hw_interface_->init_http_client();
  if (setup_sensor_) {
    const auto status = hw_interface_->set_sensor_configuration(new_config);
    status_ = status;
    reconnect_configuration_applied_ = status == Status::OK;
    if (status != Status::OK) {
      if (retry_hw_) {
        RCLCPP_WARN_STREAM(
          logger_, "Could not set sensor configuration: " << util::to_string(status)
                                                          << ". Will retry on reconnect.");
      } else {
        RCLCPP_WARN_STREAM(
          logger_, "Could not set sensor configuration: " << util::to_string(status));
      }
    }
  }
}

void VelodyneHwInterfaceWrapper::on_sensor_packet_received()
{
  last_packet_time_ = std::chrono::steady_clock::now();

  if (!sensor_operational_) {
    sensor_operational_ = true;
    RCLCPP_INFO(logger_, "Velodyne UDP packets received; sensor operational.");
  }
}

bool VelodyneHwInterfaceWrapper::is_sensor_http_reachable()
{
  constexpr uint16_t http_probe_port = 80;
  constexpr int http_probe_timeout_ms = 250;

  try {
    drivers::connections::TcpSocket::Builder(sensor_configuration_->sensor_ip, http_probe_port)
      .set_connect_timeout(http_probe_timeout_ms)
      .connect();
  } catch (const std::exception &) {
    return false;
  }

  return true;
}

void VelodyneHwInterfaceWrapper::monitor_sensor_reconnection()
{
  using namespace std::chrono_literals;

  if (!retry_hw_) {
    return;
  }

  if (sensor_operational_ && std::chrono::steady_clock::now() - last_packet_time_ > 1s) {
    sensor_operational_ = false;
    reconnect_configuration_applied_ = false;
    RCLCPP_WARN(logger_, "Velodyne UDP packets stopped.");
  }

  if (reconnect_configuration_applied_) {
    return;
  }

  if (!is_sensor_http_reachable()) {
    return;
  }

  status_ = hw_interface_->set_sensor_configuration(sensor_configuration_);
  if (status_ == Status::OK) {
    reconnect_configuration_applied_ = true;
    RCLCPP_INFO(logger_, "Velodyne configuration reapplied.");
  } else {
    RCLCPP_WARN_STREAM(
      logger_, "Velodyne LiDAR is reachable, but configuration failed: "
                 << util::to_string(status_) << ". Will retry.");
  }
}

Status VelodyneHwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<drivers::VelodyneHwInterface> VelodyneHwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

}  // namespace nebula::ros
