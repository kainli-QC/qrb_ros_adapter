// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/imu_ros_converter.hpp"

#include <cstddef>

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

ConversionResult ImuRosConversion::RosToContract(const ImuRosMessage & source,
    ImuRosContract & target)
{
  target.Clear();
  HeaderToContract(source.header, *target.mutable_header());

  auto * orientation = target.mutable_orientation();
  orientation->set_x(source.orientation.x);
  orientation->set_y(source.orientation.y);
  orientation->set_z(source.orientation.z);
  orientation->set_w(source.orientation.w);
  for (double value : source.orientation_covariance)
    target.add_orientation_covariance(value);

  auto * angularVelocity = target.mutable_angular_velocity();
  angularVelocity->set_x(source.angular_velocity.x);
  angularVelocity->set_y(source.angular_velocity.y);
  angularVelocity->set_z(source.angular_velocity.z);
  for (double value : source.angular_velocity_covariance)
    target.add_angular_velocity_covariance(value);

  auto * linearAcceleration = target.mutable_linear_acceleration();
  linearAcceleration->set_x(source.linear_acceleration.x);
  linearAcceleration->set_y(source.linear_acceleration.y);
  linearAcceleration->set_z(source.linear_acceleration.z);
  for (double value : source.linear_acceleration_covariance)
    target.add_linear_acceleration_covariance(value);

  return ValidateImu(target);
}

ConversionResult ImuRosConversion::ContractToRos(const ImuRosContract & source,
    ImuRosMessage & target)
{
  ConversionResult validation = ValidateImu(source);
  if (!validation)
    return validation;

  HeaderToRos(source.header(), target.header);

  const auto & orientation = source.orientation();
  target.orientation.x = orientation.x();
  target.orientation.y = orientation.y();
  target.orientation.z = orientation.z();
  target.orientation.w = orientation.w();
  target.orientation_covariance.fill(0.0);
  for (int i = 0; i < source.orientation_covariance_size() && i < 9; ++i)
    target.orientation_covariance[static_cast<std::size_t>(i)] = source.orientation_covariance(i);

  const auto & angularVelocity = source.angular_velocity();
  target.angular_velocity.x = angularVelocity.x();
  target.angular_velocity.y = angularVelocity.y();
  target.angular_velocity.z = angularVelocity.z();
  target.angular_velocity_covariance.fill(0.0);
  for (int i = 0; i < source.angular_velocity_covariance_size() && i < 9; ++i)
    target.angular_velocity_covariance[static_cast<std::size_t>(i)] =
        source.angular_velocity_covariance(i);

  const auto & linearAcceleration = source.linear_acceleration();
  target.linear_acceleration.x = linearAcceleration.x();
  target.linear_acceleration.y = linearAcceleration.y();
  target.linear_acceleration.z = linearAcceleration.z();
  target.linear_acceleration_covariance.fill(0.0);
  for (int i = 0; i < source.linear_acceleration_covariance_size() && i < 9; ++i)
    target.linear_acceleration_covariance[static_cast<std::size_t>(i)] =
        source.linear_acceleration_covariance(i);

  return ConversionResult::Success();
}

}  // namespace qrb::adapter
