// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/imu_hcm_converter.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter {

template <>
ConversionResult ConvertContractToHcm<ImuHcmContract, ImuHcmDescriptor>(
    const ImuHcmContract &imu, const DmaBufView & /*payload*/,
    ImuHcmDescriptor &descriptor) {
  ConversionResult validation = ValidateImu(imu);
  if (!validation)
    return validation;

  std::uint64_t timestampNs = 0;
  ConversionResult time =
      ContractTimeToNanos(imu.header().stamp(), timestampNs);
  if (!time)
    return time;

  if (!AssignHcmString(imu.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure(
        "frame_id does not fit the HCM IMU descriptor");
  descriptor.timestampNs = timestampNs;

  descriptor.orientation.x = imu.orientation().x();
  descriptor.orientation.y = imu.orientation().y();
  descriptor.orientation.z = imu.orientation().z();
  descriptor.orientation.w = imu.orientation().w();
  std::fill(std::begin(descriptor.orientation_covariance),
            std::end(descriptor.orientation_covariance), -1.0);
  for (int i = 0; i < imu.orientation_covariance_size(); ++i)
    descriptor.orientation_covariance[static_cast<std::size_t>(i)] =
        imu.orientation_covariance(i);

  descriptor.angular_velocity.x = imu.angular_velocity().x();
  descriptor.angular_velocity.y = imu.angular_velocity().y();
  descriptor.angular_velocity.z = imu.angular_velocity().z();
  std::fill(std::begin(descriptor.angular_velocity_covariance),
            std::end(descriptor.angular_velocity_covariance), -1.0);
  for (int i = 0; i < imu.angular_velocity_covariance_size(); ++i)
    descriptor.angular_velocity_covariance[static_cast<std::size_t>(i)] =
        imu.angular_velocity_covariance(i);

  descriptor.linear_acceleration.x = imu.linear_acceleration().x();
  descriptor.linear_acceleration.y = imu.linear_acceleration().y();
  descriptor.linear_acceleration.z = imu.linear_acceleration().z();
  std::fill(std::begin(descriptor.linear_acceleration_covariance),
            std::end(descriptor.linear_acceleration_covariance), -1.0);
  for (int i = 0; i < imu.linear_acceleration_covariance_size(); ++i)
    descriptor.linear_acceleration_covariance[static_cast<std::size_t>(i)] =
        imu.linear_acceleration_covariance(i);

  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<ImuHcmContract, ImuHcmDescriptor>(
    const ImuHcmDescriptor &descriptor, ImuHcmContract &imu) {
  imu.Clear();
  ConversionResult time = NanosToContractTime(
      descriptor.timestampNs, *imu.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  imu.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));

  auto *orientation = imu.mutable_orientation();
  orientation->set_x(descriptor.orientation.x);
  orientation->set_y(descriptor.orientation.y);
  orientation->set_z(descriptor.orientation.z);
  orientation->set_w(descriptor.orientation.w);
  for (double value : descriptor.orientation_covariance)
    imu.add_orientation_covariance(value);

  auto *angularVelocity = imu.mutable_angular_velocity();
  angularVelocity->set_x(descriptor.angular_velocity.x);
  angularVelocity->set_y(descriptor.angular_velocity.y);
  angularVelocity->set_z(descriptor.angular_velocity.z);
  for (double value : descriptor.angular_velocity_covariance)
    imu.add_angular_velocity_covariance(value);

  auto *linearAcceleration = imu.mutable_linear_acceleration();
  linearAcceleration->set_x(descriptor.linear_acceleration.x);
  linearAcceleration->set_y(descriptor.linear_acceleration.y);
  linearAcceleration->set_z(descriptor.linear_acceleration.z);
  for (double value : descriptor.linear_acceleration_covariance)
    imu.add_linear_acceleration_covariance(value);

  return ValidateImu(imu);
}

} // namespace qrb::adapter
