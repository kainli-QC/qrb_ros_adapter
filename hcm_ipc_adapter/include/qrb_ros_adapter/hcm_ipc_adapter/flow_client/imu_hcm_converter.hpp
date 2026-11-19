// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#pragma once

#include "QRB/Memory/Sensor/ImuDescriptor.hpp"
#include "qrb/contract/sensor/v1/imu.pb.h"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_conversion.hpp"

namespace qrb::adapter
{

using ImuHcmContract   = ::qrb::contract::sensor::v1::Imu;
using ImuHcmDescriptor = QRB::Memory::ImuDescriptor_t;

template <>
ConversionResult ConvertContractToHcm<ImuHcmContract, ImuHcmDescriptor>(
    const ImuHcmContract & source,
    const DmaBufView & payload,
    ImuHcmDescriptor & target);

template <>
ConversionResult ConvertHcmToContract<ImuHcmContract, ImuHcmDescriptor>(
    const ImuHcmDescriptor & source,
    ImuHcmContract & target);

}  // namespace qrb::adapter
