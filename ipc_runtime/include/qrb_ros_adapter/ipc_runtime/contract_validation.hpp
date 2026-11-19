// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "qrb/contract/common/v1/header.pb.h"
#include "qrb/contract/geometry/v1/pose_array.pb.h"
#include "qrb/contract/geometry/v1/pose_stamped.pb.h"
#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"
#include "qrb/contract/geometry/v1/twist_stamped.pb.h"
#include "qrb/contract/geometry/v1/vector3_stamped.pb.h"
#include "qrb/contract/nav/v1/odometry.pb.h"
#include "qrb/contract/sensor/v1/image_layout.pb.h"
#include "qrb/contract/sensor/v1/imu.pb.h"
#include "qrb/contract/sensor/v1/joint_state.pb.h"
#include "qrb/contract/sensor/v1/point_cloud2.pb.h"
#include "qrb/contract/tf2/v1/tf_message.pb.h"
#include "qrb/contract/vision/v1/detection_2d_array.pb.h"
#include "qrb/contract/vision/v1/detection_3d_array.pb.h"
#include "qrb_ros_adapter/ipc_runtime/contract_limits.hpp"
#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

namespace qrb::adapter
{

ConversionResult ValidateFrameId(const std::string & frameId);

/** @brief Checks `frame_id` and that `stamp.nanosec` is below one second. */
ConversionResult ValidateHeader(const qrb::contract::common::v1::Header & header);

ConversionResult ValidatePoseWithCovarianceStamped(
    const qrb::contract::geometry::v1::PoseWithCovarianceStamped & pose);

/**
 * @brief Checks the header, `child_frame_id` (as a frame_id), the pose as
 * ValidatePoseWithCovarianceStamped() does, and that the twist is finite with an
 * empty or 36-value covariance.
 */
ConversionResult ValidateOdometry(const qrb::contract::nav::v1::Odometry & odometry);

/** @brief Checks the header and that the twist is present and finite. */
ConversionResult ValidateTwistStamped(const qrb::contract::geometry::v1::TwistStamped & twist);

/**
 * @brief Checks the transform count, and for each transform its header,
 * `child_frame_id` (as a frame_id), a finite translation and a finite, non-zero rotation.
 */
ConversionResult ValidateTFMessage(const qrb::contract::tf2::v1::TFMessage & message);

/** @brief Checks the header, joint count/names and finite measurement arrays, each empty or
 * name-sized. */
ConversionResult ValidateJointState(const qrb::contract::sensor::v1::JointState & message);

/**
 * @brief Checks the header and that the pose is present, with a finite position and orientation
 * and an orientation that is not the zero quaternion, as for the pose of
 * ValidatePoseWithCovarianceStamped(). The quaternion may be unnormalized; it is passed on as it
 * is. A missing orientation reads as zero and is rejected.
 */
ConversionResult ValidatePoseStamped(const qrb::contract::geometry::v1::PoseStamped & pose);

/**
 * @brief Checks the pose count (kMaxPoseCount), the header, and each pose as
 * ValidatePoseStamped() does its pose. An empty array is valid: "no pose found" is a result.
 */
ConversionResult ValidatePoseArray(const qrb::contract::geometry::v1::PoseArray & poses);

/** @brief Checks the header and that the vector is present and finite. */
ConversionResult ValidateVector3Stamped(const qrb::contract::geometry::v1::Vector3Stamped & vector);

ConversionResult ValidatePointCloud2(const qrb::contract::sensor::v1::PointCloud2 & cloud,
    std::uint64_t externalPayloadSize);

/** @brief Physical storage of one image plane and the valid pixel rows within it. */
struct ImagePlaneLayout
{
  std::uint64_t offset{0};        ///< Start of the plane, relative to the payload.
  std::uint64_t stride{0};        ///< Bytes per physical row, including row padding.
  std::uint64_t actualHeight{0};  ///< Allocated rows, including height padding.
  std::uint64_t bufferSize{0};    ///< Bytes the plane occupies, padding included.
  std::uint64_t rowBytes{0};      ///< Valid pixel bytes in each row.
  std::uint64_t rows{0};          ///< Valid pixel rows.
};

/** @brief Physical storage of a single-frame image: one plane for rgb8/bgr8, Y and UV for nv12. */
struct ImageLayout
{
  std::array<ImagePlaneLayout, 2> planes{};
  std::size_t planeCount{0};
  std::uint64_t step{0};         ///< Logical row bytes, as in ImageLayout.step.
  std::uint64_t storageSize{0};  ///< Bytes of the whole storage.
  std::uint64_t pixelSize{0};    ///< Bytes of valid pixels.
};

/**
 * @brief The layout qrb_ros_transport allocates for an image.
 *
 * Mirrors qrb_ros_transport image_utils: rows are aligned to 128 pixels, plane
 * heights to 32 rows, rgb8/bgr8 allocations to 256 pixels and every allocation to
 * 4096 bytes. The ROS side carries no plane metadata, so both sides rely on this
 * rule; it must change together with qrb_ros_transport.
 */
ConversionResult CanonicalImageLayout(std::uint32_t width,
    std::uint32_t height,
    const std::string & encoding,
    ImageLayout & layout);

/**
 * @brief The layout of `image`'s pixels in an external payload of `externalPayloadSize` bytes.
 *
 * Empty planes mean the qrb_ros_transport layout, not tight storage. Explicit
 * planes describe a native layout; each starts right after the previous plane's
 * buffer_size, relative to the payload offset.
 */
ConversionResult ResolveImageLayout(const qrb::contract::sensor::v1::ImageLayout & image,
    std::uint64_t externalPayloadSize,
    ImageLayout & layout);

/** @brief Checks the header, shape, encoding and layout of an ImageLayout Contract. */
ConversionResult ValidateImageLayout(const qrb::contract::sensor::v1::ImageLayout & image,
    std::uint64_t externalPayloadSize);

/**
 * @brief Checks the header, presence and finiteness of orientation,
 * angular_velocity and linear_acceleration, and that each covariance array is
 * empty or contains exactly 9 values.
 *
 * Per REP-145, a zero orientation quaternion is accepted when
 * orientation_covariance[0] == -1, which signals that orientation is unknown.
 */
ConversionResult ValidateImu(const qrb::contract::sensor::v1::Imu & imu);

/**
 * @brief Checks a detection array a peer sent or this side built.
 *
 * Every number must be finite and every box size non-negative; `id` and `class_id` must fit
 * contract_limits::kMaxDetectionStringLength and hold no NUL; a covariance is empty (unknown)
 * or has 36 values. The detector may leave the quaternions of a box or a hypothesis pose
 * unnormalized, or all zero, since many publishers never set them; they are passed on as
 * they are. An empty array is valid: "nothing detected" is a result.
 */
ConversionResult ValidateDetection2DArray(
    const qrb::contract::vision::v1::Detection2DArray & detections);

ConversionResult ValidateDetection3DArray(
    const qrb::contract::vision::v1::Detection3DArray & detections);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_VALIDATION_HPP
