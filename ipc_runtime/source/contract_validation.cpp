// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

#include <cmath>
#include <limits>
#include <string>

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"

namespace qrb::adapter
{

namespace
{

ConversionResult Fail(const char * message)
{
  return ConversionResult::Failure(message);
}

bool IsFinitePose(const qrb::contract::geometry::v1::Pose & pose)
{
  const auto & position = pose.position();
  const auto & orientation = pose.orientation();
  return std::isfinite(position.x()) && std::isfinite(position.y()) &&
         std::isfinite(position.z()) && std::isfinite(orientation.x()) &&
         std::isfinite(orientation.y()) && std::isfinite(orientation.z()) &&
         std::isfinite(orientation.w());
}

bool IsFiniteVector3(const qrb::contract::geometry::v1::Vector3 & vector)
{
  return std::isfinite(vector.x()) && std::isfinite(vector.y()) && std::isfinite(vector.z());
}

/** @brief A 6x6 covariance is either unknown (empty) or 36 finite values. */
ConversionResult ValidateCovariance(const google::protobuf::RepeatedField<double> & covariance,
    const char * sizeMessage,
    const char * finiteMessage)
{
  if (covariance.size() != 0 && covariance.size() != 36)
    return Fail(sizeMessage);
  for (double value : covariance) {
    if (!std::isfinite(value))
      return Fail(finiteMessage);
  }
  return ConversionResult::Success();
}

bool IsFiniteTwist(const qrb::contract::geometry::v1::Twist & twist)
{
  return IsFiniteVector3(twist.linear()) && IsFiniteVector3(twist.angular());
}

bool IsZeroQuaternion(const qrb::contract::geometry::v1::Quaternion & quaternion)
{
  return quaternion.x() == 0.0 && quaternion.y() == 0.0 && quaternion.z() == 0.0 &&
         quaternion.w() == 0.0;
}

/** @brief The rules for a Pose, wherever a Contract embeds one without a covariance. */
ConversionResult ValidatePoseValue(const qrb::contract::geometry::v1::Pose & pose)
{
  if (!IsFinitePose(pose))
    return Fail("pose contains non-finite values");
  if (IsZeroQuaternion(pose.orientation()))
    return Fail("orientation quaternion must be non-zero");
  return ConversionResult::Success();
}

/** @brief The rules for a PoseWithCovariance, wherever a Contract embeds one. */
ConversionResult ValidatePoseWithCovariance(
    const qrb::contract::geometry::v1::PoseWithCovariance & value)
{
  if (!value.has_pose() || !IsFinitePose(value.pose()))
    return Fail("pose contains missing or non-finite values");
  const auto & orientation = value.pose().orientation();
  if (orientation.x() == 0.0 && orientation.y() == 0.0 && orientation.z() == 0.0 &&
      orientation.w() == 0.0)
    return Fail("orientation quaternion must be non-zero");
  return ValidateCovariance(value.covariance(), "covariance must be empty or contain 36 values",
      "covariance contains a non-finite value");
}

std::uint32_t PointFieldElementSize(qrb::contract::sensor::v1::PointFieldType type)
{
  using namespace qrb::contract::sensor::v1;
  switch (type) {
    case POINT_FIELD_TYPE_INT8:
    case POINT_FIELD_TYPE_UINT8:
      return 1;
    case POINT_FIELD_TYPE_INT16:
    case POINT_FIELD_TYPE_UINT16:
      return 2;
    case POINT_FIELD_TYPE_INT32:
    case POINT_FIELD_TYPE_UINT32:
    case POINT_FIELD_TYPE_FLOAT32:
      return 4;
    case POINT_FIELD_TYPE_FLOAT64:
      return 8;
    default:
      return 0;
  }
}

bool IsNonNegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

std::string DetectionFailure(int index, const char * message)
{
  return "detection " + std::to_string(index) + ": " + message;
}

ConversionResult ValidateDetectionString(const std::string & value, const char * name)
{
  if (value.size() > contract_limits::kMaxDetectionStringLength)
    return Fail((std::string(name) + " exceeds the contract limit").c_str());
  if (value.find('\0') != std::string::npos)
    return Fail((std::string(name) + " must not contain NUL").c_str());
  return ConversionResult::Success();
}

ConversionResult ValidateHypothesis(
    const qrb::contract::vision::v1::ObjectHypothesisWithPose & result)
{
  if (!result.has_hypothesis())
    return Fail("hypothesis is required");
  ConversionResult classId = ValidateDetectionString(result.hypothesis().class_id(), "class_id");
  if (!classId)
    return classId;
  if (!std::isfinite(result.hypothesis().score()))
    return Fail("hypothesis score must be finite");
  // The pose is optional: an absent one reads as the identity with unknown covariance.
  if (!result.has_pose())
    return ConversionResult::Success();
  if (!IsFinitePose(result.pose().pose()))
    return Fail("hypothesis pose contains non-finite values");
  return ValidateCovariance(result.pose().covariance(),
      "hypothesis covariance must be empty or contain 36 values",
      "hypothesis covariance contains a non-finite value");
}

ConversionResult ValidateBox(const qrb::contract::vision::v1::BoundingBox2D & box)
{
  const auto & center = box.center();
  if (!std::isfinite(center.position().x()) || !std::isfinite(center.position().y()) ||
      !std::isfinite(center.theta()))
    return Fail("bbox center contains non-finite values");
  if (!IsNonNegative(box.size_x()) || !IsNonNegative(box.size_y()))
    return Fail("bbox size must be finite and not negative");
  return ConversionResult::Success();
}

ConversionResult ValidateBox(const qrb::contract::vision::v1::BoundingBox3D & box)
{
  if (!IsFinitePose(box.center()))
    return Fail("bbox center contains non-finite values");
  if (!IsFiniteVector3(box.size()) || !IsNonNegative(box.size().x()) ||
      !IsNonNegative(box.size().y()) || !IsNonNegative(box.size().z()))
    return Fail("bbox size must be finite and not negative");
  return ConversionResult::Success();
}

/** @brief The checks Detection2DArray and Detection3DArray share. */
template <typename ArrayT>
ConversionResult ValidateDetectionArray(const ArrayT & array)
{
  ConversionResult header = ValidateHeader(array.header());
  if (!header)
    return header;
  int index = 0;
  for (const auto & detection : array.detections()) {
    ConversionResult id = ValidateDetectionString(detection.id(), "id");
    if (!id)
      return ConversionResult::Failure(DetectionFailure(index, id.Diagnostic().c_str()));
    if (!detection.has_bbox())
      return ConversionResult::Failure(DetectionFailure(index, "bbox is required"));
    ConversionResult box = ValidateBox(detection.bbox());
    if (!box)
      return ConversionResult::Failure(DetectionFailure(index, box.Diagnostic().c_str()));
    for (const auto & result : detection.results()) {
      ConversionResult hypothesis = ValidateHypothesis(result);
      if (!hypothesis) {
        return ConversionResult::Failure(DetectionFailure(index, hypothesis.Diagnostic().c_str()));
      }
    }
    ++index;
  }
  return ConversionResult::Success();
}

}  // namespace

ConversionResult ValidateFrameId(const std::string & frameId)
{
  if (frameId.size() > contract_limits::kMaxFrameIdLength)
    return Fail("frame_id exceeds the contract limit");
  if (!frameId.empty() && frameId.front() == '/')
    return Fail("frame_id must not start with '/'");
  if (frameId.find('\0') != std::string::npos)
    return Fail("frame_id must not contain NUL");
  return ConversionResult::Success();
}

ConversionResult ValidateHeader(const qrb::contract::common::v1::Header & header)
{
  ConversionResult frameId = ValidateFrameId(header.frame_id());
  if (!frameId)
    return frameId;
  if (header.stamp().nanosec() >= kNanosPerSecond)
    return Fail("header.stamp.nanosec must be below one second");
  return ConversionResult::Success();
}

ConversionResult ValidatePoseWithCovarianceStamped(
    const qrb::contract::geometry::v1::PoseWithCovarianceStamped & pose)
{
  ConversionResult header = ValidateHeader(pose.header());
  if (!header)
    return header;
  if (!pose.has_pose())
    return Fail("pose is required");
  return ValidatePoseWithCovariance(pose.pose());
}

ConversionResult ValidateOdometry(const qrb::contract::nav::v1::Odometry & odometry)
{
  ConversionResult header = ValidateHeader(odometry.header());
  if (!header)
    return header;
  ConversionResult childFrameId = ValidateFrameId(odometry.child_frame_id());
  if (!childFrameId)
    return ConversionResult::Failure("child_frame_id: " + childFrameId.Diagnostic());

  if (!odometry.has_pose())
    return Fail("pose is required");
  ConversionResult pose = ValidatePoseWithCovariance(odometry.pose());
  if (!pose)
    return pose;

  if (!odometry.has_twist())
    return Fail("twist is required");
  const auto & value = odometry.twist();
  if (!value.has_twist() || !IsFiniteTwist(value.twist()))
    return Fail("twist contains missing or non-finite values");
  return ValidateCovariance(value.covariance(),
      "twist covariance must be empty or contain 36 values",
      "twist covariance contains a non-finite value");
}

ConversionResult ValidateTwistStamped(const qrb::contract::geometry::v1::TwistStamped & twist)
{
  ConversionResult header = ValidateHeader(twist.header());
  if (!header)
    return header;
  if (!twist.has_twist() || !IsFiniteTwist(twist.twist()))
    return Fail("twist contains missing or non-finite values");
  return ConversionResult::Success();
}

ConversionResult ValidateTFMessage(const qrb::contract::tf2::v1::TFMessage & message)
{
  if (static_cast<std::size_t>(message.transforms_size()) > contract_limits::kMaxTransformCount)
    return Fail("TFMessage holds more transforms than the contract limit");
  for (int i = 0; i < message.transforms_size(); ++i) {
    const auto & transform = message.transforms(i);
    const std::string where = "transforms[" + std::to_string(i) + "]: ";
    ConversionResult header = ValidateHeader(transform.header());
    if (!header)
      return ConversionResult::Failure(where + header.Diagnostic());
    ConversionResult childFrameId = ValidateFrameId(transform.child_frame_id());
    if (!childFrameId)
      return ConversionResult::Failure(where + "child_frame_id: " + childFrameId.Diagnostic());
    if (!transform.has_transform())
      return ConversionResult::Failure(where + "transform is required");
    const auto & translation = transform.transform().translation();
    const auto & rotation = transform.transform().rotation();
    if (!IsFiniteVector3(translation) || !std::isfinite(rotation.x()) ||
        !std::isfinite(rotation.y()) || !std::isfinite(rotation.z()) ||
        !std::isfinite(rotation.w()))
      return ConversionResult::Failure(where + "transform contains non-finite values");
    if (rotation.x() == 0.0 && rotation.y() == 0.0 && rotation.z() == 0.0 && rotation.w() == 0.0)
      return ConversionResult::Failure(where + "rotation quaternion must be non-zero");
  }
  return ConversionResult::Success();
}

ConversionResult ValidateJointState(const qrb::contract::sensor::v1::JointState & message)
{
  ConversionResult header = ValidateHeader(message.header());
  if (!header)
    return header;
  if (static_cast<std::size_t>(message.name_size()) > contract_limits::kMaxJointCount)
    return Fail("JointState holds more joints than the contract limit");
  for (const auto & name : message.name()) {
    if (name.empty() || name.size() > contract_limits::kMaxJointNameLength)
      return Fail("joint name is outside the contract limit");
    if (name.find('\0') != std::string::npos)
      return Fail("joint name must not contain NUL");
  }
  const struct
  {
    const google::protobuf::RepeatedField<double> & values;
    const char * name;
  } arrays[] = {{message.position(), "position"}, {message.velocity(), "velocity"},
      {message.effort(), "effort"}};
  for (const auto & array : arrays) {
    if (!array.values.empty() && array.values.size() != message.name_size())
      return ConversionResult::Failure(
          std::string(array.name) + " must be empty or match name size");
    for (double value : array.values) {
      if (!std::isfinite(value))
        return ConversionResult::Failure(std::string(array.name) + " contains a non-finite value");
    }
  }
  return ConversionResult::Success();
}

ConversionResult ValidatePoseStamped(const qrb::contract::geometry::v1::PoseStamped & pose)
{
  ConversionResult header = ValidateHeader(pose.header());
  if (!header)
    return header;
  if (!pose.has_pose())
    return Fail("pose is required");
  return ValidatePoseValue(pose.pose());
}

ConversionResult ValidatePoseArray(const qrb::contract::geometry::v1::PoseArray & poses)
{
  if (static_cast<std::size_t>(poses.poses_size()) > contract_limits::kMaxPoseCount)
    return Fail("PoseArray holds more poses than the contract limit");
  ConversionResult header = ValidateHeader(poses.header());
  if (!header)
    return header;
  for (int i = 0; i < poses.poses_size(); ++i) {
    ConversionResult pose = ValidatePoseValue(poses.poses(i));
    if (!pose)
      return ConversionResult::Failure("poses[" + std::to_string(i) + "]: " + pose.Diagnostic());
  }
  return ConversionResult::Success();
}

ConversionResult ValidateVector3Stamped(const qrb::contract::geometry::v1::Vector3Stamped & vector)
{
  ConversionResult header = ValidateHeader(vector.header());
  if (!header)
    return header;
  if (!vector.has_vector() || !IsFiniteVector3(vector.vector()))
    return Fail("vector contains missing or non-finite values");
  return ConversionResult::Success();
}

ConversionResult ValidatePointCloud2(const qrb::contract::sensor::v1::PointCloud2 & cloud,
    std::uint64_t externalPayloadSize)
{
  ConversionResult header = ValidateHeader(cloud.header());
  if (!header)
    return header;
  if (cloud.height() == 0 || cloud.width() == 0)
    return Fail("point cloud dimensions must be non-zero");
  if (cloud.point_step() == 0)
    return Fail("point_step must be non-zero");
  if (cloud.width() > std::numeric_limits<std::uint32_t>::max() / cloud.point_step())
    return Fail("point_step * width overflows");
  const std::uint32_t minimumRowStep = cloud.point_step() * cloud.width();
  if (cloud.row_step() < minimumRowStep)
    return Fail("row_step is smaller than point_step * width");
  if (cloud.height() > std::numeric_limits<std::uint64_t>::max() / cloud.row_step())
    return Fail("row_step * height overflows");
  const std::uint64_t payloadSize = static_cast<std::uint64_t>(cloud.row_step()) * cloud.height();
  if (externalPayloadSize < payloadSize)
    return Fail("external payload is smaller than the point cloud layout");
  if (!cloud.data().empty() && cloud.data().size() != payloadSize)
    return Fail("embedded data size does not match the point cloud layout");

  if (static_cast<std::size_t>(cloud.fields_size()) > contract_limits::kMaxPointFieldCount)
    return Fail("point field count is outside the contract limit");
  for (const auto & field : cloud.fields()) {
    if (field.name().empty() || field.name().size() > contract_limits::kMaxPointFieldNameLength)
      return Fail("point field name is outside the contract limit");
    if (field.name().find('\0') != std::string::npos)
      return Fail("point field name must not contain NUL");
    const std::uint32_t elementSize = PointFieldElementSize(field.datatype());
    if (elementSize == 0)
      return Fail("point field datatype is invalid");
    if (field.count() == 0 || field.count() > contract_limits::kMaxPointFieldElementCount)
      return Fail("point field element count is outside the contract limit");
    const std::uint64_t fieldEnd = static_cast<std::uint64_t>(field.offset()) +
                                   static_cast<std::uint64_t>(elementSize) * field.count();
    if (fieldEnd > cloud.point_step())
      return Fail("point field extends beyond point_step");
  }
  return ConversionResult::Success();
}

namespace
{

bool CheckedMultiply(std::uint64_t a, std::uint64_t b, std::uint64_t & result)
{
  if (b != 0 && a > std::numeric_limits<std::uint64_t>::max() / b)
    return false;
  result = a * b;
  return true;
}

bool CheckedAdd(std::uint64_t a, std::uint64_t b, std::uint64_t & result)
{
  if (a > std::numeric_limits<std::uint64_t>::max() - b)
    return false;
  result = a + b;
  return true;
}

bool AlignUp(std::uint64_t value, std::uint64_t alignment, std::uint64_t & result)
{
  const std::uint64_t remainder = value % alignment;
  return CheckedAdd(value, remainder == 0 ? 0 : alignment - remainder, result);
}

}  // namespace

ConversionResult CanonicalImageLayout(std::uint32_t width,
    std::uint32_t height,
    const std::string & encoding,
    ImageLayout & layout)
{
  if (width == 0 || height == 0)
    return Fail("image dimensions must be non-zero");
  const bool nv12 = encoding == "nv12";
  if (!nv12 && encoding != "rgb8" && encoding != "bgr8")
    return Fail("image encoding is unsupported");
  if (nv12 && (width % 2 != 0 || height % 2 != 0))
    return Fail("nv12 image dimensions must be even");

  ImageLayout value;
  std::uint64_t pitchWidth = 0;
  std::uint64_t allocationWidth = 0;
  std::uint64_t actualHeight = 0;
  std::uint64_t pitch = 0;
  std::uint64_t planeStorage = 0;
  std::uint64_t allocation = 0;
  const std::uint64_t channels = nv12 ? 1 : 3;
  if (!AlignUp(width, 128, pitchWidth) || !AlignUp(width, nv12 ? 128 : 256, allocationWidth) ||
      !AlignUp(height, 32, actualHeight) || !CheckedMultiply(width, channels, value.step) ||
      !CheckedMultiply(pitchWidth, channels, pitch) ||
      !CheckedMultiply(pitch, actualHeight, planeStorage) ||
      !CheckedMultiply(allocationWidth, actualHeight, allocation) ||
      !CheckedMultiply(allocation, channels, allocation) ||
      (nv12 && !CheckedAdd(allocation, allocation / 2, allocation)) ||
      !AlignUp(allocation, 4096, allocation) ||
      !CheckedMultiply(value.step, height, value.pixelSize) ||
      (nv12 && !CheckedAdd(value.pixelSize, value.pixelSize / 2, value.pixelSize))) {
    return Fail("image layout arithmetic overflows");
  }
  // qrb_ros_transport computes the aligned sizes with signed int.
  if (allocation > static_cast<std::uint64_t>(std::numeric_limits<int>::max()) ||
      pitch > std::numeric_limits<std::uint32_t>::max() ||
      actualHeight > std::numeric_limits<std::uint32_t>::max() ||
      value.step > std::numeric_limits<std::uint32_t>::max()) {
    return Fail("image layout is not representable by the ROS transport");
  }

  value.planeCount = nv12 ? 2 : 1;
  value.storageSize = allocation;
  value.planes[0] = {0, pitch, actualHeight, nv12 ? planeStorage : allocation, value.step, height};
  if (nv12) {
    value.planes[1] = {
        planeStorage, pitch, actualHeight / 2, allocation - planeStorage, value.step, height / 2};
  }
  layout = value;
  return ConversionResult::Success();
}

ConversionResult ResolveImageLayout(const qrb::contract::sensor::v1::ImageLayout & image,
    std::uint64_t externalPayloadSize,
    ImageLayout & layout)
{
  ConversionResult header = ValidateHeader(image.header());
  if (!header)
    return header;
  if (image.is_bigendian())
    return Fail("image is_bigendian must be false");
  if (!image.data().empty())
    return Fail("image data must travel in the external DMA-BUF");

  ImageLayout value;
  ConversionResult canonical =
      CanonicalImageLayout(image.width(), image.height(), image.encoding(), value);
  if (!canonical)
    return canonical;
  if (image.step() != value.step)
    return Fail("image step must equal the logical row byte count");
  if (image.planes_size() == 0) {
    if (image.batch_size() > 1)
      return Fail("image batch_size must be zero or one");
  } else {
    if (image.batch_size() != 1)
      return Fail("explicit image planes require batch_size one");
    if (static_cast<std::size_t>(image.planes_size()) != value.planeCount)
      return Fail("image plane count does not match the encoding");
    value.storageSize = 0;
    for (std::size_t i = 0; i < value.planeCount; ++i) {
      const auto & plane = image.planes(static_cast<int>(i));
      ImagePlaneLayout & resolved = value.planes[i];
      std::uint64_t minimumSize = 0;
      if (plane.stride() < resolved.rowBytes || plane.actual_height() < resolved.rows ||
          !CheckedMultiply(plane.stride(), plane.actual_height(), minimumSize) ||
          plane.buffer_size() < minimumSize) {
        return Fail("image plane storage cannot cover its rows");
      }
      resolved.offset = value.storageSize;
      resolved.stride = plane.stride();
      resolved.actualHeight = plane.actual_height();
      resolved.bufferSize = plane.buffer_size();
      if (!CheckedAdd(value.storageSize, resolved.bufferSize, value.storageSize))
        return Fail("image plane sizes overflow");
    }
  }
  if (value.storageSize > externalPayloadSize)
    return Fail("external payload is smaller than the image layout");
  layout = value;
  return ConversionResult::Success();
}

ConversionResult ValidateImageLayout(const qrb::contract::sensor::v1::ImageLayout & image,
    std::uint64_t externalPayloadSize)
{
  ImageLayout layout;
  return ResolveImageLayout(image, externalPayloadSize, layout);
}

ConversionResult ValidateImu(const qrb::contract::sensor::v1::Imu & imu)
{
  if (!imu.has_header())
    return Fail("IMU header is missing");
  ConversionResult header = ValidateHeader(imu.header());
  if (!header)
    return header;

  if (!imu.has_orientation())
    return Fail("IMU orientation is required");
  const auto & q = imu.orientation();
  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) ||
      !std::isfinite(q.z()) || !std::isfinite(q.w()))
    return Fail("IMU orientation contains non-finite values");
  // Per REP-145: orientation_covariance[0] == -1 signals that orientation is
  // unknown; the quaternion values are meaningless and may be all zero.
  const bool orientationUnknown =
      imu.orientation_covariance_size() > 0 && imu.orientation_covariance(0) == -1.0;
  if (!orientationUnknown &&
      q.x() == 0.0 && q.y() == 0.0 && q.z() == 0.0 && q.w() == 0.0)
    return Fail("IMU orientation quaternion must be non-zero");

  if (!imu.has_angular_velocity())
    return Fail("IMU angular_velocity is required");
  const auto & av = imu.angular_velocity();
  if (!std::isfinite(av.x()) || !std::isfinite(av.y()) || !std::isfinite(av.z()))
    return Fail("IMU angular_velocity contains non-finite values");

  if (!imu.has_linear_acceleration())
    return Fail("IMU linear_acceleration is required");
  const auto & la = imu.linear_acceleration();
  if (!std::isfinite(la.x()) || !std::isfinite(la.y()) || !std::isfinite(la.z()))
    return Fail("IMU linear_acceleration contains non-finite values");

  if (imu.orientation_covariance_size() != 0 && imu.orientation_covariance_size() != 9)
    return Fail("orientation_covariance must be empty or contain 9 values");
  if (imu.angular_velocity_covariance_size() != 0 &&
      imu.angular_velocity_covariance_size() != 9)
    return Fail("angular_velocity_covariance must be empty or contain 9 values");
  if (imu.linear_acceleration_covariance_size() != 0 &&
      imu.linear_acceleration_covariance_size() != 9)
    return Fail("linear_acceleration_covariance must be empty or contain 9 values");

  return ConversionResult::Success();
}
ConversionResult ValidateDetection2DArray(
    const qrb::contract::vision::v1::Detection2DArray & detections)
{
  return ValidateDetectionArray(detections);
}

ConversionResult ValidateDetection3DArray(
    const qrb::contract::vision::v1::Detection3DArray & detections)
{
  return ValidateDetectionArray(detections);
}

}  // namespace qrb::adapter
