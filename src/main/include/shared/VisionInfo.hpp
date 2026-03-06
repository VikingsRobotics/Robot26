#pragma once
#ifndef SHARED_VISION_INFO_H
#define SHARED_VISION_INFO_H

#include <cstdint>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/struct/Transform3dStruct.h>
#include <units/time.h>
#include <wpi/struct/Struct.h>

enum class Camera : uint16_t { kFront = 0, kBack = 1, kLeft = 2, kRight = 3, kUnknown = 4 };

struct VisionInfo {
    Camera detectingCamera;
    uint16_t detectedID;
    float detectorConfidence;
    frc::Transform3d estimatedPose;
    units::millisecond_t estimatedLatency;
};

template <>
struct wpi::Struct<VisionInfo> {
    static constexpr size_t kCameraOffset = 0;
    static constexpr size_t kIdOffset = kCameraOffset + wpi::Struct<uint16_t>::GetSize();
    static constexpr size_t kConfidenceOffset = kIdOffset + wpi::Struct<uint16_t>::GetSize();
    static constexpr size_t kPoseOffset = kConfidenceOffset + wpi::Struct<float>::GetSize();
    static constexpr size_t kLatencyOffset = kPoseOffset + wpi::Struct<frc::Transform3d>::GetSize();

    static constexpr std::string_view GetTypeName() { return "struct:VisionInfo"; }

    static constexpr size_t GetSize() {
        return wpi::Struct<uint16_t>::GetSize() +          // Camera
               wpi::Struct<uint16_t>::GetSize() +          // detectedID
               wpi::Struct<float>::GetSize() +             // detectorConfidence
               wpi::Struct<frc::Transform3d>::GetSize() +  // pose
               wpi::Struct<double>::GetSize();             // estimatedLatency (milliseconds)
    }

    static constexpr std::string_view GetSchema() {
        return "uint16 detectingCamera;"
               "uint16 detectedID;"
               "float detectorConfidence;"
               "Transform3d estimatedPose;"
               "double estimatedLatency;";
    }

    static void Pack(std::span<uint8_t> buffer, const VisionInfo& value) {
        wpi::PackStruct<kCameraOffset, uint16_t>(buffer, static_cast<uint16_t>(value.detectingCamera));
        wpi::PackStruct<kIdOffset, uint16_t>(buffer, static_cast<uint16_t>(value.detectedID));
        wpi::PackStruct<kConfidenceOffset, float>(buffer, static_cast<float>(value.detectorConfidence));
        wpi::PackStruct<kPoseOffset, frc::Transform3d>(buffer, static_cast<frc::Transform3d>(value.estimatedPose));
        wpi::PackStruct<kLatencyOffset, double>(buffer, value.estimatedLatency.value());
    }

    static VisionInfo Unpack(std::span<const uint8_t> buffer) {
        VisionInfo value{};

        value.detectingCamera = static_cast<Camera>(wpi::UnpackStruct<uint16_t, kCameraOffset>(buffer));

        value.detectedID = wpi::UnpackStruct<uint16_t, kIdOffset>(buffer);

        value.detectorConfidence = wpi::UnpackStruct<float, kConfidenceOffset>(buffer);

        value.estimatedPose = wpi::UnpackStruct<frc::Transform3d, kPoseOffset>(buffer);

        value.estimatedLatency = units::millisecond_t{wpi::UnpackStruct<double, kLatencyOffset>(buffer)};

        return value;
    }
};

constexpr std::string_view VISION_NETWORKTABLE_NAME = "VisionTable";

#endif