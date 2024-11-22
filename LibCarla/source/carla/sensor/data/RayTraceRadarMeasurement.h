// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Debug.h"
#include "carla/sensor/data/Array.h"
#include "carla/sensor/s11n/RayTraceRadarSerializer.h"
#include "carla/sensor/data/RayTraceRadarData.h"

namespace carla {
namespace sensor {
namespace data {

  /// Measurement produced by a Radar. Consists of an array of RadarDetection.
  /// A RadarDetection contains 4 floats: velocity, azimuth, altitude and depth
  class RayTraceRadarMeasurement : public Array<data::RayTraceRadarDetection> {
    using Super = Array<data::RayTraceRadarDetection>;
  protected:

    using Serializer = s11n::RayTraceRadarSerializer;

    friend Serializer;

    explicit RayTraceRadarMeasurement(RawData &&data)
      : Super(0u, std::move(data)) {}

  public:

    Super::size_type GetDetectionAmount() const {
      return Super::size();
    }
  };

} // namespace data
} // namespace sensor
} // namespace carla
