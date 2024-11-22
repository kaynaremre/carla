// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Memory.h"
#include "carla/sensor/RawData.h"
#include "carla/sensor/data/RayTraceRadarData.h"

#include <cstdint>
#include <cstring>

namespace carla {
namespace sensor {

  class SensorData;

namespace s11n {

  // ===========================================================================
  // -- RadarSerializer --------------------------------------------------------
  // ===========================================================================

  /// Serializes the data generated by RayTraceRadar sensors.
  class RayTraceRadarSerializer {
  public:

    template <typename Sensor>
    static Buffer Serialize(
        const Sensor &sensor,
        const data::RayTraceRadarData &measurement,
        Buffer &&output);

    static SharedPtr<SensorData> Deserialize(RawData &&data);
  };

  template <typename Sensor>
  inline Buffer RayTraceRadarSerializer::Serialize(
      const Sensor &,
      const data::RayTraceRadarData &measurement,
      Buffer &&output) {
    output.copy_from(measurement._detections);
    return std::move(output);
  }

} // namespace s11n
} // namespace sensor
} // namespace carla
