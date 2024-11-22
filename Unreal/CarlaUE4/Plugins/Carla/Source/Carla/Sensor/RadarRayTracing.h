// Copyright (c) 2019 Computer Vision Center (CVC)
// at the Universitat Autonoma de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/Sensor.h"
#include "Carla/Actor/ActorDefinition.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/RayTraceRadarData.h>
#include <compiler/enable-ue4-macros.h>

#include "RadarRayTracing.generated.h"

/// A ray-tracing based Radar sensor.
UCLASS()
class CARLA_API ARayTraceRadar : public ASensor
{
  GENERATED_BODY()

  using FRadarData = carla::sensor::data::RayTraceRadarData;

public:

  static FActorDefinition GetSensorDefinition();

  ARayTraceRadar(const FObjectInitializer &ObjectInitializer);

  void Set(const FActorDescription &Description) override;

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetHorizontalFOV(float NewHorizontalFOV);

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetVerticalFOV(float NewVerticalFOV);

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetRange(float NewRange);

  UFUNCTION(BlueprintCallable, Category = "Radar")
  void SetPointsPerSecond(int NewPointsPerSecond);

protected:

  void BeginPlay() override;

  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection")
  float Range;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection")
  float HorizontalFOV;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection")
  float VerticalFOV;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection")
  int PointsPerSecond;

private:

  /// Calculates the current velocity of the radar sensor
  void CalculateCurrentVelocity(const float DeltaTime);

  /// Sends out rays into the environment to detect objects
  void SendLineTraces(float DeltaTime);

  struct FRadarRay
  {
    float Radius;
    float Angle;
    bool Hitted;
    bool ReturnedToRadar; // Flag to indicate if the ray returned to the radar
    float TotalDistance;  // Total distance traveled when returning to radar
    TArray<FHitResult> Hits;
    TArray<float> Distances;
    TArray<float> RelativeVelocities;
    TArray<FVector2D> AzimuthAndElevations;

    FRadarRay()
      : Radius(0.0f),
        Angle(0.0f),
        Hitted(false),
        ReturnedToRadar(false),
        TotalDistance(0.0f)
    {}
  };

  /// Recursive function to trace rays with multiple bounces
  void TraceRayRecursive(
    const FVector& StartLocation,
    const FVector& Direction,
    int BounceCount,
    float CurrentDistance,
    FRadarRay& RayData);
  /// Calculates the relative velocity between the radar and a hit object
  float CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& RadarLocation);
  void CalculateAzimuthAndElevation(
    const FVector& Direction,
    float& OutAzimuth,
    float& OutElevation);
  /// Holds the radar detections to be sent to the client
  FRadarData RadarData;

  /// Collision query parameters used for line traces
  FCollisionQueryParams TraceParams;

  /// Current velocity of the radar sensor
  FVector CurrentVelocity;

  /// Previous location of the radar sensor
  FVector PrevLocation;

  /// Structure to hold information about each ray and its bounces


  /// Array of rays used in the radar scan
  TArray<FRadarRay> Rays;

  /// Maximum number of bounces for recursive ray tracing
  private:
    static constexpr int32 MaxBounces = 3;
};
