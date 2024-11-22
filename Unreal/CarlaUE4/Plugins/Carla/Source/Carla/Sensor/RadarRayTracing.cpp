#include <PxScene.h>

#include "Carla.h"

#include "Carla/Sensor/RadarRayTracing.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

#include "carla/geom/Math.h"

FActorDefinition ARayTraceRadar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeRadarRayTracingDefinition();
}

ARayTraceRadar::ARayTraceRadar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  TraceParams = FCollisionQueryParams(FName(TEXT("Radar_Trace")), true);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;
}

void ARayTraceRadar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetRadarRayTracing(ActorDescription, this);
}

void ARayTraceRadar::SetHorizontalFOV(float NewHorizontalFOV)
{
  HorizontalFOV = NewHorizontalFOV;
}

void ARayTraceRadar::SetVerticalFOV(float NewVerticalFOV)
{
  VerticalFOV = NewVerticalFOV;
}

void ARayTraceRadar::SetRange(float NewRange)
{
  Range = NewRange;
}

void ARayTraceRadar::SetPointsPerSecond(int NewPointsPerSecond)
{
  PointsPerSecond = NewPointsPerSecond;
  RadarData.SetResolution(PointsPerSecond);
}

void ARayTraceRadar::BeginPlay()
{
  Super::BeginPlay();
  PrevLocation = GetActorLocation();
}

void ARayTraceRadar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  CalculateCurrentVelocity(DeltaTime);

  RadarData.Reset();
  SendLineTraces(DeltaTime);

  auto DataStream = GetDataStream(*this);
  DataStream.Send(*this, RadarData, DataStream.PopBufferFromPool());
}

void ARayTraceRadar::CalculateCurrentVelocity(const float DeltaTime)
{
  const FVector RadarLocation = GetActorLocation();
  CurrentVelocity = (RadarLocation - PrevLocation) / DeltaTime;
  PrevLocation = RadarLocation;
}

void ARayTraceRadar::SendLineTraces(float DeltaTime)
{
  constexpr float TO_METERS = 1e-2f;
  const FTransform& ActorTransform = GetActorTransform();
  const FVector& RadarLocation = GetActorLocation();

  // Maximum radar radius in horizontal and vertical direction
  const float MaxRx = FMath::Tan(FMath::DegreesToRadians(HorizontalFOV * 0.5f)) * Range;
  const float MaxRy = FMath::Tan(FMath::DegreesToRadians(VerticalFOV * 0.5f)) * Range;
  const int32 NumPoints = static_cast<int32>(PointsPerSecond * DeltaTime);

  // Generate the parameters of the rays in a deterministic way
  Rays.Empty();
  Rays.SetNum(NumPoints);
  for (int32 i = 0; i < Rays.Num(); i++) {
    Rays[i].Radius = RandomEngine->GetUniformFloat();
    Rays[i].Angle = RandomEngine->GetUniformFloatInRange(0.0f, carla::geom::Math::Pi2<float>());
    Rays[i].Hitted = false;
    Rays[i].ReturnedToRadar = false;
    Rays[i].TotalDistance = 0.0f;
  }
  FCriticalSection Mutex;

  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    ParallelFor(NumPoints, [this, &RadarLocation, &ActorTransform, MaxRx, MaxRy](int32 idx) {
      const float Radius = Rays[idx].Radius;
      const float Angle  = Rays[idx].Angle;

      float SinAngle, CosAngle;
      FMath::SinCos(&SinAngle, &CosAngle, Angle);

      FVector BeamDirection = FVector(1.0f, MaxRx * Radius * CosAngle / Range, MaxRy * Radius * SinAngle / Range);
      BeamDirection = ActorTransform.TransformVectorNoScale(BeamDirection).GetSafeNormal();

      TraceRayRecursive(
        RadarLocation,
        BeamDirection,
        0,        // Initial bounce count
        0.0f,     // Initial distance traveled
        Rays[idx] // Pass the FRadarRay struct by reference
      );
    });
  }

  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  // Write the detections in the output structure
  for (auto& ray : Rays) {
    if (ray.ReturnedToRadar) {
      // Record the detection corresponding to the returned ray
      RadarData.WriteDetection({
        ray.RelativeVelocities.Last(),      // Relative velocity (should be zero)
        ray.AzimuthAndElevations.Last().X,  // Azimuth
        ray.AzimuthAndElevations.Last().Y,  // Elevation
        ray.TotalDistance                   // Total distance traveled
      });
    } else if (ray.Hitted) {
      // Process other hits as before
      for (int32 i = 0; i < ray.Hits.Num(); ++i) {
        RadarData.WriteDetection({
          ray.RelativeVelocities[i],
          ray.AzimuthAndElevations[i].X,
          ray.AzimuthAndElevations[i].Y,
          ray.Distances[i]
        });
      }
    }
  }
}

void ARayTraceRadar::TraceRayRecursive(
  const FVector& StartLocation,
  const FVector& Direction,
  int BounceCount,
  float CurrentDistance,
  FRadarRay& RayData)
{
  constexpr float TO_METERS = 1e-2f;

  if (BounceCount >= MaxBounces || CurrentDistance >= Range)
    return;

  FHitResult OutHit;
  FVector EndLocation = StartLocation + Direction * (Range - CurrentDistance);

  bool bHit = GetWorld()->LineTraceSingleByChannel(
    OutHit,
    StartLocation,
    EndLocation,
    ECC_GameTraceChannel2,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );

  if (bHit)
  {
    float HitDistance = (OutHit.ImpactPoint - StartLocation).Size();
    CurrentDistance += HitDistance;

    // Check if the ray has returned to the radar sensor
    if (OutHit.Actor.Get() == this)
    {
      // Ray has returned to the radar sensor
      RayData.ReturnedToRadar = true;
      RayData.TotalDistance = CurrentDistance * TO_METERS;

      // Calculate relative velocity (should be zero since it's the radar itself)
      float RelativeVelocity = 0.0f;
      RayData.RelativeVelocities.Add(RelativeVelocity);

      // Calculate azimuth and elevation
      FVector DirectionVector = OutHit.ImpactPoint - GetActorLocation();
      float Azimuth, Elevation;
      CalculateAzimuthAndElevation(DirectionVector, Azimuth, Elevation);
      RayData.AzimuthAndElevations.Add(FVector2D(Azimuth, Elevation));
      RayData.Distances.Add(CurrentDistance * TO_METERS);

      // No need to trace further
      return;
    }
    else
    {
      // Normal processing for other hits
      RayData.Hitted = true;
      RayData.Hits.Add(OutHit);

      // Calculate relative velocity
      float RelativeVelocity = CalculateRelativeVelocity(OutHit, StartLocation);
      RayData.RelativeVelocities.Add(RelativeVelocity);

      // Calculate azimuth and elevation
      FVector DirectionVector = OutHit.ImpactPoint - GetActorLocation();
      float Azimuth, Elevation;
      CalculateAzimuthAndElevation(DirectionVector, Azimuth, Elevation);
      RayData.AzimuthAndElevations.Add(FVector2D(Azimuth, Elevation));
      RayData.Distances.Add(CurrentDistance * TO_METERS);

      // Proceed with the next bounce
      FVector ReflectedDirection = FMath::GetReflectionVector(Direction, OutHit.Normal);
      TraceRayRecursive(
        OutHit.ImpactPoint + ReflectedDirection * KINDA_SMALL_NUMBER,
        ReflectedDirection,
        BounceCount + 1,
        CurrentDistance,
        RayData
      );
    }
  }
}

float ARayTraceRadar::CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& RadarLocation)
{
  constexpr float TO_METERS = 1e-2f;

  const AActor* HittedActor = OutHit.Actor.Get();
  const FVector TargetVelocity = HittedActor ? HittedActor->GetVelocity() : FVector::ZeroVector;
  const FVector TargetLocation = OutHit.ImpactPoint;
  const FVector Direction = (TargetLocation - RadarLocation).GetSafeNormal();
  const FVector DeltaVelocity = (TargetVelocity - CurrentVelocity);
  const float V = TO_METERS * FVector::DotProduct(DeltaVelocity, Direction);

  return V;
}

void ARayTraceRadar::CalculateAzimuthAndElevation(const FVector& Direction, float& OutAzimuth, float& OutElevation)
{
  const FVector LocalDirection = GetActorTransform().InverseTransformVectorNoScale(Direction).GetSafeNormal();
  OutAzimuth = FMath::Atan2(LocalDirection.Y, LocalDirection.X);
  OutElevation = FMath::Asin(LocalDirection.Z);
}
