# Vision Module

This module is intentionally detached from the current robot implementations.
It does not depend on OpenCV types, hardware devices, or any existing robot
state structure.

## Current scope

- Accept generic 2D observations from an upstream detector.
- Undistort pixels with a standard 5-parameter pinhole camera model.
- Convert a pixel into a camera-frame bearing ray.
- Estimate approximate camera-frame position from a 2D bounding box plus known
  object dimensions.

## What this module does not assume

- A specific object shape.
- A specific detector implementation.
- Any OpenCV runtime inside this project.

## Input contract

Map your upstream result into `basic::vision::Observation2D`.

- If you only have a center pixel, use `ObservationKind::kCenter`.
- If you have a YOLO box, use `ObservationKind::kBoundingBox`.
- If you later have corners or contour points, use `ObservationKind::kKeypoints`
  for now and extend the solver later.

Known object dimensions belong in `basic::vision::MetricDimensions`, not in the
detection result.

## Output contract

`MonocularLocator::estimate()` returns the highest-level estimate that is
currently solvable:

- `kBearingOnly` when only direction is recoverable.
- `kApproximatePosition` when a valid box and metric dimensions are available.

Future `PnP` support can be added without changing the upstream detector
contract.

## Typical usage

```cpp
#include "vision/locator.h"

basic::vision::CameraModel camera{
    820.0, 820.0, 320.0, 240.0,
    -0.12, 0.03, 0.0, 0.0, 0.0};

basic::vision::Observation2D observation;
observation.kind = basic::vision::ObservationKind::kBoundingBox;
observation.score = 0.91;
observation.bbox_px = {210.0, 140.0, 84.0, 96.0};

basic::vision::MetricDimensions target;
target.has_width = true;
target.width_mm = 120.0;
target.has_height = true;
target.height_mm = 140.0;

basic::vision::MonocularLocator locator;
const basic::vision::EstimateResult result =
    locator.estimate(camera, observation, target);
```

## Extension path

When you later decide the object morphology:

1. Keep `Observation2D` as the detector boundary.
2. Add a new solver for the richer observation type.
3. Preserve the current bearing and box-based fallback path.
