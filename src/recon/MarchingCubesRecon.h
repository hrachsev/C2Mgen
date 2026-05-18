#pragma once

#include "core/GeometryTypes.h"
#include "core/JobProgress.h"

struct MarchingCubesParams;

namespace recon {

TriangleMesh marchingCubesReconstruct(const PointCloud& cloud,
                                      const MarchingCubesParams& params,
                                      const JobProgress* progress = nullptr);

}
