#pragma once

#include "core/GeometryTypes.h"
#include "core/JobProgress.h"

struct PoissonParams;

namespace recon {

TriangleMesh poissonLikeReconstruct(const PointCloud& cloud,
                                  const PoissonParams& params,
                                  const JobProgress* progress = nullptr);

}
