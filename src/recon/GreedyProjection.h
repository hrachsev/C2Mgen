#pragma once

#include "core/GeometryTypes.h"
#include "core/JobProgress.h"

struct GreedyParams;

namespace recon {

TriangleMesh greedyProjectionTriangulation(const PointCloud& cloud,
                                           const GreedyParams& params,
                                           const JobProgress* progress = nullptr);

}
