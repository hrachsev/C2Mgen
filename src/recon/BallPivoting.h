#pragma once

#include "core/GeometryTypes.h"
#include "core/JobProgress.h"

struct BPAParams;

namespace recon {

TriangleMesh ballPivoting(const PointCloud& cloud, const BPAParams& params,
                          const JobProgress* progress = nullptr);

}
