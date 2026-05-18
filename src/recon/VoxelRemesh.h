#pragma once

#include "core/GeometryTypes.h"

struct JobProgress;

namespace recon {

TriangleMesh voxelRemeshMesh(const TriangleMesh& mesh,
                             const VoxelRemeshParams& params,
                             const JobProgress* progress);

}
