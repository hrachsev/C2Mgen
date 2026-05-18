#pragma once

#include <cstdint>

#include "GeometryTypes.h"

class KdTree;

namespace normals {

void estimateMissing(PointCloud* cloud,
                     const KdTree& tree,
                     int kNeighbors,
                     unsigned progressSeed = 12345u);

void orientConsistentTowardCenter(PointCloud* cloud);

}
