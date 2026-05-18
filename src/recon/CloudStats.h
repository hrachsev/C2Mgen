#pragma once

#include <cstddef>

#include <glm/glm.hpp>

#include "core/GeometryTypes.h"

struct CloudStats {
  size_t point_count = 0;
  glm::vec3 bbox_min{0.f};
  glm::vec3 bbox_max{0.f};
  float median_nn_distance = 1.f;
  float mean_nn_distance = 1.f;
  float nn_distance_std = 0.f;
  float normal_consistency = 0.f;
  float density_cv = 0.f;
};

namespace recon {

CloudStats computeStats(const PointCloud& cloud, int kNeighbors);

}
