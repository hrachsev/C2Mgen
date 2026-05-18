#include "CloudStats.h"

#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <numeric>
#include <vector>

#include "core/KdTree.h"

namespace recon {

CloudStats computeStats(const PointCloud& cloud, int kNeighbors) {
  CloudStats s;
  s.point_count = cloud.positions.size();
  if (cloud.positions.empty()) return s;

  s.bbox_min = cloud.positions[0];
  s.bbox_max = cloud.positions[0];
  for (const auto& p : cloud.positions) {
    s.bbox_min = glm::min(s.bbox_min, p);
    s.bbox_max = glm::max(s.bbox_max, p);
  }

  const int k = glm::clamp(
      std::max(8, std::min(kNeighbors, static_cast<int>(cloud.positions.size()) - 1)),
      8, std::max(8, static_cast<int>(cloud.positions.size()) - 1));

  KdTree tree(cloud.positions);

  std::vector<float> distances;
  std::vector<size_t> nei;

  distances.reserve(tree.points().size());
  for (size_t i = 0; i < tree.points().size(); ++i) {
    nei.clear();
    std::vector<float> d2;
    tree.knn(i, std::max(1, std::min(k, static_cast<int>(tree.points().size()) - 1)), &nei,
             &d2);
    float best = NAN;
    for (float sd : d2) {
      if (sd > 1e-20f && std::isfinite(sd)) {
        best = std::sqrt(sd);
        break;
      }
    }
    if (std::isfinite(best)) distances.push_back(best);
  }

  std::nth_element(distances.begin(), distances.begin() + distances.size() / 2,
                     distances.end());
  s.median_nn_distance =
      distances.empty() ? 1.f : distances[distances.size() / 2];

  float sum = std::accumulate(distances.begin(), distances.end(), 0.f);
  s.mean_nn_distance = distances.empty() ? s.median_nn_distance : sum / distances.size();
  float var = 0.f;
  for (float d : distances) {
    float x = d - s.mean_nn_distance;
    var += x * x;
  }
  s.nn_distance_std =
      distances.size() <= 1 ? 0.f : std::sqrt(var / float(distances.size() - 1));

  const float coef =
      distances.empty()
          ? 0.f
          : (s.mean_nn_distance > 1e-12f ? s.nn_distance_std / s.mean_nn_distance : 0.f);
  s.density_cv = std::isfinite(coef) ? coef : 0.f;

  if (!cloud.normals.empty() && cloud.normals.size() == cloud.positions.size()) {
    nei.clear();
    std::vector<float> dtmp;
    std::vector<std::pair<float, size_t>> scores;
    for (size_t i = 0; i < cloud.positions.size(); ++i) {
      tree.knnAtPosition(cloud.positions[i], 8, &nei, &dtmp);
      float agg = 0.f;
      int cnt = 0;
      glm::vec3 ni = glm::normalize(cloud.normals[i]);
      for (size_t j : nei) {
        if (j == i || j >= cloud.normals.size()) continue;
        glm::vec3 nj = glm::normalize(cloud.normals[j]);
        float c = glm::clamp(glm::dot(ni, nj), -1.f, 1.f);
        agg += glm::clamp((c + 1.f) * 0.5f, 0.f, 1.f);
        ++cnt;
      }
      if (cnt > 0) scores.emplace_back(agg / static_cast<float>(cnt), i);
    }
    float avg = 0.f;
    for (auto& pr : scores) avg += pr.first;
    s.normal_consistency = scores.empty() ? 0.5f : avg / scores.size();
  }

  return s;
}

}
