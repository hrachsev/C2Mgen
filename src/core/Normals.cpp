#include "Normals.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

#include "KdTree.h"

namespace normals {

namespace {

glm::mat3 covariance(size_t cnt,
                     const glm::vec3& mean,
                     const std::vector<glm::vec3>& pts,
                     const std::vector<size_t>& nei) {
  glm::mat3 C(0.f);
  for (size_t i : nei) {
    if (i >= pts.size() || cnt == 0) continue;
    glm::vec3 q = pts[i] - mean;
    C += glm::outerProduct(q, q);
  }
  if (cnt > 1) {
    C /= static_cast<float>(cnt);
  }
  return C;
}

glm::vec3 smallestEigenVectorSymmetric(glm::mat3 C) {

  const float eps = 1e-8f;
  C += eps * glm::mat3(1.f);
  glm::vec3 v(1.f, 0.f, 0.f);
  for (int it = 0; it < 24; ++it) {

    const float det = glm::determinant(C);
    if (std::abs(det) < 1e-20f) break;
    const glm::mat3 invC = glm::inverse(C);
    v = glm::normalize(invC * v);
    C *= 1.f;
  }
  return glm::normalize(v);
}

}

void estimateMissing(PointCloud* cloud,
                     const KdTree& tree,
                     int kNeighbors,
                     unsigned seed) {
  if (!cloud || cloud->positions.empty()) return;

  cloud->normals.resize(cloud->positions.size(), glm::vec3(0.f));

  std::vector<size_t> nei;
  std::vector<float> d2;
  nei.reserve(static_cast<size_t>(kNeighbors + 1));
  const int k_use = std::max(6, std::min(kNeighbors, static_cast<int>(cloud->positions.size()) - 1));

  std::mt19937 rng(seed);

  std::vector<size_t> order(cloud->positions.size());
  std::iota(order.begin(), order.end(), 0);
  std::shuffle(order.begin(), order.end(), rng);

  const auto& pts = cloud->positions;
  for (size_t oi : order) {
    const glm::vec3& p = pts[oi];
    tree.knnAtPosition(p, k_use + 1, &nei, &d2);

    glm::vec3 mean(0.f);
    size_t cnt = 0;
    for (size_t ix : nei) {
      if (ix >= pts.size()) continue;
      if (glm::dot(pts[ix] - p, pts[ix] - p) < 1e-24f) continue;
      mean += pts[ix];
      ++cnt;
    }
    if (cnt == 0) continue;
    mean /= static_cast<float>(cnt);

    glm::mat3 C = covariance(cnt, mean, pts, nei);
    glm::vec3 n = smallestEigenVectorSymmetric(C);
    if (!std::isfinite(n.x) || !std::isfinite(n.y) || !std::isfinite(n.z)) {
      n = glm::vec3(0.f, 1.f, 0.f);
    }
    cloud->normals[oi] = glm::normalize(n);
  }
}

void orientConsistentTowardCenter(PointCloud* cloud) {
  if (!cloud || cloud->positions.empty()) return;
  glm::vec3 bmin(std::numeric_limits<float>::infinity());
  glm::vec3 bmax(-std::numeric_limits<float>::infinity());
  for (const auto& p : cloud->positions) {
    bmin = glm::min(bmin, p);
    bmax = glm::max(bmax, p);
  }
  const glm::vec3 center = 0.5f * (bmin + bmax);

  std::vector<bool> touched(cloud->positions.size(), false);
  std::vector<size_t> stack;
  if (!cloud->normals.empty()) {
    stack.push_back(0);
    touched[0] = true;
    const glm::vec3 outward0 = glm::normalize(cloud->positions[0] - center);
    if (glm::dot(cloud->normals[0], outward0) < 0.f)
      cloud->normals[0] = -cloud->normals[0];
  }

  KdTree local_tree(cloud->positions);
  std::vector<size_t> nei;
  while (!stack.empty()) {
    const size_t i = stack.back();
    stack.pop_back();
    nei.clear();
    local_tree.knn(i, 8, &nei, nullptr);
    for (size_t j : nei) {
      if (j >= cloud->positions.size() || touched[j]) continue;
      touched[j] = true;
      const glm::vec3& ni = cloud->normals[i];
      glm::vec3& nj = cloud->normals[j];
      if (glm::dot(ni, nj) < 0.f) nj = -nj;
      stack.push_back(j);
    }
  }
}

}
