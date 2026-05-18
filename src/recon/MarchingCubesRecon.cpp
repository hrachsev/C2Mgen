#include "MarchingCubesRecon.h"

#include "core/JobProgress.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/KdTree.h"
#include "core/MarchingCubes.h"

namespace recon {
namespace {

inline size_t Lin(int i, int j, int k, int sx, int sy) {
  return static_cast<size_t>(i) +
         static_cast<size_t>(j) * static_cast<size_t>(sx) +
         static_cast<size_t>(k) * static_cast<size_t>(sx) *
             static_cast<size_t>(sy);
}

}

TriangleMesh marchingCubesReconstruct(const PointCloud& cloud,
                                      const MarchingCubesParams& params,
                                      const JobProgress* progress) {
  TriangleMesh mesh;
  if (cloud.positions.empty()) return mesh;
  if (cloud.normals.size() != cloud.positions.size()) return mesh;

  if (progress) progress->set(0.f, "Marching cubes: building TSDF grid");

  glm::vec3 bmin = cloud.positions[0];
  glm::vec3 bmax = cloud.positions[0];
  for (const auto& p : cloud.positions) {
    bmin = glm::min(bmin, p);
    bmax = glm::max(bmax, p);
  }
  glm::vec3 ext = glm::max(bmax - bmin, glm::vec3(1e-3f));
  const glm::vec3 pad = ext * 0.10f;
  bmin -= pad;
  bmax += pad;
  ext = bmax - bmin;

  const int res = std::clamp(params.grid_resolution, 16, 512);
  const float longest = std::max({ext.x, ext.y, ext.z});
  const float h = longest / static_cast<float>(res);
  const int vx = std::max(4, static_cast<int>(std::ceil(ext.x / h)) + 1);
  const int vy = std::max(4, static_cast<int>(std::ceil(ext.y / h)) + 1);
  const int vz = std::max(4, static_cast<int>(std::ceil(ext.z / h)) + 1);
  const glm::vec3 cell(h, h, h);

  KdTree kd(cloud.positions);
  const int kNN = std::clamp(params.knn_for_distance, 1, 32);

  float avg_spacing = h;
  {
    std::vector<size_t> nei;
    std::vector<float> d2;
    const size_t step = std::max<size_t>(1, cloud.positions.size() / 256);
    double acc = 0.0;
    size_t cnt = 0;
    for (size_t i = 0; i < cloud.positions.size(); i += step) {
      kd.knn(i, 4, &nei, &d2);
      if (!d2.empty()) {
        acc += std::sqrt(d2.front());
        ++cnt;
      }
    }
    if (cnt > 0) avg_spacing = static_cast<float>(acc / static_cast<double>(cnt));
    avg_spacing = std::max(avg_spacing, 1e-6f * longest);
  }
  const float trunc =
      std::max(h * 1.5f, avg_spacing * std::max(1.f, params.truncation_voxels));

  const size_t total = static_cast<size_t>(vx) * static_cast<size_t>(vy) *
                       static_cast<size_t>(vz);
  std::vector<float> field(total, trunc);

  for (int k = 0; k < vz; ++k) {
    if (progress) {
      progress->setRange(0.05f, 0.82f, static_cast<float>(k) /
                                             static_cast<float>(std::max(1, vz)),
                         "Marching cubes: filling voxels");
    }
    for (int j = 0; j < vy; ++j) {
      for (int i = 0; i < vx; ++i) {
        const glm::vec3 X =
            bmin + cell * glm::vec3(static_cast<float>(i),
                                    static_cast<float>(j),
                                    static_cast<float>(k));
        std::vector<size_t> nei;
        std::vector<float> d2;
        kd.knnAtPosition(X, kNN, &nei, &d2);
        if (nei.empty()) continue;

        if (std::sqrt(d2.front()) > trunc) continue;

        double sum_w = 0.0;
        double sum_sd = 0.0;
        for (size_t t = 0; t < nei.size(); ++t) {
          const size_t pi = nei[t];
          const glm::vec3& p = cloud.positions[pi];
          glm::vec3 n = cloud.normals[pi];
          const float nl = glm::length(n);
          if (nl < 1e-20f) continue;
          n /= nl;
          const float sd = glm::dot(X - p, n);
          const float w = 1.f / (d2[t] + 1e-12f);
          sum_w += w;
          sum_sd += static_cast<double>(sd) * w;
        }
        if (sum_w > 1e-20) {
          float sd =
              static_cast<float>(sum_sd / sum_w);
          sd = std::clamp(sd, -trunc, trunc);
          field[Lin(i, j, k, vx, vy)] = sd;
        }
      }
    }
  }

  if (progress) progress->set(0.88f, "Marching cubes: extracting surface");

  std::vector<glm::vec3> pos;
  std::vector<glm::vec3> nrm;
  std::vector<glm::uvec3> idx;
  mc::extractIsoSurface(field, vx - 1, vy - 1, vz - 1, bmin, cell,
                        params.iso_level, &pos, &nrm, &idx);

  mesh.vertices = std::move(pos);
  mesh.normals = std::move(nrm);
  mesh.indices = std::move(idx);

  if (progress) progress->set(1.f, "Marching cubes: complete");
  return mesh;
}

}
