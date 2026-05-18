#include "PoissonRecon.h"

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

void splatSample(const glm::vec3& p,
                 const glm::vec3& n,
                 const glm::vec3& origin,
                 const glm::vec3& cell,
                 int vx,
                 int vy,
                 int vz,
                 std::vector<float>* Vx,
                 std::vector<float>* Vy,
                 std::vector<float>* Vz,
                 std::vector<float>* W) {
  const glm::vec3 q = (p - origin) / cell;
  const int i0 = static_cast<int>(std::floor(q.x));
  const int j0 = static_cast<int>(std::floor(q.y));
  const int k0 = static_cast<int>(std::floor(q.z));
  const float fx = q.x - static_cast<float>(i0);
  const float fy = q.y - static_cast<float>(j0);
  const float fz = q.z - static_cast<float>(k0);

  for (int dk = 0; dk <= 1; ++dk) {
    const int kk = k0 + dk;
    if (kk < 0 || kk >= vz) continue;
    const float wk = (dk == 0 ? (1.f - fz) : fz);
    for (int dj = 0; dj <= 1; ++dj) {
      const int jj = j0 + dj;
      if (jj < 0 || jj >= vy) continue;
      const float wj = (dj == 0 ? (1.f - fy) : fy);
      for (int di = 0; di <= 1; ++di) {
        const int ii = i0 + di;
        if (ii < 0 || ii >= vx) continue;
        const float wi = (di == 0 ? (1.f - fx) : fx);
        const float w = wi * wj * wk;
        const size_t idx = Lin(ii, jj, kk, vx, vy);
        (*Vx)[idx] += w * n.x;
        (*Vy)[idx] += w * n.y;
        (*Vz)[idx] += w * n.z;
        (*W)[idx] += w;
      }
    }
  }
}

void divergence(const std::vector<float>& Vx,
                const std::vector<float>& Vy,
                const std::vector<float>& Vz,
                int vx,
                int vy,
                int vz,
                const glm::vec3& cell,
                std::vector<float>* out_b) {
  out_b->assign(Vx.size(), 0.f);
  const float invDx = 1.f / (2.f * cell.x);
  const float invDy = 1.f / (2.f * cell.y);
  const float invDz = 1.f / (2.f * cell.z);

  for (int k = 0; k < vz; ++k) {
    for (int j = 0; j < vy; ++j) {
      for (int i = 0; i < vx; ++i) {
        const int ip = std::min(i + 1, vx - 1);
        const int im = std::max(i - 1, 0);
        const int jp = std::min(j + 1, vy - 1);
        const int jm = std::max(j - 1, 0);
        const int kp = std::min(k + 1, vz - 1);
        const int km = std::max(k - 1, 0);
        const float dvx =
            (Vx[Lin(ip, j, k, vx, vy)] - Vx[Lin(im, j, k, vx, vy)]) * invDx;
        const float dvy =
            (Vy[Lin(i, jp, k, vx, vy)] - Vy[Lin(i, jm, k, vx, vy)]) * invDy;
        const float dvz =
            (Vz[Lin(i, j, kp, vx, vy)] - Vz[Lin(i, j, km, vx, vy)]) * invDz;
        (*out_b)[Lin(i, j, k, vx, vy)] = dvx + dvy + dvz;
      }
    }
  }
}

void gaussSeidel(std::vector<float>* chi,
                 const std::vector<float>& b,
                 int vx,
                 int vy,
                 int vz,
                 const glm::vec3& cell,
                 float lambda,
                 int iters,
                 const JobProgress* progress) {
  if (vx < 3 || vy < 3 || vz < 3) return;
  const float h2 = cell.x * cell.y;
  const float inv_h2 = 1.f / h2;
  const float diag = -6.f * inv_h2 - lambda;
  if (std::abs(diag) < 1e-20f) return;

  for (int it = 0; it < iters; ++it) {
    if (progress) {
      progress->setIndexed(static_cast<size_t>(it), static_cast<size_t>(iters),
                           0.25f, 0.60f, "Poisson: solving", 1);
    }
    for (int parity = 0; parity < 2; ++parity) {
      for (int k = 1; k < vz - 1; ++k) {
        for (int j = 1; j < vy - 1; ++j) {
          const int start = ((j + k) & 1) ^ parity;
          for (int i = 1 + start; i < vx - 1; i += 2) {
            const float sum =
                (*chi)[Lin(i - 1, j, k, vx, vy)] +
                (*chi)[Lin(i + 1, j, k, vx, vy)] +
                (*chi)[Lin(i, j - 1, k, vx, vy)] +
                (*chi)[Lin(i, j + 1, k, vx, vy)] +
                (*chi)[Lin(i, j, k - 1, vx, vy)] +
                (*chi)[Lin(i, j, k + 1, vx, vy)];
            const size_t c = Lin(i, j, k, vx, vy);
            (*chi)[c] = (b[c] - sum * inv_h2) / diag;
          }
        }
      }
    }
  }
}

}

TriangleMesh poissonLikeReconstruct(const PointCloud& cloud,
                                    const PoissonParams& params,
                                    const JobProgress* progress) {
  TriangleMesh mesh;
  if (cloud.positions.empty()) return mesh;
  if (cloud.normals.size() != cloud.positions.size()) return mesh;

  if (progress) progress->set(0.f, "Poisson: building grid");

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

  const int depth = std::clamp(params.octree_depth, 4, 10);
  const float longest = std::max({ext.x, ext.y, ext.z});
  const float h = longest / static_cast<float>(1 << depth);
  const int vx = std::max(8, static_cast<int>(std::ceil(ext.x / h)) + 1);
  const int vy = std::max(8, static_cast<int>(std::ceil(ext.y / h)) + 1);
  const int vz = std::max(8, static_cast<int>(std::ceil(ext.z / h)) + 1);
  const glm::vec3 cell(h, h, h);

  const size_t total = static_cast<size_t>(vx) * static_cast<size_t>(vy) *
                       static_cast<size_t>(vz);
  std::vector<float> Vx(total, 0.f);
  std::vector<float> Vy(total, 0.f);
  std::vector<float> Vz(total, 0.f);
  std::vector<float> W(total, 0.f);

  for (size_t s = 0; s < cloud.positions.size(); ++s) {
    if (progress) {
      progress->setIndexed(s, cloud.positions.size(), 0.05f, 0.18f,
                           "Poisson: splatting normals", 256);
    }
    glm::vec3 n = cloud.normals[s];
    const float nl = glm::length(n);
    if (nl < 1e-20f) continue;
    n /= nl;
    splatSample(cloud.positions[s], n, bmin, cell, vx, vy, vz, &Vx, &Vy, &Vz,
                &W);
  }

  if (progress) progress->set(0.20f, "Poisson: building equations");

  std::vector<float> rhs;
  divergence(Vx, Vy, Vz, vx, vy, vz, cell, &rhs);

  std::vector<float> chi(total, 0.f);
  const float lambda =
      std::max(0.f, params.screened_weight) * 4.f / (h * h);
  const int iters = std::clamp(params.solver_iterations, 8, 1000);
  gaussSeidel(&chi, rhs, vx, vy, vz, cell, lambda, iters, progress);

  if (progress) progress->set(0.88f, "Poisson: choosing iso-value");

  double iso_acc = 0.0;
  size_t iso_n = 0;
  for (const auto& p : cloud.positions) {
    const glm::vec3 q = (p - bmin) / cell;
    const int i = std::clamp(static_cast<int>(std::round(q.x)), 0, vx - 1);
    const int j = std::clamp(static_cast<int>(std::round(q.y)), 0, vy - 1);
    const int k = std::clamp(static_cast<int>(std::round(q.z)), 0, vz - 1);
    iso_acc += chi[Lin(i, j, k, vx, vy)];
    ++iso_n;
  }
  const float iso = (iso_n > 0)
                        ? static_cast<float>(iso_acc / static_cast<double>(iso_n))
                        : 0.f;

  if (progress) progress->set(0.92f, "Poisson: marching cubes");

  std::vector<glm::vec3> pos;
  std::vector<glm::vec3> nrm;
  std::vector<glm::uvec3> idx;
  mc::extractIsoSurface(chi, vx - 1, vy - 1, vz - 1, bmin, cell,
                        iso + params.iso_level, &pos, &nrm, &idx);

  mesh.vertices = std::move(pos);
  mesh.normals = std::move(nrm);
  mesh.indices = std::move(idx);

  if (progress) progress->set(1.f, "Poisson: complete");
  return mesh;
}

}
