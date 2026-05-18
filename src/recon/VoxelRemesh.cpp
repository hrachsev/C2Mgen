#include "VoxelRemesh.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <vector>

#include <glm/glm.hpp>

#include "core/JobProgress.h"
#include "core/MarchingCubes.h"

namespace recon {
namespace {

inline size_t LinCell(int x, int y, int z, int nx, int ny) {
  return static_cast<size_t>(x) +
         static_cast<size_t>(nx) *
             (static_cast<size_t>(y) +
              static_cast<size_t>(ny) * static_cast<size_t>(z));
}

inline size_t LinCorner(int i, int j, int k, int nx1, int ny1) {
  return static_cast<size_t>(i) +
         static_cast<size_t>(nx1) *
             (static_cast<size_t>(j) +
              static_cast<size_t>(ny1) * static_cast<size_t>(k));
}

bool triAABBOverlap(const glm::vec3& box_min,
                    const glm::vec3& box_max,
                    const glm::vec3& t0,
                    const glm::vec3& t1,
                    const glm::vec3& t2) {
  const glm::vec3 c = 0.5f * (box_min + box_max);
  const glm::vec3 h = 0.5f * (box_max - box_min);
  const glm::vec3 v0 = t0 - c;
  const glm::vec3 v1 = t1 - c;
  const glm::vec3 v2 = t2 - c;

  const glm::vec3 tri_min = glm::min(v0, glm::min(v1, v2));
  const glm::vec3 tri_max = glm::max(v0, glm::max(v1, v2));
  if (tri_max.x < -h.x || tri_min.x > h.x) return false;
  if (tri_max.y < -h.y || tri_min.y > h.y) return false;
  if (tri_max.z < -h.z || tri_min.z > h.z) return false;

  auto axis_overlap = [&](const glm::vec3& axis_in) -> bool {
    glm::vec3 axis = axis_in;
    const float al = glm::length(axis);
    if (al < 1e-24f) return true;
    axis /= al;
    const float p0 = glm::dot(v0, axis);
    const float p1 = glm::dot(v1, axis);
    const float p2 = glm::dot(v2, axis);
    const float tmin = std::min({p0, p1, p2});
    const float tmax = std::max({p0, p1, p2});
    const float r = h.x * std::abs(axis.x) + h.y * std::abs(axis.y) +
                    h.z * std::abs(axis.z);
    return !(tmax < -r || tmin > r);
  };

  const glm::vec3 e0 = v1 - v0;
  const glm::vec3 e1 = v2 - v1;
  const glm::vec3 e2 = v0 - v2;
  glm::vec3 n = glm::cross(e0, v2 - v0);
  if (!axis_overlap(n)) return false;

  const glm::vec3 bx(1.f, 0.f, 0.f);
  const glm::vec3 by(0.f, 1.f, 0.f);
  const glm::vec3 bz(0.f, 0.f, 1.f);
  const glm::vec3 edges[3] = {e0, e1, e2};
  const glm::vec3 bases[3] = {bx, by, bz};
  for (const glm::vec3& e : edges) {
    for (const glm::vec3& b : bases) {
      if (!axis_overlap(glm::cross(e, b))) return false;
    }
  }
  return true;
}

void dilate26(const std::vector<uint8_t>& src,
              int nx,
              int ny,
              int nz,
              std::vector<uint8_t>* dst) {
  dst->assign(src.size(), 0);
  for (int z = 0; z < nz; ++z) {
    for (int y = 0; y < ny; ++y) {
      for (int x = 0; x < nx; ++x) {
        const size_t id = LinCell(x, y, z, nx, ny);
        if (!src[id]) continue;
        (*dst)[id] = 1;
        for (int dz = -1; dz <= 1; ++dz) {
          const int zz = z + dz;
          if (zz < 0 || zz >= nz) continue;
          for (int dy = -1; dy <= 1; ++dy) {
            const int yy = y + dy;
            if (yy < 0 || yy >= ny) continue;
            for (int dx = -1; dx <= 1; ++dx) {
              const int xx = x + dx;
              if (xx < 0 || xx >= nx) continue;
              (*dst)[LinCell(xx, yy, zz, nx, ny)] = 1;
            }
          }
        }
      }
    }
  }
}

void floodExteriorAir(const std::vector<uint8_t>& barrier,
                      int nx,
                      int ny,
                      int nz,
                      std::vector<uint8_t>* outside) {
  outside->assign(barrier.size(), 0);
  std::queue<uint32_t> q;

  auto try_seed = [&](int x, int y, int z) {
    const size_t id = LinCell(x, y, z, nx, ny);
    if (barrier[id]) return;
    if ((*outside)[id]) return;
    (*outside)[id] = 1;
    q.push(static_cast<uint32_t>(id));
  };

  for (int y = 0; y < ny; ++y) {
    for (int z = 0; z < nz; ++z) {
      try_seed(0, y, z);
      try_seed(nx - 1, y, z);
    }
  }
  for (int x = 0; x < nx; ++x) {
    for (int z = 0; z < nz; ++z) {
      try_seed(x, 0, z);
      try_seed(x, ny - 1, z);
    }
  }
  for (int x = 0; x < nx; ++x) {
    for (int y = 0; y < ny; ++y) {
      try_seed(x, y, 0);
      try_seed(x, y, nz - 1);
    }
  }

  const int dx[6] = {1, -1, 0, 0, 0, 0};
  const int dy[6] = {0, 0, 1, -1, 0, 0};
  const int dz[6] = {0, 0, 0, 0, 1, -1};

  while (!q.empty()) {
    const uint32_t cur = q.front();
    q.pop();
    const int z = static_cast<int>(cur / static_cast<size_t>(nx * ny));
    const size_t tmp = cur % static_cast<size_t>(nx * ny);
    const int y = static_cast<int>(tmp / static_cast<size_t>(nx));
    const int x = static_cast<int>(tmp % static_cast<size_t>(nx));

    for (int e = 0; e < 6; ++e) {
      const int nx2 = x + dx[e];
      const int ny2 = y + dy[e];
      const int nz2 = z + dz[e];
      if (nx2 < 0 || nx2 >= nx || ny2 < 0 || ny2 >= ny || nz2 < 0 ||
          nz2 >= nz)
        continue;
      const size_t nid = LinCell(nx2, ny2, nz2, nx, ny);
      if (barrier[nid]) continue;
      if ((*outside)[nid]) continue;
      (*outside)[nid] = 1;
      q.push(static_cast<uint32_t>(nid));
    }
  }
}

float cornerPhi(const std::vector<uint8_t>& solid,
                int dims_x,
                int dims_y,
                int dims_z,
                int ci,
                int cj,
                int ck) {
  int solid_count = 0;
  int total = 0;
  for (int ox = -1; ox <= 0; ++ox) {
    for (int oy = -1; oy <= 0; ++oy) {
      for (int oz = -1; oz <= 0; ++oz) {
        const int cx = ci + ox;
        const int cy = cj + oy;
        const int cz = ck + oz;
        if (cx < 0 || cy < 0 || cz < 0 || cx >= dims_x || cy >= dims_y ||
            cz >= dims_z)
          continue;
        ++total;
        if (solid[LinCell(cx, cy, cz, dims_x, dims_y)]) ++solid_count;
      }
    }
  }
  if (total <= 0) return 1.f;
  const float frac = static_cast<float>(solid_count) / static_cast<float>(total);
  return 0.5f - frac;
}

}

TriangleMesh voxelRemeshMesh(const TriangleMesh& mesh,
                             const VoxelRemeshParams& params_in,
                             const JobProgress* progress) {
  TriangleMesh out;
  if (mesh.vertices.empty() || mesh.indices.empty()) return out;

  VoxelRemeshParams params = params_in;
  params.voxel_size =
      std::max(params.voxel_size, static_cast<float>(1e-12));
  params.hole_close_passes =
      std::clamp(params.hole_close_passes, 0, 64);
  params.bbox_padding_voxels =
      std::max(0.f, params.bbox_padding_voxels);
  const int max_dim =
      std::max(8, params.max_grid_resolution_per_axis);

  glm::vec3 bmin = mesh.vertices[0];
  glm::vec3 bmax = mesh.vertices[0];
  for (const glm::vec3& v : mesh.vertices) {
    bmin = glm::min(bmin, v);
    bmax = glm::max(bmax, v);
  }
  glm::vec3 ext = glm::max(bmax - bmin, glm::vec3(1e-6f));
  const float pad_vox =
      params.bbox_padding_voxels * params.voxel_size;
  bmin -= glm::vec3(pad_vox);
  bmax += glm::vec3(pad_vox);
  ext = bmax - bmin;

  float h = params.voxel_size;
  auto dims_for_h = [&]() -> glm::ivec3 {
    return glm::ivec3(
        std::max(1, static_cast<int>(std::ceil(ext.x / h))),
        std::max(1, static_cast<int>(std::ceil(ext.y / h))),
        std::max(1, static_cast<int>(std::ceil(ext.z / h))));
  };
  glm::ivec3 dims = dims_for_h();
  while (dims.x > max_dim || dims.y > max_dim || dims.z > max_dim) {
    h *= 1.0625f;
    dims = dims_for_h();
  }

  const int nx = dims.x;
  const int ny = dims.y;
  const int nz = dims.z;

  if (progress) progress->set(0.f, "Voxel remesh: voxelizing mesh");

  std::vector<uint8_t> shell(static_cast<size_t>(nx) *
                                 static_cast<size_t>(ny) *
                                 static_cast<size_t>(nz),
                             0);
  std::vector<uint8_t> tmp;

  const glm::vec3 origin = bmin;
  const glm::vec3 cell(h, h, h);

  size_t tri_index = 0;
  for (const glm::uvec3& tri : mesh.indices) {
    ++tri_index;
    if (progress && (tri_index & 1023u) == 0u) {
      const float u =
          static_cast<float>(tri_index) /
          static_cast<float>(std::max<size_t>(1, mesh.indices.size()));
      progress->setRange(0.f, 0.35f, u, "Voxel remesh: voxelizing mesh");
    }
    const unsigned int ia = tri[0];
    const unsigned int ib = tri[1];
    const unsigned int ic = tri[2];
    if (ia >= mesh.vertices.size() || ib >= mesh.vertices.size() ||
        ic >= mesh.vertices.size())
      continue;

    const glm::vec3& va = mesh.vertices[ia];
    const glm::vec3& vb = mesh.vertices[ib];
    const glm::vec3& vc = mesh.vertices[ic];

    if (glm::length(glm::cross(vb - va, vc - va)) < 1e-20f) continue;

    glm::vec3 tri_min = glm::min(va, glm::min(vb, vc));
    glm::vec3 tri_max = glm::max(va, glm::max(vb, vc));
    tri_min -= glm::vec3(h * 1.001f);
    tri_max += glm::vec3(h * 1.001f);

    glm::vec3 q0 = (tri_min - origin) / cell;
    glm::vec3 q1 = (tri_max - origin) / cell;
    int ix0 = static_cast<int>(std::floor(q0.x));
    int iy0 = static_cast<int>(std::floor(q0.y));
    int iz0 = static_cast<int>(std::floor(q0.z));
    int ix1 = static_cast<int>(std::ceil(q1.x));
    int iy1 = static_cast<int>(std::ceil(q1.y));
    int iz1 = static_cast<int>(std::ceil(q1.z));

    ix0 = std::clamp(ix0, 0, nx - 1);
    iy0 = std::clamp(iy0, 0, ny - 1);
    iz0 = std::clamp(iz0, 0, nz - 1);
    ix1 = std::clamp(ix1, 0, nx - 1);
    iy1 = std::clamp(iy1, 0, ny - 1);
    iz1 = std::clamp(iz1, 0, nz - 1);

    for (int iz = iz0; iz <= iz1; ++iz) {
      for (int iy = iy0; iy <= iy1; ++iy) {
        for (int ix = ix0; ix <= ix1; ++ix) {
          const glm::vec3 cmn = origin + cell * glm::vec3(
                                            static_cast<float>(ix),
                                            static_cast<float>(iy),
                                            static_cast<float>(iz));
          const glm::vec3 cmx = cmn + cell;
          if (triAABBOverlap(cmn, cmx, va, vb, vc)) {
            shell[LinCell(ix, iy, iz, nx, ny)] = 1;
          }
        }
      }
    }
  }

  if (progress) progress->set(0.38f, "Voxel remesh: closing small gaps");
  std::vector<uint8_t> barrier = shell;
  for (int p = 0; p < params.hole_close_passes; ++p) {
    dilate26(barrier, nx, ny, nz, &tmp);
    barrier.swap(tmp);
    if (progress) {
      const float u =
          static_cast<float>(p + 1) /
          static_cast<float>(std::max(1, params.hole_close_passes));
      progress->setRange(0.38f, 0.52f, u,
                           "Voxel remesh: closing small gaps");
    }
  }

  if (progress) progress->set(0.55f, "Voxel remesh: labeling exterior air");
  std::vector<uint8_t> outside;
  floodExteriorAir(barrier, nx, ny, nz, &outside);

  if (progress) progress->set(0.65f, "Voxel remesh: building solid volume");
  std::vector<uint8_t> solid(shell.size(), 0);
  for (size_t i = 0; i < solid.size(); ++i) {
    if (!outside[i]) solid[i] = 1;
  }

  const int nx1 = nx + 1;
  const int ny1 = ny + 1;
  const int nz1 = nz + 1;
  std::vector<float> field(static_cast<size_t>(nx1) *
                               static_cast<size_t>(ny1) *
                               static_cast<size_t>(nz1),
                           1.f);

  if (progress) progress->set(0.72f, "Voxel remesh: implicit corners");
  for (int ck = 0; ck < nz1; ++ck) {
    if (progress && (ck & 15) == 0) {
      const float u = static_cast<float>(ck) / static_cast<float>(nz1);
      progress->setRange(0.72f, 0.85f, u,
                         "Voxel remesh: implicit corners");
    }
    for (int cj = 0; cj < ny1; ++cj) {
      for (int ci = 0; ci < nx1; ++ci) {
        field[LinCorner(ci, cj, ck, nx1, ny1)] =
            cornerPhi(solid, nx, ny, nz, ci, cj, ck);
      }
    }
  }

  if (progress) progress->set(0.88f, "Voxel remesh: marching cubes");

  std::vector<glm::vec3> pos;
  std::vector<glm::vec3> nrm;
  std::vector<glm::uvec3> idx;
  mc::extractIsoSurface(field, nx, ny, nz, origin, cell, 0.f, &pos, &nrm,
                        &idx);

  out.vertices = std::move(pos);
  out.normals = std::move(nrm);
  out.indices = std::move(idx);

  if (progress) progress->set(1.f, "Voxel remesh: complete");
  return out;
}

}
