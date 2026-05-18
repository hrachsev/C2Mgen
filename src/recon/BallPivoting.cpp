#include "BallPivoting.h"

#include "core/JobProgress.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/KdTree.h"

namespace recon {
namespace {

struct Edge {
  uint32_t a = 0;
  uint32_t b = 0;
  uint32_t opposite = 0;
  glm::vec3 ball_center{0.f};
};

inline uint64_t edgeKey(uint32_t a, uint32_t b) {
  if (a > b) std::swap(a, b);
  return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
}

inline uint64_t triKey(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t v[3] = {a, b, c};
  std::sort(v, v + 3);
  return (static_cast<uint64_t>(v[0]) * 73856093ull) ^
         (static_cast<uint64_t>(v[1]) * 19349663ull) ^
         (static_cast<uint64_t>(v[2]) * 83492791ull);
}

bool ballCenter(const glm::vec3& a,
                const glm::vec3& b,
                const glm::vec3& c,
                float rho,
                const glm::vec3& side_hint,
                glm::vec3* out_center) {
  const glm::vec3 ab = b - a;
  const glm::vec3 ac = c - a;
  const glm::vec3 n = glm::cross(ab, ac);
  const float n2 = glm::dot(n, n);
  if (n2 < 1e-30f) return false;

  const float ab2 = glm::dot(ab, ab);
  const float ac2 = glm::dot(ac, ac);
  const glm::vec3 num = glm::cross(ab2 * ac - ac2 * ab, n);
  const glm::vec3 cc = a + num / (2.f * n2);

  const float r2 = glm::dot(cc - a, cc - a);
  const float diff = rho * rho - r2;
  if (diff < 0.f) return false;

  const float h = std::sqrt(diff);
  const glm::vec3 nh = glm::normalize(n);
  const glm::vec3 c1 = cc + h * nh;
  const glm::vec3 c2 = cc - h * nh;

  const float d1 = glm::dot(c1 - cc, side_hint);
  *out_center = (d1 > 0.f) ? c1 : c2;
  return true;
}

bool ballEmpty(const KdTree& kd,
               const glm::vec3& c,
               float rho,
               const std::vector<size_t>& ignore) {
  std::vector<size_t> nei;
  std::vector<float> d2;
  kd.knnAtPosition(c, 32, &nei, &d2);
  const float thresh2 = (rho * 0.999f) * (rho * 0.999f);
  for (size_t i = 0; i < nei.size(); ++i) {
    if (d2[i] >= thresh2) continue;
    bool skip = false;
    for (size_t s : ignore) {
      if (nei[i] == s) {
        skip = true;
        break;
      }
    }
    if (!skip) return false;
  }
  return true;
}

bool findSeed(const PointCloud& cloud,
              const KdTree& kd,
              float rho,
              size_t seed_pt,
              const std::vector<bool>& removed,
              uint32_t* a,
              uint32_t* b,
              uint32_t* c,
              glm::vec3* ball) {
  std::vector<size_t> nei;
  std::vector<float> d2;
  kd.knnAtPosition(cloud.positions[seed_pt], 24, &nei, &d2);
  const float max_edge2 = (2.f * rho) * (2.f * rho);

  glm::vec3 hint(0.f);
  if (!cloud.normals.empty()) {
    for (size_t n : nei) {
      if (n < cloud.normals.size()) hint += cloud.normals[n];
    }
  }
  if (glm::length(hint) < 1e-12f) hint = glm::vec3(0.f, 1.f, 0.f);
  hint = glm::normalize(hint);

  for (size_t ji = 0; ji < nei.size(); ++ji) {
    const size_t j = nei[ji];
    if (j == seed_pt || removed[j]) continue;
    if (d2[ji] > max_edge2) continue;
    for (size_t ki = ji + 1; ki < nei.size(); ++ki) {
      const size_t k = nei[ki];
      if (k == seed_pt || k == j || removed[k]) continue;
      if (d2[ki] > max_edge2) continue;
      const glm::vec3& pa = cloud.positions[seed_pt];
      const glm::vec3& pb = cloud.positions[j];
      const glm::vec3& pc = cloud.positions[k];
      if (glm::dot(pb - pc, pb - pc) > max_edge2) continue;

      glm::vec3 cen;
      if (!ballCenter(pa, pb, pc, rho, hint, &cen)) continue;
      if (!ballEmpty(kd, cen, rho, {seed_pt, j, k})) continue;

      *a = static_cast<uint32_t>(seed_pt);
      *b = static_cast<uint32_t>(j);
      *c = static_cast<uint32_t>(k);
      *ball = cen;
      return true;
    }
  }
  return false;
}

bool pivotEdge(const PointCloud& cloud,
               const KdTree& kd,
               float rho,
               const Edge& e,
               const std::vector<bool>& removed,
               uint32_t* out_k,
               glm::vec3* out_center) {
  const glm::vec3& pa = cloud.positions[e.a];
  const glm::vec3& pb = cloud.positions[e.b];
  const glm::vec3& po = cloud.positions[e.opposite];
  const glm::vec3 mid = 0.5f * (pa + pb);

  std::vector<size_t> nei;
  std::vector<float> d2;
  kd.knnAtPosition(mid, 48, &nei, &d2);

  const glm::vec3 axis = glm::normalize(pb - pa);
  const glm::vec3 prev_dir = glm::normalize(e.ball_center - mid);

  glm::vec3 u = prev_dir -
                axis * glm::dot(axis, prev_dir);
  if (glm::length(u) < 1e-12f) {
    u = glm::abs(axis.x) < 0.9f
            ? glm::cross(axis, glm::vec3(1.f, 0.f, 0.f))
            : glm::cross(axis, glm::vec3(0.f, 1.f, 0.f));
  }
  u = glm::normalize(u);
  const glm::vec3 v = glm::normalize(glm::cross(axis, u));

  const float max_edge2 = (2.f * rho) * (2.f * rho);

  float best_angle = 1e30f;
  uint32_t best_k = 0;
  glm::vec3 best_center(0.f);
  bool found = false;

  glm::vec3 hint =
      glm::normalize((!cloud.normals.empty() &&
                      e.a < cloud.normals.size() &&
                      e.b < cloud.normals.size())
                         ? cloud.normals[e.a] + cloud.normals[e.b]
                         : glm::cross(pb - pa, po - pa));
  if (glm::length(hint) < 1e-12f) hint = glm::vec3(0.f, 1.f, 0.f);

  for (size_t k_idx : nei) {
    if (k_idx == e.a || k_idx == e.b || k_idx == e.opposite) continue;
    if (k_idx >= cloud.positions.size() || removed[k_idx]) continue;

    const glm::vec3& pk = cloud.positions[k_idx];
    if (glm::dot(pk - pa, pk - pa) > max_edge2) continue;
    if (glm::dot(pk - pb, pk - pb) > max_edge2) continue;

    glm::vec3 cen;
    if (!ballCenter(pa, pb, pk, rho, hint, &cen)) continue;

    const glm::vec3 dir = cen - mid;
    const glm::vec3 dir_proj = dir - axis * glm::dot(axis, dir);
    if (glm::length(dir_proj) < 1e-12f) continue;
    const float cu = glm::dot(dir_proj, u);
    const float cv = glm::dot(dir_proj, v);
    float ang = std::atan2(cv, cu);

    if (ang <= 1e-5f) ang += 2.f * 3.14159265358979323846f;
    if (ang >= best_angle) continue;
    if (!ballEmpty(kd, cen, rho, {e.a, e.b, k_idx})) continue;

    best_angle = ang;
    best_k = static_cast<uint32_t>(k_idx);
    best_center = cen;
    found = true;
  }

  if (found) {
    *out_k = best_k;
    *out_center = best_center;
  }
  return found;
}

}

TriangleMesh ballPivoting(const PointCloud& cloud, const BPAParams& params,
                          const JobProgress* progress) {
  TriangleMesh mesh;
  if (cloud.positions.size() < 3) return mesh;

  const float rho = std::max(1e-6f, params.ball_radius * params.radius_scale);
  KdTree kd(cloud.positions);

  std::vector<bool> removed(cloud.positions.size(), false);
  std::unordered_map<uint64_t, int> edge_count;
  std::unordered_set<uint64_t> tri_set;

  std::vector<int> remap(cloud.positions.size(), -1);
  auto vertOf = [&](uint32_t i) -> uint32_t {
    if (remap[i] < 0) {
      remap[i] = static_cast<int>(mesh.vertices.size());
      mesh.vertices.push_back(cloud.positions[i]);
      glm::vec3 n = (i < cloud.normals.size()) ? cloud.normals[i]
                                               : glm::vec3(0.f, 1.f, 0.f);
      const float nl = glm::length(n);
      mesh.normals.push_back(nl > 1e-20f ? n / nl : glm::vec3(0.f, 1.f, 0.f));
    }
    return static_cast<uint32_t>(remap[i]);
  };

  auto emitTriangle = [&](uint32_t a, uint32_t b, uint32_t c) -> bool {
    if (a == b || b == c || a == c) return false;
    if (!tri_set.insert(triKey(a, b, c)).second) return false;

    const glm::vec3& pa = cloud.positions[a];
    const glm::vec3& pb = cloud.positions[b];
    const glm::vec3& pc = cloud.positions[c];
    const float dab = glm::length(pb - pa);
    const float dbc = glm::length(pc - pb);
    const float dac = glm::length(pc - pa);
    const float area = 0.5f * glm::length(glm::cross(pb - pa, pc - pa));
    const float perimeter = dab + dbc + dac;
    const float aspect = (perimeter > 1e-20f) ? (4.f * 1.7320508f * area) /
                                                    (perimeter * perimeter)
                                              : 0.f;
    if (aspect < params.min_triangle_aspect) {
      tri_set.erase(triKey(a, b, c));
      return false;
    }

    if (params.use_normals && !cloud.normals.empty()) {
      glm::vec3 fn = glm::cross(pb - pa, pc - pa);
      if (glm::length(fn) > 1e-20f) {
        fn = glm::normalize(fn);
        const glm::vec3 avg =
            cloud.normals[a] + cloud.normals[b] + cloud.normals[c];
        if (glm::length(avg) > 1e-12f && glm::dot(fn, glm::normalize(avg)) < -0.5f) {
          tri_set.erase(triKey(a, b, c));
          return false;
        }
      }
    }

    const uint32_t va = vertOf(a);
    const uint32_t vb = vertOf(b);
    const uint32_t vc = vertOf(c);

    glm::vec3 fn = glm::cross(pb - pa, pc - pa);
    if (params.use_normals && !cloud.normals.empty()) {
      const glm::vec3 avg =
          cloud.normals[a] + cloud.normals[b] + cloud.normals[c];
      if (glm::length(fn) > 1e-20f && glm::length(avg) > 1e-12f &&
          glm::dot(fn, avg) < 0.f) {
        mesh.indices.emplace_back(va, vc, vb);
      } else {
        mesh.indices.emplace_back(va, vb, vc);
      }
    } else {
      mesh.indices.emplace_back(va, vb, vc);
    }
    return true;
  };

  std::deque<Edge> front;

  if (progress) progress->set(0.02f, "Ball pivoting: growing mesh");

  for (size_t seed = 0; seed < cloud.positions.size(); ++seed) {
    if (progress) {
      progress->setIndexed(seed, cloud.positions.size(), 0.05f, 0.93f,
                           "Ball pivoting: seeding components", 128);
    }
    if (removed[seed]) continue;

    uint32_t a = 0, b = 0, c = 0;
    glm::vec3 ball;
    if (!findSeed(cloud, kd, rho, seed, removed, &a, &b, &c, &ball)) continue;

    if (!emitTriangle(a, b, c)) continue;

    auto pushEdge = [&](uint32_t u, uint32_t w, uint32_t opp,
                        const glm::vec3& bc) {
      const uint64_t key = edgeKey(u, w);
      auto it = edge_count.find(key);
      if (it == edge_count.end()) {
        edge_count[key] = 1;
        front.push_back({u, w, opp, bc});
      } else {
        it->second += 1;
      }
    };
    pushEdge(a, b, c, ball);
    pushEdge(b, c, a, ball);
    pushEdge(c, a, b, ball);

    int safety = 0;
    const int max_iters =
        static_cast<int>(cloud.positions.size()) * 6 + 1024;
    while (!front.empty() && safety++ < max_iters) {
      Edge e = front.front();
      front.pop_front();
      const uint64_t k_e = edgeKey(e.a, e.b);
      auto it = edge_count.find(k_e);
      if (it == edge_count.end() || it->second >= 2) continue;

      uint32_t k = 0;
      glm::vec3 newCenter;
      if (!pivotEdge(cloud, kd, rho, e, removed, &k, &newCenter)) {

        edge_count[k_e] = 2;
        continue;
      }

      if (!emitTriangle(e.a, e.b, k)) {
        edge_count[k_e] = 2;
        continue;
      }

      edge_count[k_e] = 2;

      pushEdge(e.b, k, e.a, newCenter);
      pushEdge(k, e.a, e.b, newCenter);
    }
  }

  if (progress) progress->set(1.f, "Ball pivoting: complete");
  return mesh;
}

}
