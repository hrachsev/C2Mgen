#include "GreedyProjection.h"

#include "core/JobProgress.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/KdTree.h"

namespace recon {
namespace {

constexpr float kPi = 3.14159265358979323846f;

enum class Status : uint8_t {
  Free = 0,
  Fringe = 1,
  Boundary = 2,
  Completed = 3,
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

struct FringeEdge {
  uint32_t a = 0;
  uint32_t b = 0;
};

void tangentFrame(const glm::vec3& n, glm::vec3* u, glm::vec3* v) {
  glm::vec3 nn = n;
  const float nl = glm::length(nn);
  if (nl < 1e-20f) {
    *u = glm::vec3(1.f, 0.f, 0.f);
    *v = glm::vec3(0.f, 1.f, 0.f);
    return;
  }
  nn /= nl;
  glm::vec3 ref = std::abs(nn.x) < 0.9f ? glm::vec3(1.f, 0.f, 0.f)
                                        : glm::vec3(0.f, 1.f, 0.f);
  *u = glm::normalize(glm::cross(nn, ref));
  *v = glm::normalize(glm::cross(nn, *u));
}

float medianNNSpacing(const KdTree& kd) {
  std::vector<float> dists;
  const auto& pts = kd.points();
  if (pts.empty()) return 1e-3f;
  const size_t step = std::max<size_t>(1, pts.size() / 1024);
  std::vector<size_t> nei;
  std::vector<float> d2;
  for (size_t i = 0; i < pts.size(); i += step) {
    kd.knn(i, 1, &nei, &d2);
    if (!d2.empty()) dists.push_back(std::sqrt(d2.front()));
  }
  if (dists.empty()) return 1e-3f;
  std::nth_element(dists.begin(), dists.begin() + dists.size() / 2,
                     dists.end());
  return dists[dists.size() / 2];
}

}

TriangleMesh greedyProjectionTriangulation(const PointCloud& cloud,
                                           const GreedyParams& params,
                                           const JobProgress* progress) {
  TriangleMesh mesh;
  if (cloud.positions.size() < 3) return mesh;
  if (cloud.normals.size() != cloud.positions.size()) return mesh;

  const size_t N = cloud.positions.size();

  if (progress) progress->set(0.02f, "Greedy projection: initializing");
  KdTree kd(cloud.positions);

  const float spacing = std::max(1e-9f, medianNNSpacing(kd));
  const float radius = std::max(spacing * 1.5f,
                                spacing * params.search_radius_mult);
  const float radius2 = radius * radius;
  const float cos_max_surface =
      std::cos(glm::radians(std::clamp(params.max_surface_angle_deg, 1.f, 89.f)));
  const float min_angle_rad =
      glm::radians(std::clamp(params.min_angle_deg, 1.f, 60.f));
  const float max_angle_rad =
      glm::radians(std::clamp(params.max_angle_deg, 60.f, 175.f));
  const int kmax = std::clamp(params.max_nearest_neighbors, 8, 256);

  std::vector<Status> status(N, Status::Free);
  std::unordered_map<uint64_t, int> edge_count;
  std::unordered_set<uint64_t> tri_set;
  std::vector<int> remap(N, -1);

  auto vertOf = [&](uint32_t i) -> uint32_t {
    if (remap[i] < 0) {
      remap[i] = static_cast<int>(mesh.vertices.size());
      mesh.vertices.push_back(cloud.positions[i]);
      mesh.normals.push_back(cloud.normals[i]);
    }
    return static_cast<uint32_t>(remap[i]);
  };

  auto canAccept = [&](uint32_t i, const glm::vec3& seed_n) -> bool {
    if (status[i] == Status::Completed || status[i] == Status::Boundary)
      return false;
    const glm::vec3 ni = cloud.normals[i];
    const float nl = glm::length(ni);
    if (nl < 1e-20f) return false;
    const float c = glm::dot(ni / nl, seed_n);
    return c >= cos_max_surface;
  };

  std::queue<FringeEdge> fringe;

  auto pushEdge = [&](uint32_t a, uint32_t b) {
    if (a == b) return;
    const uint64_t k = edgeKey(a, b);
    auto it = edge_count.find(k);
    if (it == edge_count.end()) {
      edge_count[k] = 1;
      fringe.push({a, b});
    } else {
      it->second += 1;
    }
  };

  auto emitTriangle = [&](uint32_t a, uint32_t b, uint32_t c) -> bool {
    if (a == b || b == c || a == c) return false;
    const uint64_t tk = triKey(a, b, c);
    if (!tri_set.insert(tk).second) return false;

    const glm::vec3& pa = cloud.positions[a];
    const glm::vec3& pb = cloud.positions[b];
    const glm::vec3& pc = cloud.positions[c];

    const glm::vec3 ab = pb - pa;
    const glm::vec3 bc = pc - pb;
    const glm::vec3 ca = pa - pc;
    const float lab = glm::length(ab);
    const float lbc = glm::length(bc);
    const float lca = glm::length(ca);
    if (lab < 1e-12f || lbc < 1e-12f || lca < 1e-12f) {
      tri_set.erase(tk);
      return false;
    }
    const float ang_a = std::acos(std::clamp(
        glm::dot(-ca, ab) / (lca * lab), -1.f, 1.f));
    const float ang_b = std::acos(std::clamp(
        glm::dot(-ab, bc) / (lab * lbc), -1.f, 1.f));
    const float ang_c = kPi - ang_a - ang_b;
    if (ang_a < min_angle_rad || ang_b < min_angle_rad ||
        ang_c < min_angle_rad) {
      tri_set.erase(tk);
      return false;
    }
    if (ang_a > max_angle_rad || ang_b > max_angle_rad ||
        ang_c > max_angle_rad) {
      tri_set.erase(tk);
      return false;
    }

    glm::vec3 fn = glm::cross(ab, pc - pa);
    if (glm::length(fn) < 1e-20f) {
      tri_set.erase(tk);
      return false;
    }
    fn = glm::normalize(fn);
    const glm::vec3 avg_n = glm::normalize(
        cloud.normals[a] + cloud.normals[b] + cloud.normals[c]);
    if (glm::dot(fn, avg_n) < -0.5f) {
      tri_set.erase(tk);
      return false;
    }
    const bool flip = glm::dot(fn, avg_n) < 0.f;

    const uint32_t va = vertOf(a);
    const uint32_t vb = vertOf(b);
    const uint32_t vc = vertOf(c);
    if (flip) mesh.indices.emplace_back(va, vc, vb);
    else      mesh.indices.emplace_back(va, vb, vc);
    return true;
  };

  std::vector<size_t> order(N);
  for (size_t i = 0; i < N; ++i) order[i] = i;
  std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    return cloud.positions[a].y < cloud.positions[b].y;
  });

  std::vector<size_t> nei;
  std::vector<float> d2;

  for (size_t oi = 0; oi < N; ++oi) {
    if (progress) {
      progress->setIndexed(oi, N, 0.05f, 0.93f,
                           "Greedy projection: triangulating", 64);
    }
    const size_t seed = order[oi];
    if (status[seed] != Status::Free) continue;

    const glm::vec3& sp = cloud.positions[seed];
    glm::vec3 sn = cloud.normals[seed];
    const float snl = glm::length(sn);
    if (snl < 1e-20f) {
      status[seed] = Status::Boundary;
      continue;
    }
    sn /= snl;

    glm::vec3 u, v;
    tangentFrame(sn, &u, &v);

    kd.knnAtPosition(sp, kmax, &nei, &d2);

    struct Cand {
      uint32_t idx;
      float ang;
      float dist;
    };
    std::vector<Cand> cands;
    cands.reserve(nei.size());
    for (size_t t = 0; t < nei.size(); ++t) {
      const size_t j = nei[t];
      if (j == seed) continue;
      if (d2[t] > radius2) continue;
      if (!canAccept(static_cast<uint32_t>(j), sn)) continue;
      const glm::vec3 dp = cloud.positions[j] - sp;
      const float du = glm::dot(dp, u);
      const float dv = glm::dot(dp, v);
      const float ang = std::atan2(dv, du);
      cands.push_back({static_cast<uint32_t>(j), ang, std::sqrt(d2[t])});
    }
    if (cands.size() < 2) {
      status[seed] = Status::Boundary;
      continue;
    }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.ang < b.ang; });

    const float closest = cands.front().dist;
    const float mu_max = closest * std::max(1.f, params.mu);
    std::vector<Cand> kept;
    kept.reserve(cands.size());
    for (const auto& c : cands) {
      if (c.dist <= mu_max) kept.push_back(c);
    }
    if (kept.size() < 2) {
      status[seed] = Status::Boundary;
      continue;
    }

    const float max_gap = kPi;
    bool seeded = false;
    for (size_t i = 0; i + 1 < kept.size(); ++i) {
      const Cand& a = kept[i];
      const Cand& b = kept[i + 1];
      const float gap = b.ang - a.ang;
      if (gap <= 0.f || gap >= max_gap) continue;
      if (emitTriangle(static_cast<uint32_t>(seed), a.idx, b.idx)) {
        seeded = true;
        status[a.idx] = Status::Fringe;
        status[b.idx] = Status::Fringe;
        pushEdge(a.idx, b.idx);
        pushEdge(static_cast<uint32_t>(seed), a.idx);
        pushEdge(static_cast<uint32_t>(seed), b.idx);
      }
    }

    if (kept.size() >= 3) {
      const Cand& a = kept.back();
      const Cand& b = kept.front();
      const float gap = (b.ang + 2.f * kPi) - a.ang;
      if (gap > 0.f && gap < kPi * 0.66f) {
        if (emitTriangle(static_cast<uint32_t>(seed), a.idx, b.idx)) {
          seeded = true;
          status[a.idx] = Status::Fringe;
          status[b.idx] = Status::Fringe;
          pushEdge(a.idx, b.idx);
          pushEdge(static_cast<uint32_t>(seed), a.idx);
          pushEdge(static_cast<uint32_t>(seed), b.idx);
        }
      }
    }

    status[seed] = seeded ? Status::Completed : Status::Boundary;

    int safety = 0;
    const int safety_cap = static_cast<int>(N) * 6 + 1024;
    while (!fringe.empty() && safety++ < safety_cap) {
      FringeEdge e = fringe.front();
      fringe.pop();
      const uint64_t ek = edgeKey(e.a, e.b);
      auto eit = edge_count.find(ek);
      if (eit == edge_count.end() || eit->second >= 2) continue;

      const glm::vec3& ea = cloud.positions[e.a];
      const glm::vec3& eb = cloud.positions[e.b];
      const glm::vec3 mid = 0.5f * (ea + eb);

      glm::vec3 en =
          glm::normalize(cloud.normals[e.a] + cloud.normals[e.b]);
      if (glm::length(en) < 1e-20f) en = sn;

      glm::vec3 eu, ev;
      tangentFrame(en, &eu, &ev);

      kd.knnAtPosition(mid, kmax, &nei, &d2);
      const float closest_d2 =
          d2.empty() ? radius2 : std::max(d2.front(), 1e-12f);
      const float mu_d = std::sqrt(closest_d2) * std::max(1.f, params.mu);

      uint32_t best = UINT32_MAX;
      float best_score = 1e30f;
      for (size_t t = 0; t < nei.size(); ++t) {
        const size_t j = nei[t];
        if (j == e.a || j == e.b) continue;
        if (d2[t] > radius2) continue;
        if (std::sqrt(d2[t]) > mu_d) continue;
        if (!canAccept(static_cast<uint32_t>(j), en)) continue;

        const uint64_t k1 = edgeKey(e.a, static_cast<uint32_t>(j));
        const uint64_t k2 = edgeKey(e.b, static_cast<uint32_t>(j));
        auto it1 = edge_count.find(k1);
        auto it2 = edge_count.find(k2);
        if ((it1 != edge_count.end() && it1->second >= 2) ||
            (it2 != edge_count.end() && it2->second >= 2))
          continue;

        const glm::vec3 pj = cloud.positions[j];
        const glm::vec3 dp = pj - mid;
        const float du = glm::dot(dp, eu);
        const float dv = glm::dot(dp, ev);
        const float r = std::sqrt(du * du + dv * dv);
        if (r < 1e-12f) continue;
        const float score = r;
        if (score < best_score) {
          best_score = score;
          best = static_cast<uint32_t>(j);
        }
      }

      if (best == UINT32_MAX) {
        edge_count[ek] = 2;
        if (status[e.a] == Status::Fringe) status[e.a] = Status::Boundary;
        if (status[e.b] == Status::Fringe) status[e.b] = Status::Boundary;
        continue;
      }

      if (!emitTriangle(e.a, e.b, best)) {
        edge_count[ek] = 2;
        continue;
      }
      edge_count[ek] = 2;
      pushEdge(e.a, best);
      pushEdge(e.b, best);
      if (status[best] == Status::Free) status[best] = Status::Fringe;
    }
  }

  if (progress) progress->set(1.f, "Greedy projection: complete");
  return mesh;
}

}
