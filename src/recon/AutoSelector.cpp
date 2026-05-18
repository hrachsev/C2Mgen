#include "AutoSelector.h"

#include <algorithm>
#include <sstream>

#include <glm/glm.hpp>

namespace recon {
namespace {

struct Scores {
  float bpa = 0.f;
  float poisson = 0.f;
  float greedy = 0.f;
  float mc = 0.f;
};

Scores rankAlgorithms(const CloudStats& s) {
  Scores S;

  const float consistent = std::clamp(s.normal_consistency, 0.f, 1.f);
  const float cv = std::clamp(s.density_cv, 0.f, 2.f);
  const float pc = static_cast<float>(s.point_count);

  S.bpa = 1.0f;
  S.bpa += 1.4f * (1.0f - cv);
  S.bpa += 0.8f * std::clamp(consistent, 0.f, 1.f);
  S.bpa += (pc < 200000.f) ? 0.5f : -0.3f;
  S.bpa += (pc > 5000.f)   ? 0.3f : -0.6f;

  S.poisson = 0.6f;
  S.poisson += 1.6f * consistent;
  S.poisson += 1.0f * (1.0f - std::min(cv, 1.0f));
  S.poisson += (pc > 80000.f)   ? 0.7f : -0.2f;
  S.poisson += (pc > 400000.f)  ? 0.4f : 0.f;

  S.greedy = 0.9f;
  S.greedy += 1.2f * consistent;
  S.greedy += 0.8f * (1.0f - cv);
  S.greedy += (pc > 8000.f && pc < 600000.f) ? 0.6f : -0.4f;

  S.mc = 0.7f;
  S.mc += 0.6f * std::clamp(cv, 0.f, 1.0f);
  S.mc += 0.6f * consistent;
  S.mc += (pc > 250000.f) ? 0.6f : 0.f;

  return S;
}

}

AutoSuggestion suggest(const CloudStats& stats) {
  AutoSuggestion s;
  std::ostringstream r;

  const float extent = std::max(1e-6f, glm::length(stats.bbox_max - stats.bbox_min));
  const float spacing = std::max(1e-9f, stats.median_nn_distance);

  s.bpa.ball_radius = std::clamp(spacing, extent * 1e-4f, extent * 0.05f);
  s.bpa.radius_scale =
      glm::clamp(1.0f + 0.4f * stats.density_cv, 0.85f, 1.8f);
  s.bpa.min_triangle_aspect =
      glm::clamp(0.015f + 0.04f * (1.f - stats.density_cv), 0.005f, 0.08f);
  s.bpa.use_normals = stats.normal_consistency > 0.55f;

  if (stats.point_count < 30000u)        s.poisson.octree_depth = 6;
  else if (stats.point_count < 120000u)  s.poisson.octree_depth = 7;
  else if (stats.point_count < 500000u)  s.poisson.octree_depth = 8;
  else                                   s.poisson.octree_depth = 9;
  s.poisson.solver_iterations =
      glm::clamp(60 + static_cast<int>(120.f * (1.f - stats.normal_consistency)),
                 40, 200);
  s.poisson.screened_weight =
      glm::clamp(0.8f + 1.2f * stats.normal_consistency, 0.5f, 2.5f);
  s.poisson.iso_level = 0.f;

  if (stats.point_count < 30000u)        s.marching_cubes.grid_resolution = 64;
  else if (stats.point_count < 200000u)  s.marching_cubes.grid_resolution = 128;
  else if (stats.point_count < 1500000u) s.marching_cubes.grid_resolution = 192;
  else                                   s.marching_cubes.grid_resolution = 256;
  s.marching_cubes.truncation_voxels =
      glm::clamp(2.0f + 4.0f * stats.density_cv, 2.0f, 8.0f);
  s.marching_cubes.knn_for_distance =
      glm::clamp(6 + static_cast<int>(8.f * stats.density_cv), 4, 16);
  s.marching_cubes.iso_level = 0.f;

  s.greedy.search_radius_mult =
      glm::clamp(2.0f + 1.5f * stats.density_cv, 2.0f, 4.5f);
  s.greedy.mu = glm::clamp(2.0f + stats.density_cv, 2.0f, 3.5f);
  s.greedy.max_nearest_neighbors =
      glm::clamp(60 + static_cast<int>(40.f * stats.density_cv), 60, 160);
  s.greedy.max_surface_angle_deg =
      glm::clamp(40.0f + 20.0f * (1.f - stats.normal_consistency), 30.0f, 70.0f);
  s.greedy.min_angle_deg = 10.f;
  s.greedy.max_angle_deg = 120.f;
  s.greedy.consistent_normals = stats.normal_consistency < 0.85f;

  if (stats.point_count == 0) {
    s.method = ReconstructionMethod::BallPivoting;
    s.rationale = "Empty cloud - nothing to recommend.";
    return s;
  }

  Scores sc = rankAlgorithms(stats);

  struct Candidate {
    ReconstructionMethod::Type m;
    float score;
    const char* name;
  };
  Candidate cands[4] = {
      {ReconstructionMethod::BallPivoting,     sc.bpa,     "Ball Pivoting"},
      {ReconstructionMethod::Poisson,          sc.poisson, "Poisson"},
      {ReconstructionMethod::GreedyProjection, sc.greedy,  "Greedy Projection Triangulation"},
      {ReconstructionMethod::MarchingCubes,    sc.mc,      "Marching Cubes (TSDF)"},
  };
  std::sort(cands, cands + 4,
            [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

  s.method = cands[0].m;

  r << "Recommended algorithm: " << cands[0].name
    << " (score=" << cands[0].score << ").\n";
  r << "Cloud profile: "
    << stats.point_count << " points, "
    << "density CV=" << stats.density_cv
    << ", normal consistency=" << stats.normal_consistency
    << ", median spacing=" << stats.median_nn_distance
    << ", extent=" << extent << ".\n";

  switch (s.method) {
    case ReconstructionMethod::BallPivoting:
      r << "Why: spacing looks " << (stats.density_cv < 0.4f ? "uniform" : "moderately uneven")
        << " and the cloud is in a size range where rolling-ball reconstruction "
           "stays close to the input samples.\n";
      r << "Tuned: radius=" << s.bpa.ball_radius
        << " (~ median spacing), scale=" << s.bpa.radius_scale
        << ", aspect=" << s.bpa.min_triangle_aspect << ".\n";
      break;
    case ReconstructionMethod::Poisson:
      r << "Why: large/dense cloud with consistent normals - screened Poisson "
           "fills holes and produces a watertight surface.\n";
      r << "Tuned: depth=" << s.poisson.octree_depth
        << ", iterations=" << s.poisson.solver_iterations
        << ", screened_weight=" << s.poisson.screened_weight << ".\n";
      break;
    case ReconstructionMethod::GreedyProjection:
      r << "Why: smooth manifold with workable normals - greedy projection "
           "follows the samples without over-smoothing.\n";
      r << "Tuned: search_radius=" << s.greedy.search_radius_mult
        << "x median, mu=" << s.greedy.mu
        << ", max_surface_angle=" << s.greedy.max_surface_angle_deg << "deg.\n";
      break;
    case ReconstructionMethod::MarchingCubes:
      r << "Why: density varies a lot or the cloud is volumetric - building a "
           "truncated SDF and meshing the zero iso-surface is the safest bet.\n";
      r << "Tuned: grid=" << s.marching_cubes.grid_resolution
        << ", truncation=" << s.marching_cubes.truncation_voxels
        << " voxels, knn=" << s.marching_cubes.knn_for_distance << ".\n";
      break;
    default:
      break;
  }

  if (cands[1].score > cands[0].score - 0.3f) {
    r << "Close runner-up: " << cands[1].name
      << " (score=" << cands[1].score
      << "). Try it if the result is unsatisfying.\n";
  }

  if (stats.point_count <= 4500u) {
    r << "Note: small cloud (< 5k points). Expect holes or low triangle counts.\n";
  }
  if (stats.point_count >= 1500000u) {
    r << "Note: very large cloud. Reconstruction may take a while.\n";
  }
  if (stats.normal_consistency < 0.45f) {
    r << "Note: normals look inconsistent. Consider re-orienting or use "
         "Ball Pivoting / Marching Cubes which tolerate this better.\n";
  }

  s.rationale = r.str();
  return s;
}

}
