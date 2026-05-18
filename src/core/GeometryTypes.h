#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

struct PointCloud {
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
};

struct TriangleMesh {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  std::vector<glm::uvec3> indices;
};

struct ReconstructionMethod {
  enum Type : int32_t {
    Auto = 0,
    BallPivoting = 1,
    Poisson = 2,
    MarchingCubes = 3,
    GreedyProjection = 4,
  };
};

struct BPAParams {
  float ball_radius = 1.0f;
  float radius_scale = 1.0f;
  float min_triangle_aspect = 0.02f;
  bool use_normals = true;
};

struct PoissonParams {
  int octree_depth = 8;
  int solver_iterations = 80;
  float screened_weight = 1.0f;
  float iso_level = 0.0f;
};

struct MarchingCubesParams {
  int grid_resolution = 128;
  float truncation_voxels = 3.0f;
  int knn_for_distance = 8;
  float iso_level = 0.0f;
};

struct GreedyParams {
  float search_radius_mult = 2.75f;
  float mu = 2.5f;
  int max_nearest_neighbors = 80;
  float max_surface_angle_deg = 45.0f;
  float min_angle_deg = 10.0f;
  float max_angle_deg = 120.0f;
  bool consistent_normals = true;
};

struct VoxelRemeshParams {
  float voxel_size = 0.02f;
  int hole_close_passes = 1;
  float bbox_padding_voxels = 3.f;
  int max_grid_resolution_per_axis = 320;
};
