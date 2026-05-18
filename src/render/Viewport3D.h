#pragma once

#include <cstdint>

#include <glm/glm.hpp>

#include "core/GeometryTypes.h"
#include "ui/OrbitCamera.h"

namespace render {

struct ViewportRect {
  int x = 0, y = 0, w = 0, h = 0;
};

void drawPointCloud(int framebuffer_height,
                    const ViewportRect& vp,
                    const OrbitCamera& cam,
                    const PointCloud& cloud,
                    const glm::vec3& color,
                    float point_pixel_size);

void drawMesh(int framebuffer_height,
              const ViewportRect& vp,
              const OrbitCamera& cam,
              const TriangleMesh& mesh,
              const glm::vec3& color,
              bool wireframe);

void computeBounds(const PointCloud& cloud, glm::vec3* center, float* radius);
void computeBounds(const TriangleMesh& mesh, glm::vec3* center, float* radius);

}
