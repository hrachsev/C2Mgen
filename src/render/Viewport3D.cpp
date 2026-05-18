#include "Viewport3D.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <GL/gl.h>

#include <glm/gtc/type_ptr.hpp>

namespace render {
namespace {

void applyViewportScissor(const ViewportRect& vp, int fb_h) {

  const int gl_y = fb_h - (vp.y + vp.h);
  glViewport(vp.x, gl_y, vp.w, vp.h);
  glEnable(GL_SCISSOR_TEST);
  glScissor(vp.x, gl_y, vp.w, vp.h);
}

void clearDepthColor() {
  glClearColor(0.08f, 0.08f, 0.10f, 1.f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

}

void computeBounds(const PointCloud& cloud, glm::vec3* center, float* radius) {
  if (cloud.positions.empty()) {
    *center = glm::vec3(0.f);
    *radius = 1.f;
    return;
  }
  glm::vec3 bmin = cloud.positions[0];
  glm::vec3 bmax = cloud.positions[0];
  for (const auto& p : cloud.positions) {
    bmin = glm::min(bmin, p);
    bmax = glm::max(bmax, p);
  }
  *center = 0.5f * (bmin + bmax);
  *radius = 0.5f * glm::length(bmax - bmin);
  if (*radius < 1e-5f) *radius = 1e-3f;
}

void computeBounds(const TriangleMesh& mesh, glm::vec3* center, float* radius) {
  if (mesh.vertices.empty()) {
    *center = glm::vec3(0.f);
    *radius = 1.f;
    return;
  }
  glm::vec3 bmin = mesh.vertices[0];
  glm::vec3 bmax = mesh.vertices[0];
  for (const auto& p : mesh.vertices) {
    bmin = glm::min(bmin, p);
    bmax = glm::max(bmax, p);
  }
  *center = 0.5f * (bmin + bmax);
  *radius = 0.5f * glm::length(bmax - bmin);
  if (*radius < 1e-5f) *radius = 1e-3f;
}

void drawPointCloud(int framebuffer_height,
                    const ViewportRect& vp,
                    const OrbitCamera& cam,
                    const PointCloud& cloud,
                    const glm::vec3& color,
                    float point_pixel_size) {
  if (vp.w <= 1 || vp.h <= 1) return;

  GLint cur_vp[4];
  glGetIntegerv(GL_VIEWPORT, cur_vp);

  applyViewportScissor(vp, framebuffer_height);
  clearDepthColor();

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glPointSize(std::clamp(point_pixel_size, 1.f, 128.f));

  const float aspect = static_cast<float>(vp.w) / static_cast<float>(vp.h);
  const glm::mat4 P = cam.projection(aspect);
  const glm::mat4 V = cam.view();
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(glm::value_ptr(P));
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(glm::value_ptr(V));

  glBegin(GL_POINTS);
  glColor3f(color.r, color.g, color.b);
  for (const auto& p : cloud.positions) {
    glVertex3f(p.x, p.y, p.z);
  }
  glEnd();

  glDisable(GL_SCISSOR_TEST);
  glViewport(cur_vp[0], cur_vp[1], cur_vp[2], cur_vp[3]);
}

void drawMesh(int framebuffer_height,
              const ViewportRect& vp,
              const OrbitCamera& cam,
              const TriangleMesh& mesh,
              const glm::vec3& color,
              bool wireframe) {
  if (vp.w <= 1 || vp.h <= 1) return;

  GLint cur_vp[4];
  glGetIntegerv(GL_VIEWPORT, cur_vp);

  applyViewportScissor(vp, framebuffer_height);
  clearDepthColor();

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  if (wireframe) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  } else {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  const float aspect = static_cast<float>(vp.w) / static_cast<float>(vp.h);
  const glm::mat4 P = cam.projection(aspect);
  const glm::mat4 V = cam.view();

  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(glm::value_ptr(P));
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(glm::value_ptr(V));

  glColor3f(color.r, color.g, color.b);
  glBegin(GL_TRIANGLES);
  for (const auto& tri : mesh.indices) {
    for (int k = 0; k < 3; ++k) {
      const unsigned int idx = tri[k];
      if (idx < mesh.vertices.size()) {
        const glm::vec3& v = mesh.vertices[idx];
        if (!mesh.normals.empty() && idx < mesh.normals.size()) {
          const glm::vec3& n = mesh.normals[idx];
          glNormal3f(n.x, n.y, n.z);
        }
        glVertex3f(v.x, v.y, v.z);
      }
    }
  }
  glEnd();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glDisable(GL_SCISSOR_TEST);
  glViewport(cur_vp[0], cur_vp[1], cur_vp[2], cur_vp[3]);
}

}
