#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class OrbitCamera {
 public:
  void setTarget(const glm::vec3& center) { target_ = center; }
  void fitToSphere(const glm::vec3& center, float radius);
  void handleOrbit(float d_yaw, float d_pitch, float orbit_sensitivity);
  void handlePan(float dx, float dy, float pan_speed);
  void handleZoom(float wheel, float zoom_speed);

  glm::mat4 projection(float aspect) const;
  glm::mat4 view() const;

  float distance() const { return distance_; }

 private:
  glm::vec3 target_{0.f};
  float yaw_ = 0.78f;
  float pitch_ = 0.62f;
  float distance_ = 3.f;
  float fov_deg_ = 55.f;
};
