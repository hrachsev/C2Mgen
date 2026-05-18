#include "OrbitCamera.h"

#include <algorithm>
#include <cmath>

void OrbitCamera::fitToSphere(const glm::vec3& center, float radius) {
  target_ = center;
  if (!std::isfinite(radius) || radius <= 1e-6f) radius = 1.f;
  distance_ = glm::clamp(radius * 2.55f, 1e-3f, 1e12f);
  fov_deg_ = 52.f;
}

void OrbitCamera::handleOrbit(float d_yaw, float d_pitch, float orbit_sensitivity) {
  yaw_ += d_yaw * orbit_sensitivity;
  pitch_ += d_pitch * orbit_sensitivity;
  const float limit = glm::radians(89.f);
  pitch_ = std::clamp(pitch_, -limit, limit);
}

void OrbitCamera::handlePan(float dx, float dy, float pan_speed) {
  const glm::vec3 forward(
      std::cos(pitch_) * std::cos(yaw_),
      std::sin(pitch_),
      std::cos(pitch_) * std::sin(yaw_));
  glm::vec3 up(0.f, 1.f, 0.f);
  glm::vec3 right = glm::normalize(glm::cross(forward, up));
  up = glm::normalize(glm::cross(right, forward));
  target_ += (-right * dx + up * dy) * pan_speed;
}

void OrbitCamera::handleZoom(float wheel, float zoom_speed) {
  if (wheel == 0.f) return;
  distance_ *= std::exp(-wheel * zoom_speed);
  distance_ = std::clamp(distance_, 1e-4f, 1e9f);
}

glm::mat4 OrbitCamera::projection(float aspect) const {
  return glm::perspective(glm::radians(fov_deg_), std::max(0.05f, aspect), 0.001f, 1e7f);
}

glm::mat4 OrbitCamera::view() const {
  const glm::vec3 forward(
      std::cos(pitch_) * std::cos(yaw_),
      std::sin(pitch_),
      std::cos(pitch_) * std::sin(yaw_));
  const glm::vec3 eye = target_ - forward * distance_;
  return glm::lookAt(eye, target_, glm::vec3(0.f, 1.f, 0.f));
}
