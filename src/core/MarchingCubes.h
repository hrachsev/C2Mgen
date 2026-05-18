#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace mc {

void extractIsoSurface(const std::vector<float>& field,
                       int nx,
                       int ny,
                       int nz,
                       const glm::vec3& origin,
                       const glm::vec3& cell,
                       float iso,
                       std::vector<glm::vec3>* out_pos,
                       std::vector<glm::vec3>* out_nrm,
                       std::vector<glm::uvec3>* out_idx);

}
