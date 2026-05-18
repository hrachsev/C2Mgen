#pragma once

#include <glm/glm.hpp>
#include <limits>
#include <vector>

class KdTree {
 public:
  explicit KdTree(const std::vector<glm::vec3>& pts);

  void knn(size_t idx, int k,
           std::vector<size_t>* out_indices,
           std::vector<float>* out_d2 = nullptr) const;

  void knnAtPosition(const glm::vec3& p, int k,
                     std::vector<size_t>* out_indices,
                     std::vector<float>* out_d2 = nullptr) const;

  const std::vector<glm::vec3>& points() const { return points_; }

 private:
  struct Node {
    size_t split_axis = 0;
    float split_val = 0.f;
    int left = -1;
    int right = -1;
    int point_index = -1;
    int count = 0;
  };

  void knnRecursive(int node,
                      const glm::vec3& p,
                      int k,
                      std::vector<std::pair<float, size_t>>& heap) const;

  int build_recursive(std::vector<int>& indices, int axis);

  std::vector<glm::vec3> points_;
  std::vector<Node> nodes_;
};
