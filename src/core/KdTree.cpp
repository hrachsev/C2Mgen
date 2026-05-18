#include "KdTree.h"

#include <algorithm>
#include <limits>
#include <numeric>
namespace {

void pushMaxHeapK(std::vector<std::pair<float, size_t>>& heap,
                  int k,
                  float d2,
                  size_t idx) {
  if (static_cast<int>(heap.size()) < k) {
    heap.emplace_back(d2, idx);
    std::push_heap(heap.begin(), heap.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
    });
  } else if (!heap.empty() && d2 < heap.front().first) {
    std::pop_heap(heap.begin(), heap.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
    });
    heap.back() = {d2, idx};
    std::push_heap(heap.begin(), heap.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
    });
  }
}

}

KdTree::KdTree(const std::vector<glm::vec3>& pts) : points_(pts) {
  if (points_.empty()) return;
  std::vector<int> ids(points_.size());
  std::iota(ids.begin(), ids.end(), 0);
  [[maybe_unused]] const int root = build_recursive(ids, 0);
}

int KdTree::build_recursive(std::vector<int>& ids, int axis) {
  const int n = static_cast<int>(ids.size());
  Node node{};
  node.split_axis = static_cast<size_t>(axis % 3);
  node.count = n;
  node.point_index = -1;

  if (n == 1) {
    node.left = node.right = -1;
    node.point_index = ids[0];
    nodes_.push_back(node);
    return static_cast<int>(nodes_.size()) - 1;
  }

  const int mid = n / 2;
  const size_t ax = node.split_axis;
  auto cmp = [&](int a, int b) {
    const int aix = static_cast<int>(ax);
    return points_[static_cast<size_t>(a)][aix] < points_[static_cast<size_t>(b)][aix];
  };
  std::nth_element(ids.begin(), ids.begin() + mid, ids.end(), cmp);
  node.split_val = points_[static_cast<size_t>(ids[mid])][static_cast<int>(ax)];

  std::vector<int> left_ids(ids.begin(), ids.begin() + mid);
  std::vector<int> right_ids(ids.begin() + mid, ids.end());

  nodes_.push_back(node);
  const int self = static_cast<int>(nodes_.size()) - 1;

  const int L = build_recursive(left_ids, axis + 1);
  const int R = build_recursive(right_ids, axis + 1);
  nodes_[static_cast<size_t>(self)].left = L;
  nodes_[static_cast<size_t>(self)].right = R;
  return self;
}

void KdTree::knnRecursive(int node_idx,
                          const glm::vec3& p,
                          int k,
                          std::vector<std::pair<float, size_t>>& heap) const {
  if (node_idx < 0 || node_idx >= static_cast<int>(nodes_.size())) return;
  const Node& node = nodes_[static_cast<size_t>(node_idx)];

  if (node.left < 0 && node.right < 0) {
    if (node.point_index >= 0) {
      const glm::vec3& q = points_[static_cast<size_t>(node.point_index)];
      const glm::vec3 d = p - q;
      const float d2 = glm::dot(d, d);
      pushMaxHeapK(heap, k, d2, static_cast<size_t>(node.point_index));
    }
    return;
  }

  const int ax = static_cast<int>(node.split_axis);
  const float dv = p[ax] - node.split_val;
  int first = node.left;
  int second = node.right;
  if (dv >= 0.f) {
    std::swap(first, second);
  }

  knnRecursive(first, p, k, heap);

  float plane_d2 = dv * dv;
  const bool accept_second =
      static_cast<int>(heap.size()) < k ||
      (!heap.empty() && plane_d2 < heap.front().first);
  if (accept_second && second >= 0) {
    knnRecursive(second, p, k, heap);
  }
}

void KdTree::knnAtPosition(const glm::vec3& p,
                           int k,
                           std::vector<size_t>* out_indices,
                           std::vector<float>* out_d2) const {
  out_indices->clear();
  if (out_d2) out_d2->clear();
  if (points_.empty() || k <= 0) return;

  std::vector<std::pair<float, size_t>> heap;
  heap.reserve(static_cast<size_t>(k));
  knnRecursive(0, p, k, heap);

  std::sort(heap.begin(), heap.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  for (const auto& pr : heap) {
    out_indices->push_back(pr.second);
    if (out_d2) out_d2->push_back(pr.first);
  }
}

void KdTree::knn(size_t idx,
                 int k,
                 std::vector<size_t>* out_indices,
                 std::vector<float>* out_d2) const {
  if (idx >= points_.size()) {
    out_indices->clear();
    if (out_d2) out_d2->clear();
    return;
  }
  knnAtPosition(points_[idx], k + 1, out_indices, out_d2);
  for (size_t i = 0; i < out_indices->size(); ++i) {
    if ((*out_indices)[i] == idx) {
      out_indices->erase(out_indices->begin() + static_cast<long>(i));
      if (out_d2 && i < out_d2->size())
        out_d2->erase(out_d2->begin() + static_cast<long>(i));
      break;
    }
  }
  if (static_cast<int>(out_indices->size()) > k) {
    out_indices->resize(static_cast<size_t>(k));
    if (out_d2) out_d2->resize(static_cast<size_t>(k));
  }
}
