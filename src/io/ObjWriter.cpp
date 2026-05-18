#include "ObjWriter.h"

#include <cmath>
#include <fstream>
#include <iomanip>

namespace io {

bool writeObj(const std::string& path, const TriangleMesh& mesh, std::string* error) {
  std::ofstream f(path);
  if (!f) {
    if (error) *error = "Cannot open file for write";
    return false;
  }
  f << std::fixed << std::setprecision(8);
  for (const auto& v : mesh.vertices) {
    f << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';
  }
  bool have_n = mesh.normals.size() == mesh.vertices.size();
  if (have_n) {
    for (const auto& n : mesh.normals) {
      f << "vn " << n.x << ' ' << n.y << ' ' << n.z << '\n';
    }
    for (const auto& tri : mesh.indices) {
      f << 'f';
      for (int c = 0; c < 3; ++c) {
        const unsigned i = tri[c] + 1u;
        f << ' ' << i << "//" << i;
      }
      f << '\n';
    }
  } else {
    for (const auto& tri : mesh.indices) {
      f << "f";
      for (int c = 0; c < 3; ++c) {
        f << ' ' << (tri[c] + 1u);
      }
      f << '\n';
    }
  }
  return true;
}

}
