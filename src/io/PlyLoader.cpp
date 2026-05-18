#include "PlyLoader.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace io {
namespace {

struct Prop {
  std::string name;
  std::string type;
  int list_count_type_size = 0;
  int list_prop_type_size = 0;
};

struct Element {
  std::string name;
  size_t count = 0;
  std::vector<Prop> props;
  size_t vertex_stride_bytes = 0;
  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  int offset_nx = -1;
  int offset_ny = -1;
  int offset_nz = -1;
  std::string prop_type_x, prop_type_y, prop_type_z;
  std::string prop_type_nx, prop_type_ny, prop_type_nz;
};

int typeSize(const std::string& t) {
  if (t == "char" || t == "uchar" || t == "int8" || t == "uint8") return 1;
  if (t == "short" || t == "ushort" || t == "int16" || t == "uint16") return 2;
  if (t == "int" || t == "uint" || t == "int32" || t == "uint32" || t == "float" ||
      t == "float32")
    return 4;
  if (t == "double" || t == "float64") return 8;
  return 0;
}

int64_t readIntegerFromStream(std::istream& f, int sz) {
  int64_t v = 0;
  if (sz == 1) {
    uint8_t u;
    f.read(reinterpret_cast<char*>(&u), 1);
    return static_cast<int64_t>(u);
  }
  if (sz == 2) {
    uint16_t u;
    f.read(reinterpret_cast<char*>(&u), sizeof(u));
    return static_cast<int64_t>(u);
  }
  if (sz == 4) {
    int32_t u;
    f.read(reinterpret_cast<char*>(&u), sizeof(u));
    return static_cast<int64_t>(u);
  }
  (void)v;
  return 0;
}

void skipBinaryElementRow(std::istream& f, const Element& el) {
  for (const Prop& pr : el.props) {
    if (pr.list_count_type_size > 0) {
      const int64_t list_len = readIntegerFromStream(f, pr.list_count_type_size);
      for (int64_t k = 0; k < list_len && f; ++k) {
        f.seekg(pr.list_prop_type_size, std::ios::cur);
      }
    } else {
      const int sz = typeSize(pr.type);
      if (sz > 0) f.seekg(sz, std::ios::cur);
    }
  }
}

float readFloatAt(const unsigned char* base, int off, const std::string& type) {
  if (off < 0) return 0.f;
  const unsigned char* p = base + off;
  if (type == "float" || type == "float32") {
    float f;
    std::memcpy(&f, p, sizeof(float));
    return f;
  }
  if (type == "double" || type == "float64") {
    double d;
    std::memcpy(&d, p, sizeof(double));
    return static_cast<float>(d);
  }
  if (type == "uchar" || type == "uint8") {
    return static_cast<float>(*reinterpret_cast<const unsigned char*>(p));
  }
  if (type == "char" || type == "int8") {
    return static_cast<float>(*reinterpret_cast<const signed char*>(p));
  }
  if (type == "short" || type == "int16") {
    int16_t s;
    std::memcpy(&s, p, sizeof(int16_t));
    return static_cast<float>(s);
  }
  if (type == "ushort" || type == "uint16") {
    uint16_t s;
    std::memcpy(&s, p, sizeof(uint16_t));
    return static_cast<float>(s);
  }
  if (type == "int" || type == "int32") {
    int32_t s;
    std::memcpy(&s, p, sizeof(int32_t));
    return static_cast<float>(s);
  }
  if (type == "uint" || type == "uint32") {
    uint32_t s;
    std::memcpy(&s, p, sizeof(uint32_t));
    return static_cast<float>(s);
  }
  return 0.f;
}

bool readHeader(std::istream& in, std::vector<Element>* elements, bool* binary,
                bool* little_endian, std::string* err) {
  std::string magic;
  std::getline(in, magic);
  if (magic.size() < 3 || magic.compare(0, 3, "ply") != 0) {
    *err = "Not a PLY file";
    return false;
  }
  *binary = false;
  *little_endian = true;
  elements->clear();

  std::string line;
  Element* cur = nullptr;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string key;
    if (!(ls >> key)) continue;
    if (key == "format") {
      std::string fmt, ver;
      ls >> fmt >> ver;
      if (fmt == "binary_little_endian")
        *binary = true;
      else if (fmt == "binary_big_endian") {
        *binary = true;
        *little_endian = false;
      }
    } else if (key == "element") {
      Element e{};
      ls >> e.name >> e.count;
      elements->push_back(e);
      cur = &elements->back();
    } else if (key == "property" && cur) {
      std::string tok;
      ls >> tok;
      if (tok == "list") {
        Prop p{};
        std::string ctype, etype;
        ls >> ctype >> etype >> p.name;
        p.list_count_type_size = typeSize(ctype);
        p.list_prop_type_size = typeSize(etype);
        p.type = etype;
        cur->props.push_back(p);
      } else {
        Prop p{};
        p.type = tok;
        ls >> p.name;
        cur->props.push_back(p);
      }
    } else if (key == "end_header") {
      break;
    }
  }

  for (auto& el : *elements) {
    size_t agg = 0;
    for (const Prop& pr : el.props) {
      if (pr.list_count_type_size > 0) {

        agg = 0;
        break;
      }
      const int sz = typeSize(pr.type);
      if (pr.name == "x") {
        el.offset_x = static_cast<int>(agg);
        el.prop_type_x = pr.type;
      } else if (pr.name == "y") {
        el.offset_y = static_cast<int>(agg);
        el.prop_type_y = pr.type;
      } else if (pr.name == "z") {
        el.offset_z = static_cast<int>(agg);
        el.prop_type_z = pr.type;
      } else if (pr.name == "nx") {
        el.offset_nx = static_cast<int>(agg);
        el.prop_type_nx = pr.type;
      } else if (pr.name == "ny") {
        el.offset_ny = static_cast<int>(agg);
        el.prop_type_ny = pr.type;
      } else if (pr.name == "nz") {
        el.offset_nz = static_cast<int>(agg);
        el.prop_type_nz = pr.type;
      }
      agg += sz > 0 ? static_cast<size_t>(sz) : 0;
    }
    el.vertex_stride_bytes = agg;
  }
  return true;
}

bool readAsciiNumber(std::istream& in, const std::string& type, double* out) {
  if (type == "float" || type == "float32") {
    float f;
    in >> f;
    *out = f;
    return !in.fail();
  }
  if (type == "double" || type == "float64") {
    in >> *out;
    return !in.fail();
  }
  if (type == "uchar" || type == "uint8") {
    unsigned u;
    in >> u;
    *out = u;
    return !in.fail();
  }
  int64_t iv;
  in >> iv;
  *out = static_cast<double>(iv);
  return !in.fail();
}

}

bool loadPly(const std::string& path, PointCloud* out, std::string* error) {
  out->positions.clear();
  out->normals.clear();
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    if (error) *error = "Cannot open file";
    return false;
  }
  std::vector<Element> elements;
  bool binary = false;
  bool little_endian = true;
  std::string err;
  if (!readHeader(f, &elements, &binary, &little_endian, &err)) {
    if (error) *error = err;
    return false;
  }
  if (!little_endian && binary) {
    if (error) *error = "Big-endian binary PLY not supported";
    return false;
  }

  Element* vtx = nullptr;
  for (auto& e : elements) {
    if (e.name == "vertex" && e.offset_x >= 0 && e.offset_y >= 0 && e.offset_z >= 0 &&
        e.vertex_stride_bytes > 0) {
      vtx = &e;
      break;
    }
  }
  if (!vtx) {
    if (error) *error = "PLY has no fixed-stride vertex element with x/y/z";
    return false;
  }

  if (!binary) {
    for (const auto& el : elements) {
      for (size_t i = 0; i < el.count; ++i) {
        std::string raw;
        if (!std::getline(f, raw)) break;
        if (el.name != "vertex") continue;
        std::istringstream ls(raw);
        glm::vec3 p(0.f);
        glm::vec3 n(0.f);
        bool have_n = vtx->offset_nx >= 0;
        for (const Prop& pr : el.props) {
          if (pr.list_count_type_size > 0) continue;
          double val = 0.0;
          if (!readAsciiNumber(ls, pr.type, &val)) continue;
          if (pr.name == "x") p.x = static_cast<float>(val);
          if (pr.name == "y") p.y = static_cast<float>(val);
          if (pr.name == "z") p.z = static_cast<float>(val);
          if (pr.name == "nx") n.x = static_cast<float>(val);
          if (pr.name == "ny") n.y = static_cast<float>(val);
          if (pr.name == "nz") n.z = static_cast<float>(val);
        }
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
          out->positions.push_back(p);
          if (have_n && vtx->name == el.name) {
            const glm::vec3 nn = glm::length(n) > 1e-20f ? glm::normalize(n) : glm::vec3(0, 1, 0);
            out->normals.push_back(nn);
          }
        }
      }
    }
    return !out->positions.empty();
  }

  for (const auto& el : elements) {
    if (el.name == "vertex") {
      const size_t stride = el.vertex_stride_bytes;
      std::vector<unsigned char> row(stride);
      const bool have_n = el.offset_nx >= 0 && el.offset_ny >= 0 && el.offset_nz >= 0;
      for (size_t i = 0; i < el.count; ++i) {
        f.read(reinterpret_cast<char*>(row.data()), static_cast<std::streamsize>(stride));
        if (!f) break;
        const unsigned char* base = row.data();
        glm::vec3 p(readFloatAt(base, el.offset_x, el.prop_type_x),
                    readFloatAt(base, el.offset_y, el.prop_type_y),
                    readFloatAt(base, el.offset_z, el.prop_type_z));
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
          out->positions.push_back(p);
          if (have_n) {
            glm::vec3 n(readFloatAt(base, el.offset_nx, el.prop_type_nx),
                        readFloatAt(base, el.offset_ny, el.prop_type_ny),
                        readFloatAt(base, el.offset_nz, el.prop_type_nz));
            out->normals.push_back(glm::length(n) > 1e-20f ? glm::normalize(n) : glm::vec3(0, 1, 0));
          }
        }
      }
    } else {
      for (size_t i = 0; i < el.count && f; ++i) {
        skipBinaryElementRow(f, el);
      }
    }
  }

  if (out->positions.empty()) {
    if (error) *error = "No vertices read";
    return false;
  }
  return true;
}

}
