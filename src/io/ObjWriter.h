#pragma once

#include <string>

#include "core/GeometryTypes.h"

namespace io {

bool writeObj(const std::string& path, const TriangleMesh& mesh, std::string* error = nullptr);

}
