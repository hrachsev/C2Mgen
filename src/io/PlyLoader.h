#pragma once

#include <string>

#include "core/GeometryTypes.h"

namespace io {

bool loadPly(const std::string& path, PointCloud* out, std::string* error = nullptr);

}
