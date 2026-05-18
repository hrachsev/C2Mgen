#pragma once

#include <string>

#include "core/GeometryTypes.h"
#include "recon/CloudStats.h"

namespace recon {

struct AutoSuggestion {
  ReconstructionMethod::Type method = ReconstructionMethod::BallPivoting;
  BPAParams bpa{};
  PoissonParams poisson{};
  MarchingCubesParams marching_cubes{};
  GreedyParams greedy{};
  std::string rationale;
};

AutoSuggestion suggest(const CloudStats& stats);

}
