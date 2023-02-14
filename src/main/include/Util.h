// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

namespace Util {
  int ramp(auto *var, auto target, auto tick) {
    if (*var < target) {
      *var = fmin(target, *var + tick);
      return 1;
    }

    if (*var > target) {
      *var = fmax(target, *var - tick);
      return -1;
    }

    return 0;
  }

  auto constrained(auto var, auto min, auto max) {
    if (var < min) {
      return min;
    }

    if (var > max) {
      return max;
    }

    return var;
  }

  auto thresholded(auto var, auto min, auto max) {
    if (min < var && var < max) {
      return (decltype(var)) 0;
    }

    return var;
  }
};
