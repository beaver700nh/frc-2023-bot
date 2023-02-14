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
};
