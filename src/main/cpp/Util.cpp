// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Util.h"

namespace Util {

units::radian_t scaleAngle(units::radian_t angle) {
  double ang = angle.to<double>();
  constexpr double M_2PI = 2 * M_PI;
  ang = std::fmod(ang, M_2PI);
  ang -= (ang > M_PI) ? M_2PI : (ang < -M_PI) ? -M_2PI : 0;
  return units::radian_t {ang};
}

}