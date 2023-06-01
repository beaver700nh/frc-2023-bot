// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>
#include <utility>

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Arm.h"
#include "subsystems/Drive.h"
#include "subsystems/Pneumatics.h"

#include "commands/ExampleCommand.h"
#include "commands/ClawControl.h"
#include "commands/HomeArm.h"

#include "subsystems/ExampleSubsystem.h"
#include "Movement.h"

namespace autos {
  extern std::vector<std::pair<std::string, frc2::CommandPtr>> autos;

  void populate(Arm *arm, Drive *drive, Pneumatics *pneu);
}
