// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

namespace autos {

std::vector<std::pair<std::string, frc2::CommandPtr>> autos;

void populate(Arm *arm, Drive *drive, Pneumatics *pneu) {
  autos.emplace_back(
    "2 cube",
    frc2::cmd::Sequence(
      Movement::GenerateCommand(drive, {-0.6_m, 0.0_m, 0.0_deg}, true),
      ClawControl(pneu, true),
      arm->m_position_pickup,
      Movement::GenerateCommand(drive, {5.0_m, 0.0_m, 0.0_deg}),
      ClawControl(pneu, false),
      frc2::WaitCommand(500_ms),
      SetArmPosition(arm, {std::nullopt, -125.0, std::nullopt}),
      arm->m_position_loCubeBack,
      Movement::GenerateCommand(drive, {-4.9_m, 0.2_m, -3.0_deg}, true),
      ClawControl(pneu, true)
    )
  );
  autos.emplace_back(
    "simple cube",
    frc2::cmd::Sequence(
      Movement::GenerateCommand(drive, {-0.6_m, 0.0_m, 0.0_deg}, true),
      Movement::GenerateCommand(drive, {0.6_m, 0.0_m, 0.0_deg})
    )
  );
}

}