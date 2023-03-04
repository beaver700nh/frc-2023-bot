// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

namespace autos {

std::vector<std::pair<std::string, frc2::CommandPtr>> autos;

void populate(Arm *arm, Drive *drive, Pneumatics *pneu) {
  autos.emplace_back(
    "2 cube low & high (near wall)",
    frc2::cmd::Sequence(
      Movement::GenerateCommand(drive, {-0.6_m, 0.0_m, 0.0_deg}, true),
      ClawControl(pneu, true),
      arm->m_position_pickup,
      Movement::GenerateCommand(drive, {5.0_m, 0.0_m, 0.0_deg}),
      ClawControl(pneu, false),
      frc2::WaitCommand(500_ms),
      SetArmPosition(arm, {std::nullopt, 125.0, std::nullopt}),
      arm->m_position_hiCubeBack,
      Movement::GenerateCommand(drive, {-4.9_m, 0.2_m, -3.0_deg}, true),
      ClawControl(pneu, true)
    )
  );

  autos.emplace_back(
    "2 cube high & mid (untested)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, 125, std::nullopt}),
      arm->m_position_hiCubeBack,
      ClawControl(pneu, true),
      frc2::WaitCommand(1000_ms),
      arm->m_position_pickupFront,
      Movement::GenerateCommand(drive, {5.0_m, 0.0_m, 0.0_deg}),
      ClawControl(pneu, false),
      frc2::WaitCommand(1000_ms),
      SetArmPosition(arm, {std::nullopt, 125.0, std::nullopt}),
      arm->m_position_hiCubeBack,
      Movement::GenerateCommand(drive, {-4.9_m, 0.2_m, -3.0_deg}, true),
      ClawControl(pneu, true)
    )
  );

  autos.emplace_back(
    "1 cube low",
    frc2::cmd::Sequence(
      Movement::GenerateCommand(drive, {-0.6_m, 0.0_m, 0.0_deg}, true),
      Movement::GenerateCommand(drive, {0.6_m, 0.0_m, 0.0_deg})
    )
  );

  autos.emplace_back(
    "1 cube high (untested)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, 125, std::nullopt}),
      arm->m_position_hiCubeBack,
      ClawControl(pneu, true),
      SetArmPositionWait(arm, {0,std::nullopt,0}),
      SetArmPosition(arm, {0,0,0})
    )
  );
}

}