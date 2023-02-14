// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveRunner.h"

#include <iostream>

DriveRunner::DriveRunner(Drive *drive, frc2::CommandXboxController *driverController)
  : m_drive(drive), m_driverController(driverController) {
  // Register that this command requires the subsystem.
  AddRequirements(m_drive);
}

void DriveRunner::Execute() {
  std::cout << "TICK\n";

  const auto x = m_driverController->GetLeftY(); // controller y-axis is reversed
  const auto r = m_driverController->GetRightX();

  m_drive->SetPower(-x, r, 0.25);
}

bool DriveRunner::IsFinished() {
  return false;
}

void DriveRunner::End(bool interrupted) {
  // nothing
}
