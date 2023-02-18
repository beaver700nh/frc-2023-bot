// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/Pneumatics.h"

#include "Util.h"

Pneumatics::Pneumatics() {
  m_slnd_shoe.Set(SolenoidValue::kReverse);
  m_slnd_claw.Set(SolenoidValue::kForward);
}

void Pneumatics::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

bool Pneumatics::IsShoeDown() {
  return m_slnd_shoe.Get() == SolenoidValue::kForward;
}

void Pneumatics::Periodic() {
  if (!m_driverController) {
    std::cerr << "ERROR in Pneumatics: driverController is null." << std::endl;
    return;
  }

  if (m_driverController->GetAButtonPressed()) {
    m_slnd_shoe.Toggle();
  }

  if (m_driverController->GetBButtonPressed()) {
    m_slnd_claw.Toggle();
  }

  if (m_driverController->GetLeftBumperPressed()) {
    m_slnd_gear.Set(SolenoidValue::kForward);
  }
  else if (m_driverController->GetRightBumperPressed()) {
    m_slnd_gear.Set(SolenoidValue::kReverse);
  }
}
