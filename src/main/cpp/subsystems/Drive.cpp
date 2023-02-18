// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

#include "Util.h"

Drive::Drive(bool invertL, bool invertR, double rampX, double rampR)
  : m_rampX(rampX), m_rampR(rampR) {
  m_ctrlL.SetInverted(invertL);
  m_ctrlR.SetInverted(invertR);
}

void Drive::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

void Drive::SetPower(double x, double r, double k) {
  Util::ramp(&m_curX, x * k, m_rampX);
  Util::ramp(&m_curR, r * k, m_rampR);

  m_ctrlL.Set(m_curX - m_curR);
  m_ctrlR.Set(m_curX + m_curR);
}

void Drive::Periodic() {
  const auto x = Util::thresholded(m_driverController->GetLeftY(), 0.1, -0.1);
  const auto r = Util::thresholded(m_driverController->GetLeftX(), 0.1, -0.1);

  // x is negative because joystick y-axis is inverted
  SetPower(-x, r, Drive::kCoeffDriveTrain);
}
