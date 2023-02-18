// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm(bool invert_tilt, bool invert_rotate, bool invert_extend) {
  m_motor_tilt  .SetInverted(invert_tilt  );
  m_motor_rotate.SetInverted(invert_rotate);
  m_motor_extend.SetInverted(invert_extend);
}

void Arm::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

void Arm::SetTilt(double x, double k) {
  m_motor_tilt
}

void Arm::Periodic() {
  const auto x = Util::thresholded(m_driverController->GetLeftY(), 0.1, -0.1);
  const auto r = Util::thresholded(m_driverController->GetRightX(), 0.1, -0.1);

  // x is negative because joystick y-axis is inverted
  SetPower(-x, r, 0.25);
}
