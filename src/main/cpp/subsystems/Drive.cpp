// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

#include "Util.h"

Drive::Drive(DriveConfig config)
  : config(config) {
  // Implementation of subsystem constructor goes here.
}

#include <iostream>

void Drive::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

void Drive::SetPower(double x, double r, double k) {
  Util::ramp(&curX, x * k, config.rampX);
  Util::ramp(&curR, r * k, config.rampR);

  config.motorL.ctrl->Set(curX - curR);
  config.motorR.ctrl->Set(curX + curR);
}

void Drive::Periodic() {
  const auto x = Util::thresholded(m_driverController->GetLeftY(), 0.1, -0.1);
  const auto r = Util::thresholded(m_driverController->GetRightX(), 0.1, -0.1);

  // x is negative because joystick y-axis is inverted
  SetPower(-x, r, 0.25);
}

DriveConfig::DriveConfig(Motor _motorL, Motor _motorR, double _rampX, double _rampR)
  : motorL(_motorL), motorR(_motorR), rampX(_rampX), rampR(_rampR) {
  motorL.ctrl->SetInverted(motorL.invert);
  motorR.ctrl->SetInverted(motorR.invert);
}
