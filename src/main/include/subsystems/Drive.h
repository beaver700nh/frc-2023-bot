// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/motorcontrol/MotorController.h>

struct Motor {
  frc::MotorController *ctrl;
  bool invert;
};

struct DriveConfig {
  DriveConfig(Motor motorL, Motor motorR, double rampX, double rampR);

  Motor motorL, motorR;
  const double rampX, rampR;
};

class Drive : public frc2::SubsystemBase {
public:
  Drive(DriveConfig config);

  /**
   * Example command factory method.
   */
  void SetPower(double x, double r, double k);

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  DriveConfig config;

  double curX = 0, curR = 0;
};
