// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>

#include <rev/CANSparkMax.h>

using MotorArm = rev::CANSparkMax;
using MotorArmType = rev::CANSparkMaxLowLevel::MotorType;

class Arm : public frc2::SubsystemBase {
public:
  Arm(bool invert_tilt, bool invert_rotate, bool invert_extend);

  void AttachController(frc2::CommandXboxController *driverController);

  void SetTilt(double x, double k);
  void SetRotate(double x, double k);
  void SetExtend(double x, double k);

  void Periodic();

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandXboxController *m_driverController;

  MotorArm m_motor_tilt   {6, MotorArmType::kBrushless};
  MotorArm m_motor_rotate {7, MotorArmType::kBrushless};
  MotorArm m_motor_extend {8, MotorArmType::kBrushless};
};
