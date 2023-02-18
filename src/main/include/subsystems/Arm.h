// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/DigitalInput.h>

#include <rev/CANSparkMax.h>

#include "subsystems/Pneumatics.h"

#include "Constants.h"

using MotorArm = rev::CANSparkMax;
using MotorArmType = rev::CANSparkMaxLowLevel::MotorType;

class Arm : public frc2::SubsystemBase {
public:
  Arm(
    bool invertTilt, bool invertRotate, bool invertExtend,
    double rampTilt, double rampRotate, double rampExtend
  );

  void AttachController(frc2::CommandXboxController *driverController);
  void AttachPneumatics(Pneumatics *pneumatics);

  void SetTilt  (double x, double k = 1.0);
  void SetRotate(double x, double k = 1.0);
  void SetExtend(double x, double k = 1.0);

  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandXboxController *m_driverController = nullptr;

  Pneumatics *m_pneumatics = nullptr;

  MotorArm m_motorTilt   {CanIds::kArmTilt,   MotorArmType::kBrushless};
  MotorArm m_motorRotate {CanIds::kArmRotate, MotorArmType::kBrushless};
  MotorArm m_motorExtend {CanIds::kArmExtend, MotorArmType::kBrushless};

  frc::DigitalInput m_lmswTilt   {0};
  frc::DigitalInput m_lmswRotate {1};
  frc::DigitalInput m_lmswExtend {2};

  const double m_rampTilt, m_rampRotate, m_rampExtend;

  double m_curTilt = 0.0, m_curRotate = 0.0, m_curExtend = 0.0;

  static constexpr double kCoeffTilt   = 1.0;
  static constexpr double kCoeffRotate = 1.0;
  static constexpr double kCoeffExtend = 1.0;
};
