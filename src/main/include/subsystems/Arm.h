// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/DigitalInput.h>

#include <rev/SparkMaxPIDController.h>
#include <rev/CANSparkMax.h>

#include "subsystems/Pneumatics.h"

#include "Constants.h"

using MotorArm = rev::CANSparkMax;
using MotorArmType = rev::CANSparkMaxLowLevel::MotorType;
using SparkMaxCtrlType = rev::CANSparkMax::ControlType;

struct ArmComponent {
public:
  ArmComponent(int motorCanId, int lmswPort, double coeff, double maxPos);

  void Initialize(bool invert, double p, double i, double d, double iz, double ff, double min, double max);
  void Set(double rawControllerInput, bool inverted);

  MotorArm motor;
  rev::SparkMaxRelativeEncoder encoder;
  rev::SparkMaxPIDController pidCtrl;
  frc::DigitalInput lmsw;

  const double m_coeff;
  const double m_maxPos;
};

class Arm : public frc2::SubsystemBase {
public:
  Arm(bool invertTilt, bool invertRotate, bool invertExtend);

  void AttachController(frc2::CommandXboxController *driverController);
  void AttachPneumatics(Pneumatics *pneumatics);

  void Periodic() override;

  ArmComponent m_tilt   {CanIds::kArmTilt,   PortsDIO::kArmLmswTilt,   2.5, 160.0};
  ArmComponent m_rotate {CanIds::kArmRotate, PortsDIO::kArmLmswRotate, 1.5, 50.0};
  ArmComponent m_extend {CanIds::kArmExtend, PortsDIO::kArmLmswExtend, 1.5, 50.0};

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandXboxController *m_driverController = nullptr;

  Pneumatics *m_pneumatics = nullptr;
};
