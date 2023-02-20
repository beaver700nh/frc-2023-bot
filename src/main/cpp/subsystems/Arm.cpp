// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Arm.h"

#include "Util.h"

Arm::Arm(bool invertTilt, bool invertRotate, bool invertExtend) {
  m_tilt  .Initialize(invertTilt,   1.0e-1, 1.0e-4, 1.0e+0, 0.0, 0.0, -0.5, 0.5);
  m_rotate.Initialize(invertRotate, 1.0e-1, 1.0e-4, 1.0e+0, 0.0, 0.0, -0.5, 0.5);
  m_extend.Initialize(invertExtend, 1.0e-1, 1.0e-4, 1.0e+0, 0.0, 0.0, -0.5, 0.5);
}

void Arm::AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB) {
  m_driverControllerA = driverControllerA;
  m_driverControllerB = driverControllerB;
}

void Arm::AttachPneumatics(Pneumatics *pneumatics) {
  m_pneumatics = pneumatics;
}

void Arm::Periodic() {
  if (!m_driverControllerA || !m_driverControllerB) {
    std::cerr << "ERROR in Arm: driverController is null." << std::endl;
    return;
  }

  const double tilt = m_driverControllerB->GetLeftY();
  auto info = m_tilt.Set(tilt, true);

  if (info.has_value()) {
    if (std::abs(info.value().position - info.value().constrained) >= 2.2) {
      m_pneumatics->Unshoe();
    }
    else {
      m_pneumatics->Shoe();
    }
  }

  const double rotate = m_driverControllerB->GetRightX();
  m_rotate.Set(rotate, false);

  const double extend = m_driverControllerB->GetRightTriggerAxis() - m_driverControllerB->GetLeftTriggerAxis();
  m_extend.Set(extend, false);
}

ArmComponent::ArmComponent(int motorCanId, int lmswPort, double coeff, double minPos, double maxPos) :
  motor(motorCanId, MotorArmType::kBrushless), encoder(motor.GetEncoder()), pidCtrl(motor.GetPIDController()),
  lmsw(lmswPort), coeff(coeff), minPos(minPos), maxPos(maxPos) {
  // empty
}

void ArmComponent::Initialize(bool invert, double p, double i, double d, double iz, double ff, double min, double max) {
  motor.SetInverted(invert);

  encoder.SetPosition(0.0);

  pidCtrl.SetReference(0.0, SparkMaxCtrlType::kPosition);
  pidCtrl.SetP(p);
  pidCtrl.SetI(i);
  pidCtrl.SetD(d);
  pidCtrl.SetIZone(iz);
  pidCtrl.SetFF(ff);
  pidCtrl.SetOutputRange(min, max);
}

std::optional<ArmComponent::MoveInfo> ArmComponent::Set(double rawControllerInput, bool inverted) {
  const double thresholded = Util::thresholded(rawControllerInput, -0.08, 0.08);

  const double position = encoder.GetPosition();
  const double adjusted = position + coeff * thresholded * (inverted ? -1.0 : 1.0);
  const double constrained = Util::constrained(adjusted, minPos, maxPos);

  if (thresholded == 0) {
    return std::nullopt;
  }

  pidCtrl.SetReference(constrained, SparkMaxCtrlType::kPosition);

  return MoveInfo {position, adjusted, constrained};
}

void ArmComponent::Reset(double pos) {
  motor.Set(0.0);
  encoder.SetPosition(pos);
  pidCtrl.SetReference(pos, SparkMaxCtrlType::kPosition);
}
