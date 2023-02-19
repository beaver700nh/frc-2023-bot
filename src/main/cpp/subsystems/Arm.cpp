// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Arm.h"

#include "Util.h"

Arm::Arm(bool invertTilt, bool invertRotate, bool invertExtend) {
  m_tilt  .Initialize(invertTilt,   1.0e-1, 1.0e-4, 1.0e+0, 0.0, 0.0, -0.5, 0.5);
  m_rotate.Initialize(invertRotate, 1.0e-1, 1.0e-4, 1.0e+0, 0.0, 0.0, -0.5, 0.5);
  m_extend.Initialize(invertExtend, 1.0e-1, 1.0e-4, 1.0e+0, 0.0, 0.0, -0.5, 0.5);
}

void Arm::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

void Arm::AttachPneumatics(Pneumatics *pneumatics) {
  m_pneumatics = pneumatics;
}

void Arm::Periodic() {
  if (!m_driverController) {
    std::cerr << "ERROR in Arm: driverController is null." << std::endl;
    return;
  }

  const double tilt = m_driverController->GetRightY();
  m_tilt.Set(tilt, true);

  const double rotate = m_driverController->GetRightX();
  m_rotate.Set(rotate, false);

  const double extend = m_driverController->GetRightTriggerAxis() - m_driverController->GetLeftTriggerAxis();
  m_extend.Set(extend, false);
}

ArmComponent::ArmComponent(int motorCanId, int lmswPort, double coeff, double maxPos) :
  motor(motorCanId, MotorArmType::kBrushless), encoder(motor.GetEncoder()), pidCtrl(motor.GetPIDController()),
  lmsw(lmswPort), m_coeff(coeff), m_maxPos(maxPos) {
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

void ArmComponent::Set(double rawControllerInput, bool inverted) {
  const double thresholded = Util::thresholded(rawControllerInput, -0.06, 0.06);

  if (thresholded == 0) {
    return;
  }

  const double position = encoder.GetPosition() + m_coeff * thresholded * (inverted ? -1.0 : 1.0);
  const double constrained = Util::constrained(position, 0.0, m_maxPos);

  pidCtrl.SetReference(constrained, SparkMaxCtrlType::kPosition);
}
