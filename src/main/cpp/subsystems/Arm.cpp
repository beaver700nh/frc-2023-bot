// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Arm.h"

#include "Util.h"

Arm::Arm(
  bool invertTilt, bool invertRotate, bool invertExtend,
  double rampTilt, double rampRotate, double rampExtend
) : m_rampTilt(rampTilt), m_rampRotate(rampRotate), m_rampExtend(rampExtend) {
  m_motorTilt  .SetInverted(invertTilt  );
  m_motorRotate.SetInverted(invertRotate);
  m_motorExtend.SetInverted(invertExtend);

  m_encoderTilt.SetPosition(0.0);
  m_pidCtrlTilt.SetReference(0.0, SparkMaxCtrlType::kPosition);
  m_pidCtrlTilt.SetP(1.0e-1);
  m_pidCtrlTilt.SetI(1.0e-4);
  m_pidCtrlTilt.SetD(1.0e0);
  m_pidCtrlTilt.SetIZone(0);
  m_pidCtrlTilt.SetFF(0);
  m_pidCtrlTilt.SetOutputRange(-0.5, 0.5);
}

void Arm::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

void Arm::AttachPneumatics(Pneumatics *pneumatics) {
  m_pneumatics = pneumatics;
}

rev::SparkMaxPIDController *Arm::GetPIDCtrlTilt() {
  return &m_pidCtrlTilt;
}

rev::SparkMaxRelativeEncoder *Arm::GetEncoderTilt() {
  return &m_encoderTilt;
}

MotorArm *Arm::GetMotorTilt() {
  return &m_motorTilt;
}

void Arm::StopTilt() {
  m_motorTilt.Set(0.0);
}

void Arm::SetTilt(double x) {
  if (!m_pneumatics) {
    std::cerr << "ERROR in Arm: pneumatics is null." << std::endl;
    return;
  }

  // if (x == 0) { // assumes x is thresholded
  //   m_pneumatics->Shoe();
  // }
  // else if (m_pneumatics->IsShoeDown()) {
  //   m_pneumatics->Unshoe();
  // }

  double constrained = Util::constrained(x, 0.0, Arm::kMaxTilt);
  m_pidCtrlTilt.SetReference(constrained, SparkMaxCtrlType::kPosition);
  frc::SmartDashboard::PutNumber("setPoint", constrained);
}

void Arm::SetRotate(double x, double k) {
  Util::ramp(&m_curRotate, x * k, m_rampRotate);
  m_motorRotate.Set(m_curRotate);
}

void Arm::SetExtend(double x, double k) {
  Util::ramp(&m_curExtend, x * k, m_rampExtend);

  if (!m_lmswExtend.Get() && x < 0) {
    m_motorExtend.Set(0);
  }
  else {
    m_motorExtend.Set(m_curExtend);
  }
}

void Arm::Periodic() {
  if (!m_driverController) {
    std::cerr << "ERROR in Arm: driverController is null." << std::endl;
    return;
  }

  const double tilt = Util::thresholded(m_driverController->GetRightY(), -0.1, 0.1);
  if (tilt != 0) {
    double curPos = m_encoderTilt.GetPosition();
    SetTilt(curPos - 2 * tilt);
  }

  const double rotate = Util::thresholded(m_driverController->GetRightX(), -0.1, 0.1);
  SetRotate(rotate, Arm::kCoeffRotate);

  const double temp = m_driverController->GetRightTriggerAxis() - m_driverController->GetLeftTriggerAxis();
  const double extend = Util::thresholded(temp, -0.1, 0.1);
  SetExtend(extend, Arm::kCoeffExtend);

  frc::SmartDashboard::PutBoolean("lmswTilt",   m_lmswTilt  .Get());
  frc::SmartDashboard::PutBoolean("lmswRotate", m_lmswRotate.Get());
  frc::SmartDashboard::PutBoolean("lmswExtend", m_lmswExtend.Get());

  frc::SmartDashboard::PutNumber("tiltPos", m_encoderTilt.GetPosition());
}
