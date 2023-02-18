// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

#include "Util.h"

Arm::Arm(
  bool invertTilt, bool invertRotate, bool invertExtend,
  double rampTilt, double rampRotate, double rampExtend
) : m_rampTilt(rampTilt), m_rampRotate(rampRotate), m_rampExtend(rampExtend) {
  m_motorTilt  .SetInverted(invertTilt  );
  m_motorRotate.SetInverted(invertRotate);
  m_motorExtend.SetInverted(invertExtend);
}

void Arm::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

void Arm::SetTilt(double x, double k) {
  Util::ramp(&m_curTilt, x * k, m_rampTilt);
  m_motorTilt.Set(m_curTilt);
}

void Arm::SetRotate(double x, double k) {
  Util::ramp(&m_curRotate, x * k, m_rampRotate);
  m_motorRotate.Set(m_curRotate);
}

void Arm::SetExtend(double x, double k) {
  Util::ramp(&m_curExtend, x * k, m_curExtend);
  m_motorExtend.Set(m_curExtend);
}

void Arm::Periodic() {
  const double tilt = Util::thresholded(m_driverController->GetRightY(), 0.1, -0.1);
  SetTilt(-tilt, Arm::kCoeffTilt); // tilt is negative because joystick y-axis is inverted

  const double rotate = Util::thresholded(m_driverController->GetRightX(), 0.1, -0.1);
  SetRotate(rotate, Arm::kCoeffRotate);

  const double temp = m_driverController->GetRightTriggerAxis() - m_driverController->GetLeftTriggerAxis();
  const double extend = Util::thresholded(temp, 0.1, -0.1);
  SetExtend(extend, Arm::kCoeffExtend);
}
