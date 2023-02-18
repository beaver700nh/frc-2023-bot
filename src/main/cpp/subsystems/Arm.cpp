// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

#include "Util.h"

Arm::Arm(bool invertTilt, bool invertRotate, bool invertExtend) {
  m_motorTilt  .SetInverted(invertTilt  );
  m_motorRotate.SetInverted(invertRotate);
  m_motorExtend.SetInverted(invertExtend);
}

void Arm::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

void Arm::SetTilt(double x, double k) {
  m_motorTilt.Set(x * k);
}

void Arm::SetRotate(double x, double k) {
  m_motorRotate.Set(x * k);
}

void Arm::SetExtend(double x, double k) {
  m_motorExtend.Set(x * k);
}

void Arm::Periodic() {
  const double tilt = Util::thresholded(m_driverController->GetRightY(), 0.1, -0.1);
  SetTilt(-tilt, Arm::kTiltPower); // tilt is negative because joystick y-axis is inverted

  const double rotate = Util::thresholded(m_driverController->GetRightX(), 0.1, -0.1);
  SetRotate(rotate, Arm::kRotatePower);

  const double temp = m_driverController->GetLeftTriggerAxis() - m_driverController->GetRightTriggerAxis();
  const double extend = Util::thresholded(temp, 0.1, -0.1);
  SetExtend(extend, Arm::kExtendPower);
}
