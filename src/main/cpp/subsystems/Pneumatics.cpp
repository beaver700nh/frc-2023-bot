// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/Pneumatics.h"

#include "Constants.h"
#include "Util.h"

Pneumatics::Pneumatics() {
  m_slndShoe.Set(SolenoidValue::kReverse);
  m_slndClaw.Set(SolenoidValue::kForward);
}

void Pneumatics::AttachController(frc2::CommandXboxController *driverController) {
  m_driverController = driverController;
}

void Pneumatics::Shoe() {
  if (m_shoeOverride != SolenoidValue::kOff) {
    return;
  }

  m_shoeValue = SolenoidValue::kForward;
}

void Pneumatics::Unshoe() {
  if (m_shoeOverride != SolenoidValue::kOff) {
    return;
  }

  m_shoeValue = SolenoidValue::kReverse;
}

bool Pneumatics::IsShoeDown() {
  return m_slndShoe.Get() == SolenoidValue::kForward;
}

void Pneumatics::Periodic() {
  if (!m_driverController) {
    std::cerr << "ERROR in Pneumatics: driverController is null." << std::endl;
    return;
  }

  HandleShoe();
  HandleClaw();
  HandleGear();
}

void Pneumatics::HandleShoe() {
  if (m_driverController->GetPOV() == POV_UP) {
    m_shoeOverride = SolenoidValue::kOff;
  }
  else if (m_driverController->GetPOV() == POV_LEFT) {
    m_shoeOverride = SolenoidValue::kForward;
  }
  else if (m_driverController->GetPOV() == POV_RIGHT) {
    m_shoeOverride = SolenoidValue::kReverse;
  }

  if (m_shoeOverride == SolenoidValue::kOff) {
    m_slndShoe.Set(m_shoeValue);
  }
  else {
    m_slndShoe.Set(m_shoeOverride);
  }
}

void Pneumatics::HandleClaw() {
  if (m_driverController->GetAButtonPressed()) {
    m_slndClaw.Set(SolenoidValue::kForward);
  }
  else if (m_driverController->GetBButtonPressed()) {
    m_slndClaw.Set(SolenoidValue::kReverse);
  }
}

void Pneumatics::HandleGear() {
  if (m_driverController->GetLeftBumperPressed()) {
    m_slndGear.Set(SolenoidValue::kForward);
  }
  else if (m_driverController->GetRightBumperPressed()) {
    m_slndGear.Set(SolenoidValue::kReverse);
  }
}
