// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Pneumatics.h"

#include "Constants.h"
#include "Util.h"

Pneumatics::Pneumatics() {
  m_slndShoe.Set(SolenoidValue::kReverse);
  m_slndClaw.Set(SolenoidValue::kReverse);
  m_slndGear.Set(SolenoidValue::kForward);
}

void Pneumatics::AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB) {
  m_driverControllerA = driverControllerA;
  m_driverControllerB = driverControllerB;
}

void Pneumatics::Shoe() {
  if (m_shoeOverride == SolenoidValue::kOff) {
    m_shoeValue = SolenoidValue::kForward;
  }
}

void Pneumatics::Unshoe() {
  if (m_shoeOverride == SolenoidValue::kOff) {
    m_shoeValue = SolenoidValue::kReverse;
  }
}

bool Pneumatics::IsShoeDown() {
  return m_slndShoe.Get() == SolenoidValue::kForward;
}

// High gear is the slow speed (based on gear ratio)
bool Pneumatics::IsHighGear() {
  return m_slndGear.Get() == SolenoidValue::kForward;
}

void Pneumatics::SetGear(bool highGear){
  return m_slndGear.Set(highGear ? SolenoidValue::kReverse : SolenoidValue::kForward);
}

void Pneumatics::Periodic() {
  HandleShoe();
  HandleClaw();
  // HandleGear();
}

void Pneumatics::HandleShoe() {
  // if (m_driverControllerA->GetPOV() == POV_UP) {
  //   m_shoeOverride = SolenoidValue::kOff;
  // }
  // else if (m_driverControllerA->GetPOV() == POV_LEFT) {
  //   m_shoeOverride = SolenoidValue::kForward;
  // }
  // else if (m_driverControllerA->GetPOV() == POV_RIGHT) {
  //   m_shoeOverride = SolenoidValue::kReverse;
  // }

  if (m_shoeOverride == SolenoidValue::kOff) {
    m_slndShoe.Set(m_shoeValue);
    frc::SmartDashboard::PutString("Pneu. Shoe", "normal " + solenoidValueToString(m_shoeValue));
  }
  else {
    m_slndShoe.Set(m_shoeOverride);
    frc::SmartDashboard::PutString("Pneu. Shoe", "override " + solenoidValueToString(m_shoeOverride));
  }
}

void Pneumatics::HandleClaw() {
  if (m_driverControllerB->GetLeftBumperPressed()) {
    m_slndClaw.Set(SolenoidValue::kForward);
    frc::SmartDashboard::PutString("Pneu. Claw", "open");
  }
  else if (m_driverControllerB->GetRightBumperPressed()) {
    m_slndClaw.Set(SolenoidValue::kReverse);
    frc::SmartDashboard::PutString("Pneu. Claw", "grab");
  }
}

void Pneumatics::HandleGear() {
  if (m_driverControllerA->GetLeftBumperPressed()) {
    m_slndGear.Set(SolenoidValue::kForward);
    frc::SmartDashboard::PutString("Pneu. Gear", "slow");
  }
  else if (m_driverControllerA->GetRightBumperPressed()) {
    m_slndGear.Set(SolenoidValue::kReverse);
    frc::SmartDashboard::PutString("Pneu. Gear", "fast");
  }
}

std::string solenoidValueToString(SolenoidValue value) {
  switch (value) {
  case SolenoidValue::kOff:     return "off";
  case SolenoidValue::kForward: return "forward";
  case SolenoidValue::kReverse: return "reverse";
  default:                      return "error";
  }
}
