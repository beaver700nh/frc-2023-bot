// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HomeArm.h"

HomeArm::HomeArm(Arm *arm)
  : m_arm(arm) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void HomeArm::Initialize() {
  m_arm->GetMotorTilt()->Set(-0.2);
}

void HomeArm::Execute() {
  // empty
}

void HomeArm::End(bool interrupted) {
  m_arm->StopTilt();
  m_arm->GetEncoderTilt()->SetPosition(0.0);
  m_arm->GetPIDCtrlTilt()->SetReference(0.0, SparkMaxCtrlType::kPosition);
}

bool HomeArm::IsFinished() {
  return !m_arm->m_lmswTilt.Get();
}
