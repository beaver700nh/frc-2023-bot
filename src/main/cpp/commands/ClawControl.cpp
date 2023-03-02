// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClawControl.h"

ClawControl::ClawControl(Pneumatics *pneu, bool open) : m_pneu(pneu), m_open(open) {
  // Register that this command requires the subsystem.
  AddRequirements(m_pneu);
}

void ClawControl::Initialize() {
  m_pneu->SetClaw(m_open);
}

void ClawControl::Execute() {
  // empty
}

void ClawControl::End(bool interrupted) {
  // empty
}

bool ClawControl::IsFinished() {
  return true;
}
