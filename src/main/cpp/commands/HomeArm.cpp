// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HomeArm.h"

HomeArmTilt::HomeArmTilt(Arm *arm)
  : m_arm(arm) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void HomeArmTilt::Initialize() {
  m_arm->m_tilt.motor.Set(-0.2);
}

void HomeArmTilt::Execute() {
  // empty
}

void HomeArmTilt::End(bool interrupted) {
  m_arm->m_tilt.Reset();
}

bool HomeArmTilt::IsFinished() {
  return !m_arm->m_tilt.lmsw.Get();
}

HomeArmRotate::HomeArmRotate(Arm *arm)
  : m_arm(arm) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void HomeArmRotate::Initialize() {
  m_arm->m_rotate.motor.Set(-0.2);
}

void HomeArmRotate::Execute() {
  // empty
}

void HomeArmRotate::End(bool interrupted) {
  m_arm->m_rotate.Reset(-13.0);
}

bool HomeArmRotate::IsFinished() {
  return !m_arm->m_rotate.lmsw.Get();
}

HomeArmExtend::HomeArmExtend(Arm *arm)
  : m_arm(arm) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void HomeArmExtend::Initialize() {
  m_arm->m_extend.motor.Set(-0.2);
}

void HomeArmExtend::Execute() {
  // empty
}

void HomeArmExtend::End(bool interrupted) {
  m_arm->m_extend.Reset();
}

bool HomeArmExtend::IsFinished() {
  return !m_arm->m_extend.lmsw.Get();
}
