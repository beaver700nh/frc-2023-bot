// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HomeArm.h"

#define tilt m_arm->m_tilt
#define rotate m_arm->m_rotate
#define extend m_arm->m_extend

HomeArmTilt::HomeArmTilt(Arm *arm) : m_arm(arm) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void HomeArmTilt::Initialize() {
  tilt.motor.Set(-0.2);
}

void HomeArmTilt::Execute() {
  // empty
}

void HomeArmTilt::End(bool interrupted) {
  if(interrupted){
    tilt.Reset((tilt.maxPos - tilt.minPos) / 2.0);
  } else {
    tilt.Reset();
  }
}

bool HomeArmTilt::IsFinished() {
  return !m_arm->m_tilt.lmsw.Get() || m_arm->m_driverControllerB->GetBButtonPressed();
}

HomeArmRotate::HomeArmRotate(Arm *arm)
  : m_arm(arm) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void HomeArmRotate::Initialize() {
  rotate.motor.Set(-0.3);
}

void HomeArmRotate::Execute() {
  // empty
}

void HomeArmRotate::End(bool interrupted) {
  if(interrupted){
    rotate.Reset((rotate.maxPos - rotate.minPos) / 2);
  } else {
    rotate.Reset(-13.0);
    rotate.SetAbsolute(0);
  }
}

bool HomeArmRotate::IsFinished() {
  return !rotate.lmsw.Get();
}

HomeArmExtend::HomeArmExtend(Arm *arm)
  : m_arm(arm) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void HomeArmExtend::Initialize() {
  extend.motor.Set(-0.4);
}

void HomeArmExtend::Execute() {
  // empty
}

void HomeArmExtend::End(bool interrupted) {
  if(interrupted){
    extend.Reset((extend.maxPos - extend.minPos) / 2);
  } else {
    extend.Reset();
  }
}

bool HomeArmExtend::IsFinished() {
  return !extend.lmsw.Get();
}
