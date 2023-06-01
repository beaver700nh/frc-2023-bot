// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "commands/SetArmPosition.h"

#include "subsystems/Arm.h"

SetArmPosition::SetArmPosition(Arm *arm, ArmPosition position)
  : m_arm(arm), m_position(position){
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void SetArmPosition::Initialize() {
  m_position.SetArmToPosition(m_arm);
}

void SetArmPosition::Execute() {
  // Empty
}

void SetArmPosition::End(bool interrupted) {
  // Empty
}

bool SetArmPosition::IsFinished() {
  return true;
}

SetArmPositionWait::SetArmPositionWait(Arm *arm, ArmPosition position)
  : m_arm(arm), m_position(position){
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void SetArmPositionWait::Initialize() {
  m_position.SetArmToPosition(m_arm);
}

void SetArmPositionWait::Execute() {
  // Empty
}

void SetArmPositionWait::End(bool interrupted) {
  // Empty
}

bool SetArmPositionWait::IsFinished() {
  return m_arm->InTolerance();
}

SetArmPositionEx::SetArmPositionEx(Arm *arm, std::vector<ArmPosition> positions)
  :m_arm(arm), m_positions(positions) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void SetArmPositionEx::DoPosition() {
  m_positions.at(nextPosition++).SetArmToPosition(m_arm);
}

void SetArmPositionEx::Initialize() {
  DoPosition();
}

void SetArmPositionEx::Execute() {
  if (m_arm->InTolerance()){
    DoPosition();
  }
}

void SetArmPositionEx::End(bool interrupted) {
  // Empty
}

bool SetArmPositionEx::IsFinished() {
  return nextPosition >= m_positions.size();
}


SetArmPositionExWait::SetArmPositionExWait(Arm *arm, std::vector<ArmPosition> positions)
  :m_arm(arm), m_positions(positions) {
  // Register that this command requires the subsystem.
  AddRequirements(m_arm);
}

void SetArmPositionExWait::DoPosition() {
  m_positions.at(nextPosition++).SetArmToPosition(m_arm);
}

void SetArmPositionExWait::Initialize() {
  DoPosition();
}


void SetArmPositionExWait::Execute() {
  if (m_arm->InTolerance()) {
    if (nextPosition < m_positions.size()) {
      m_positions.at(nextPosition).SetArmToPosition(m_arm);
    }
    ++nextPosition;
  }
}

void SetArmPositionExWait::End(bool interrupted) {
  // Empty
}

bool SetArmPositionExWait::IsFinished() {
  return nextPosition > m_positions.size();
}

SetArmPositionCurrent::SetArmPositionCurrent(Arm *arm) : m_arm(arm){
  AddRequirements(m_arm);
}

void SetArmPositionCurrent::Initialize(){
  m_arm->m_tilt.SetAbsolute(m_arm->m_tilt.encoder.GetPosition());
  m_arm->m_rotate.SetAbsolute(m_arm->m_rotate.encoder.GetPosition());
  m_arm->m_extend.SetAbsolute(m_arm->m_extend.encoder.GetPosition());
}

void SetArmPositionCurrent::Execute(){
  // empty
}

void SetArmPositionCurrent::End(bool interrupted){
  //empty
}

bool SetArmPositionCurrent::IsFinished(){
  return true;
}

ArmPosition::ArmPosition(std::optional<double> tilt, std::optional<double> rotate, std::optional<double> extend)
  : tilt(tilt), rotate(rotate), extend(extend) {
  // Empty
}

void ArmPosition::SetArmToPosition(Arm *arm){
  if (tilt.has_value()) {
    arm->m_tilt.SetAbsolute(tilt.value());
  }

  if (rotate.has_value()) {
    arm->m_rotate.SetAbsolute(rotate.value());
  }

  if (extend.has_value()) {
    arm->m_extend.SetAbsolute(extend.value());
  }
}

