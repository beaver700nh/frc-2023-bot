// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Arm;

struct ArmPosition {
public:
 ArmPosition(std::optional<double> tilt, std::optional<double> rotate, std::optional<double> extend);

 const std::optional<double> tilt, rotate, extend; 

 void SetArmToPosition(Arm *arm);
};

class SetArmPosition : public frc2::CommandHelper<frc2::CommandBase, SetArmPosition> {
public:
  
  explicit SetArmPosition(Arm *arm, ArmPosition position);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

protected:
  Arm *m_arm;
  ArmPosition m_position;
};

class SetArmPositionWait : public SetArmPosition {
public:
  explicit SetArmPositionWait(Arm *arm, ArmPosition position);

  bool IsFinished() override;
};

class SetArmPositionEx : public frc2::CommandHelper<frc2::CommandBase, SetArmPositionEx>{
public:
  explicit SetArmPositionEx(Arm *arm, std::vector<ArmPosition> position);

  void DoPosition();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

protected:
  Arm *m_arm;
  std::vector<ArmPosition> m_positions;
  unsigned int nextPosition = 0;
};

class SetArmPositionExWait : public SetArmPositionEx {
public:
  using SetArmPositionEx::SetArmPositionEx;

  void Execute() override;
  bool IsFinished() override;
};