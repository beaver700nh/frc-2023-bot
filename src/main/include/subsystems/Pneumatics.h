// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>

#include "Constants.h"

using PneuType = frc::PneumaticsModuleType;
using SolenoidValue = frc::DoubleSolenoid::Value;

class Pneumatics : public frc2::SubsystemBase {
public:
  Pneumatics();

  void AttachController(frc2::CommandXboxController *driverController);

  void Shoe();
  void Unshoe();
  bool IsShoeDown();

  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandXboxController *m_driverController = nullptr;

  frc::Compressor m_comp {CanIds::kPneuCtrlHub, PneuType::REVPH};

  frc::DoubleSolenoid m_slndShoe {CanIds::kPneuCtrlHub, PneuType::REVPH, 2, 10};
  frc::DoubleSolenoid m_slndClaw {CanIds::kPneuCtrlHub, PneuType::REVPH, 3, 11};
  frc::DoubleSolenoid m_slndGear {CanIds::kPneuCtrlHub, PneuType::REVPH, 7, 15};

  SolenoidValue m_shoeOverride = SolenoidValue::kOff;
  SolenoidValue m_shoeValue = SolenoidValue::kReverse;

  void HandleShoe();
  void HandleClaw();
  void HandleGear();
};
