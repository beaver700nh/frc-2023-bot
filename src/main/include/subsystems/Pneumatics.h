// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>

#include "Constants.h"

using PneuType = frc::PneumaticsModuleType;
using SolenoidValue = frc::DoubleSolenoid::Value;

std::string solenoidValueToString(SolenoidValue value);

class Pneumatics : public frc2::SubsystemBase {
public:
  Pneumatics();

  void AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB);

  void Shoe();
  void Unshoe();
  bool IsShoeDown();

  bool IsHighSpeed();
  void SetGear(bool highGear);
  void SetClaw(bool open);
  void SetIntake(bool forward);
  bool IsIntakeForward();
  void SetLockTilt(bool lock);

  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandXboxController *m_driverControllerA = nullptr;
  frc2::CommandXboxController *m_driverControllerB = nullptr;

  frc::Compressor m_comp {CanIds::kPneuCtrlHub, PneuType::REVPH};

  frc::DoubleSolenoid m_slndShoe {
    CanIds::kPneuCtrlHub, PneuType::REVPH,
    PortsPCH::kPneuSlndShoe1, PortsPCH::kPneuSlndShoe2,
  };
  frc::DoubleSolenoid m_slndClaw {
    CanIds::kPneuCtrlHub, PneuType::REVPH,
    PortsPCH::kPneuSlndClaw1, PortsPCH::kPneuSlndClaw2,
  };
  frc::DoubleSolenoid m_slndGear {
    CanIds::kPneuCtrlHub, PneuType::REVPH,
    PortsPCH::kPneuSlndGear1, PortsPCH::kPneuSlndGear2,
  };

  frc::DoubleSolenoid m_slndIntake {
    CanIds::kPneuCtrlHub, PneuType::REVPH,
    PortsPCH::kPneuSlndIntake1, PortsPCH::kPneuSlndIntake2,
  };

  frc::DoubleSolenoid m_slndLockTilt {
    CanIds::kPneuCtrlHub, PneuType::REVPH,
    PortsPCH::kPneuSlndTiltLock1, PortsPCH::kPneuSlndTiltLock2
  };

  SolenoidValue m_shoeOverride = SolenoidValue::kOff;
  SolenoidValue m_shoeValue = SolenoidValue::kReverse;

  void HandleShoe();
  void HandleClaw();
  void HandleGear();
};
