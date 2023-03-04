// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "Constants.h"

#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/Arm.h"
#include "subsystems/Drive.h"
#include "subsystems/Pneumatics.h"
#include "subsystems/ExampleSubsystem.h"

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverControllerA {OperatorConstants::kDriverControllerAPort};
  frc2::CommandXboxController m_driverControllerB {OperatorConstants::kDriverControllerBPort};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;

  Arm m_arm {
    OperatorConstants::kArmInvertTilt,
    OperatorConstants::kArmInvertRotate,
    OperatorConstants::kArmInvertExtend,
  };

  Drive m_drive {
    OperatorConstants::kDriveInvertL,
    OperatorConstants::kDriveInvertR,
    OperatorConstants::kDriveRampX,
    OperatorConstants::kDriveRampR,
  };

  Pneumatics m_pneu {};

  void ConfigureBindings();

  frc::SendableChooser<int> autoChooser {};

};
