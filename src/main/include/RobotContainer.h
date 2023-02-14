// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/motorcontrol/MotorControllerGroup.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#include "Constants.h"

#include "subsystems/Drive.h"
#include "subsystems/ExampleSubsystem.h"

#include "commands/Autos.h"
#include "commands/DriveRunner.h"
#include "commands/ExampleCommand.h"

using Driver = ctre::phoenix::motorcontrol::can::WPI_TalonFX;
using MotorCollection = std::vector<std::reference_wrapper<frc::MotorController>>;

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
  frc2::CommandXboxController m_driverController {OperatorConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;

  MotorCollection m_motors_l {*(frc::MotorController*) new Driver {0}, *(frc::MotorController*) new Driver {1}};
  MotorCollection m_motors_r {*(frc::MotorController*) new Driver {2}, *(frc::MotorController*) new Driver {3}};

  Drive m_drive {
    DriveConfig {
      Motor {new frc::MotorControllerGroup(std::move(m_motors_l)), false},
      Motor {new frc::MotorControllerGroup(std::move(m_motors_r)), true},
      0.04, 0.04
    }
  };

  // The robot's commands are defined here...
  DriveRunner driveRunner {&m_drive, &m_driverController};

  void ConfigureBindings();
};
