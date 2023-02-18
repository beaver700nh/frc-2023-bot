// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include <frc2/command/CommandPtr.h>

#include <frc/Solenoid.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

#include "subsystems/Drive.h"
#include "subsystems/ExampleSubsystem.h"

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

using MotorDriver = ctre::phoenix::motorcontrol::can::WPI_TalonFX;
using MotorArm = rev::CANSparkMax;
using MotorArmType = rev::CANSparkMaxLowLevel::MotorType;
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

  MotorDriver m_motors[4] {{0}, {1}, {2}, {3}};

#define MOTOR(n) (*(frc::MotorController *) (m_motors + (n)))
  MotorCollection m_motors_l {MOTOR(0), MOTOR(1)};
  MotorCollection m_motors_r {MOTOR(2), MOTOR(3)};
#undef MOTOR

  MotorArm m_arm_tilt   {6, MotorArmType::kBrushless};
  MotorArm m_arm_rotate {7, MotorArmType::kBrushless};
  MotorArm m_arm_extend {8, MotorArmType::kBrushless};

  Drive m_drive {
    DriveConfig {
      Motor {new frc::MotorControllerGroup(std::move(m_motors_l)), false},
      Motor {new frc::MotorControllerGroup(std::move(m_motors_r)), true},
      0.04, 0.04
    }
  };

  void ConfigureBindings();
};
