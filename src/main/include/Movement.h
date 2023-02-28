// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>


#include <frc2/command/CommandPtr.h>
#include <frc2/command/RamseteCommand.h>

#include "subsystems/Drive.h"

#include "Constants.h"

class Movement {
public:
  static frc2::CommandPtr GenerateCommand(Drive *drive, frc::Pose2d start, std::vector<frc::Translation2d> waypoints, frc::Pose2d end, bool reversed = false);
  static frc2::CommandPtr GenerateCommand(Drive *drive, std::vector<frc::Translation2d> waypoints, frc::Pose2d end, bool reversed = false);
  static frc2::CommandPtr GenerateCommand(Drive *drive, frc::Pose2d end, bool reversed = false);

  inline static const frc::DifferentialDriveKinematics kinematics {
    DriveConstants::kTrackwidth
  };
  inline static const frc::SimpleMotorFeedforward<units::meters> feedForward {
    DriveConstants::kDriveTrajectoryS,
    DriveConstants::kDriveTrajectoryV,
    DriveConstants::kDriveTrajectoryA
  };
  inline static const frc::DifferentialDriveVoltageConstraint voltageConstraint{
    feedForward, kinematics, DriveConstants::kMaxVoltage
  };

  inline static bool wasControlled = false;
};
