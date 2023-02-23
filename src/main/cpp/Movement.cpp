// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "Movement.h"

Movement::Movement(Drive *drive) : m_drive(drive) {
  std::cerr << "A\n";

  if (!trajectoryConfigInitialized) {
    trajectoryConfig.SetKinematics(kinematics);
    trajectoryConfig.AddConstraint(voltageConstraint);
    trajectoryConfigInitialized = true;
  }

  std::cerr << "B\n";

  if (!ramseteControllerInitialized) {
    ramseteController.SetTolerance({1.0_m, 1.0_m, 5.0_deg});
    ramseteControllerInitialized = true;
  }

  std::cerr << "C\n";
}

frc2::CommandPtr Movement::GenerateCommand(std::vector<frc::Translation2d> waypoints, frc::Pose2d end) {
  std::cerr << "1\n";

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(m_drive->GetPosition(), waypoints, end, trajectoryConfig);

  std::cerr << "2\n";

  frc2::RamseteCommand command {
    trajectory,
    std::bind(&Drive::GetPosition, m_drive),
    ramseteController, feedForward, kinematics,
    std::bind(&Drive::GetWheelSpeeds, m_drive),
    frc2::PIDController(DriveConstants::kDriveTrajectoryP, 0, 0),
    frc2::PIDController(DriveConstants::kDriveTrajectoryP, 0, 0),
    [this](auto left, auto right) { m_drive->SetVolts(right, left); },
    {m_drive}
  };

  std::cerr << "3\n";

  return std::move(command).BeforeStarting(
    [this] { 
      m_drive->SetVolts(0_V, 0_V); 
      m_wasControllerControlled = m_drive->m_controllerControllable;
      m_drive->m_controllerControllable = false;
      std::cerr << "X\n";
    }
  ).AndThen(
    [this] {
      m_drive->m_controllerControllable = m_wasControllerControlled;
      std::cerr << "Y\n";
    }
  );
}

frc2::CommandPtr Movement::GenerateCommand(frc::Pose2d end) {
  return GenerateCommand({}, end);
}
