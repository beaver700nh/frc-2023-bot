#include <iostream>

#include "Movement.h"


frc2::CommandPtr Movement::GenerateCommand(Drive *drive, frc::Pose2d start, std::vector<frc::Translation2d> waypoints, frc::Pose2d end, bool reversed, units::meters_per_second_t maxSpeed) {
  std::cout << "get traj\n";
  
  frc::TrajectoryConfig config {
    maxSpeed,
    DriveConstants::kMaxAcceleration
  };

  config.SetKinematics(kinematics);
  config.AddConstraint(voltageConstraint);
  config.SetReversed(reversed);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    start, waypoints, end, config
  );

  frc::RamseteController ramseteController {
    DriveConstants::kRamseteB, DriveConstants::kRamseteZeta
  };

  ramseteController.SetTolerance({0.25_m, 0.25_m, 5_deg});

  frc2::RamseteCommand command {
    trajectory,
    [drive] { return drive->GetPosition(); },
    ramseteController,
    feedForward, kinematics,
    [drive] { return drive->GetWheelSpeeds(); },
    frc2::PIDController(DriveConstants::kDriveTrajectoryP, 0, 0),
    frc2::PIDController(DriveConstants::kDriveTrajectoryP, 0, 0),
    [drive](auto left, auto right) { drive->SetVolts(left, right); },
    {drive}
  };

  return std::move(command).BeforeStarting(
    [drive] { 
      drive->SetVolts(0_V, 0_V); 
      wasControlled = drive->m_controllerControllable;
      drive->m_controllerControllable = false;
    }
  ).AndThen(
    [drive]{
      drive->m_controllerControllable = wasControlled;
      drive->ResetOdometry();
    }
  );
}

frc2::CommandPtr Movement::GenerateCommand(Drive *drive, std::vector<frc::Translation2d> waypoints, frc::Pose2d end, bool reversed, units::meters_per_second_t maxSpeed) {
  return GenerateCommand(drive, drive->kStartPos, {}, end, reversed, maxSpeed);
}

frc2::CommandPtr Movement::GenerateCommand(Drive *drive, frc::Pose2d end, bool reversed, units::meters_per_second_t maxSpeed) {
  return GenerateCommand(drive, {}, end, reversed, maxSpeed);
}
