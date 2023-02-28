#include "Movement.h"

frc2::CommandPtr Movement::GenerateCommand(Drive *drive, frc::Pose2d start, std::vector<frc::Translation2d> waypoints, frc::Pose2d end, bool reversed) {
  frc::TrajectoryConfig config {
    DriveConstants::kMaxSpeed,
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
    }
  );
}

frc2::CommandPtr Movement::GenerateCommand(Drive *drive, std::vector<frc::Translation2d> waypoints, frc::Pose2d end, bool reversed) {
  return GenerateCommand(drive, drive->GetPosition(), {}, end, reversed);
}

frc2::CommandPtr Movement::GenerateCommand(Drive *drive, frc::Pose2d end, bool reversed) {
  return GenerateCommand(drive, {}, end, reversed);
}
