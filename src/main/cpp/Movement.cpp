#include "Movement.h"

frc2::CommandPtr Movement::GenerateCommand(Drive *drive, std::vector<frc::Translation2d> waypoints, frc::Pose2d end) {
  frc::TrajectoryConfig config {
    DriveConstants::kMaxSpeed,
    DriveConstants::kMaxAcceleration
  };

  config.SetKinematics(kinematics);
  config.AddConstraint(voltageConstraint);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    drive->GetPosition(), waypoints, end, config
  );

  frc2::RamseteCommand command {
    trajectory,
    [drive] { return drive->GetPosition(); },
    frc::RamseteController(DriveConstants::kRamseteB, DriveConstants::kRamseteZeta),
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

frc2::CommandPtr Movement::GenerateCommand(Drive *drive, frc::Pose2d end) {
  return GenerateCommand(drive, {}, end);
}
