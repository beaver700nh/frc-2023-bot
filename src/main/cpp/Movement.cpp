#include "Movement.h"

namespace Movement {

bool wasControlled = false;
frc::Field2d field;

frc2::CommandPtr GenerateCommand(Drive *drive, std::vector<frc::Translation2d> waypoints, frc::Pose2d end) {
  frc::TrajectoryConfig config {
    OperatorConstants::kMaxSpeed,
    OperatorConstants::kMaxAcceleration
  };

  config.SetKinematics(kinematics);
  config.AddConstraint(voltageConstraint);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    drive->GetPosition(), waypoints, end, config
  );

  frc::SmartDashboard::PutData("Field", &field);
  field.SetRobotPose(drive->GetPosition());
  field.GetObject("traj")->SetTrajectory(trajectory);

  frc2::RamseteCommand command {
    trajectory,
    [drive] { return drive->GetPosition(); },
    frc::RamseteController(OperatorConstants::kRamseteB, OperatorConstants::kRamseteZeta),
    feedForward, kinematics,
    [drive] { return drive->GetWheelSpeeds(); },
    frc2::PIDController(OperatorConstants::kDriveTrajectoryP, 0, 0),
    frc2::PIDController(OperatorConstants::kDriveTrajectoryP, 0, 0),
    [drive](auto left, auto right) { 
      drive->SetVolts(right, left);
    },
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
      field.SetRobotPose(drive->GetPosition());
    }
  );
}

frc2::CommandPtr GenerateCommand(Drive *drive, frc::Pose2d end) {
  return GenerateCommand(drive, {}, end);
}

}
