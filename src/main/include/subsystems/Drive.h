// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <vector>
#include <string>
#include <sstream>

#include <units/length.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/motorcontrol/MotorController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#include "subsystems/Pneumatics.h"

#include "Constants.h"

using MotorDriver = ctre::phoenix::motorcontrol::can::WPI_TalonFX;

struct Gains {
  const double p;
  const double i;
  const double d;
  const double f;
  const int iZone;
  const double peakOutput;
};

struct WheelOdometryInfo {
  double lastPosition = 0.0; // in encoder ticks
  units::meter_t distance = 0.0_m;
  units::meters_per_second_t velocity = 0.0_mps;

  void Calculate(double curPosition, double curVelocity, units::meter_t tickToMeterFactor);
  void Reset(double encoderPosition = 0.0);
  std::string Stringify();
};

class Drive : public frc2::SubsystemBase {
public:
  Drive(bool invertL, bool invertR, double rampX, double rampR);

  void AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB);
  void AttachPneumatics(Pneumatics *pneumatics);

  void SetPower(double x, double r, double k = 1.0);
  void SetVolts(units::volt_t left, units::volt_t right);

  void Periodic() override;

  void Calculate();

  frc::Pose2d GetPosition();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  void ResetOdometry(frc::Pose2d start = kStartPos, bool calibrateImu = false);

  bool m_controllerControllable = true;

  static constexpr frc::Pose2d kStartPos {0_m, 0_m, 0_deg};

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandXboxController *m_driverControllerA = nullptr;
  frc2::CommandXboxController *m_driverControllerB = nullptr;

  Pneumatics *m_pneumatics = nullptr;

  MotorDriver m_motors[4] {
    {CanIds::kDriveL1},
    {CanIds::kDriveL2},
    {CanIds::kDriveR1},
    {CanIds::kDriveR2},
  };

  frc::MotorControllerGroup m_ctrlL {m_motors[0], m_motors[1]};
  frc::MotorControllerGroup m_ctrlR {m_motors[2], m_motors[3]};
  frc::DifferentialDrive m_drive {m_ctrlL, m_ctrlR};

  frc::ADIS16470_IMU m_imu {};

  WheelOdometryInfo m_leftInfo;
  WheelOdometryInfo m_rightInfo;

  frc::DifferentialDriveOdometry m_odometry {
    frc::Rotation2d(units::radian_t(m_imu.GetAngle())),
    0.0_m, 0.0_m, kStartPos
  };

  const double m_rampX, m_rampR;
  double m_curX = 0.0, m_curR = 0.0;

  bool m_usePosition = false;
  bool m_motorsSetToPosition = false;

  static constexpr double kCoeffDriveTrain = 1.0;
  static constexpr double maxEncoderSpeed = 4096;
  static constexpr const Gains kDriveGains {0.05, 0.0, 1.0, 0.0, 0, 1.0};

  void HandleController();

  units::meter_t GetEncoderTicksToMeterFactor();

  double GetAverageMotorPosition(int front, int rear);
  double GetAverageMotorVelocity(int front, int rear);

  void InitializePID();
  void InitializeMotorPID(MotorDriver *motor);
  void SetUsePosition(bool value);
  bool GetUsePosition();
  void MoveUsingPosition(double x, double r);
  void AddPositionToMotor(MotorDriver* drive, int amount);
};
