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

struct WheelOdometryInfo {
  double lastEncoderTicks = 0.0;
  units::meter_t distance = 0_m;
  units::meters_per_second_t velocity = 0_mps;

  void calculate(MotorDriver *motorFront, MotorDriver *motorRear, units::meter_t tickToMeterFactor);

  void reset(double encoderPosition = 0.0);

  std::string toString() {
    std::stringstream ss;
    ss <<
      "WheelOdometryInfo{" <<
      "lastEncoderTicks=" << lastEncoderTicks << ", " <<
      "distance=" << distance.to<double>() << ", " <<
      "velocity=" << velocity.to<double>() <<
      "}";
    return ss.str();
  } 
};

class Drive : public frc2::SubsystemBase {
public:
  Drive(bool invertL, bool invertR, double rampX, double rampR);

  void AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB);
  void AttachPneumatics(Pneumatics *pneumatics);

  void SetPower(double x, double r, double k = 1.0);
  void SetVolts(units::volt_t left, units::volt_t right);

  void Periodic() override;

  frc::Pose2d GetPosition();

  const WheelOdometryInfo &GetLeftInfo();
  const WheelOdometryInfo &GetRightInfo();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  bool m_controllerControllable = true;

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
  
  const double m_rampX, m_rampR;

  double m_curX = 0.0, m_curR = 0.0;

  static constexpr double kCoeffDriveTrain = 1.0;

  frc::ADIS16470_IMU m_imu {};

  WheelOdometryInfo m_leftInfo;
  WheelOdometryInfo m_rightInfo;

  units::second_t m_lastUpdateTime;

  static constexpr frc::Pose2d kStartPos {0_in ,0_in, 90_deg};//{-84_in, -84_in, 90_deg};

  frc::DifferentialDriveOdometry m_odometry {
    frc::Rotation2d(units::radian_t(m_imu.GetAngle())),
    0.0_m, 0.0_m,
    kStartPos
  };

  units::meter_t EncoderTicksToMeterFactor();

  void Calculate();

  void ResetOdometry(frc::Pose2d start = kStartPos, bool calibrateImu = false);

  void HandleController();
};
