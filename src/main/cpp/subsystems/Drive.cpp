// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Drive.h"

#include "Constants.h"

#include "Util.h"

Drive::Drive(bool invertL, bool invertR, double rampX, double rampR) : m_rampX(rampX), m_rampR(rampR) {
  m_motors[0].SetInverted(invertL);
  m_motors[1].SetInverted(invertL);
  m_motors[2].SetInverted(invertR);  
  m_motors[3].SetInverted(invertR);

  ResetOdometry();
}

void Drive::AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB) {
  m_driverControllerA = driverControllerA;
  m_driverControllerB = driverControllerB;
}

void Drive::AttachPneumatics(Pneumatics *pneumatics) {
  m_pneumatics = pneumatics;
}

void Drive::SetPower(double x, double r, double k) {
  Util::ramp(&m_curX, x * k, m_rampX);
  Util::ramp(&m_curR, r * k, m_rampR);

  m_drive.ArcadeDrive(m_curX, m_curR);
}

void Drive::SetVolts(units::volt_t left, units::volt_t right) {
  m_ctrlL.SetVoltage(left);
  m_ctrlR.SetVoltage(right);
  m_drive.Feed();
}

void Drive::Periodic() {
  if (m_controllerControllable) {
    HandleController();
  }

  if (m_driverControllerA->GetAButtonPressed()) {
    ResetOdometry(kStartPos, true);
  }

  Calculate();

  m_odometry.Update(
    frc::Rotation2d(units::radian_t(m_imu.GetAngle())),
    m_leftInfo.distance, m_rightInfo.distance
  );

  frc::SmartDashboard::PutNumber("L Encoder", GetAverageMotorPosition(0, 1) / 2048.0);
  frc::SmartDashboard::PutNumber("R Encoder", GetAverageMotorPosition(2, 3) / 2048.0);

  frc::SmartDashboard::PutString("L Info", m_leftInfo.Stringify());
  frc::SmartDashboard::PutString("R Info", m_rightInfo.Stringify());

  frc::SmartDashboard::PutNumber("Pos. X", m_odometry.GetPose().Translation().X().to<double>());
  frc::SmartDashboard::PutNumber("Pos. Y", m_odometry.GetPose().Translation().Y().to<double>());
  frc::SmartDashboard::PutNumber("Pos. A", m_odometry.GetPose().Rotation().Degrees().to<double>());
}

void Drive::Calculate() {
  units::meter_t conversionFactor = GetEncoderTicksToMeterFactor();
  m_leftInfo .Calculate(GetAverageMotorPosition(0, 1), GetAverageMotorVelocity(0, 1), conversionFactor);
  m_rightInfo.Calculate(GetAverageMotorPosition(2, 3), GetAverageMotorVelocity(2, 3), conversionFactor);
}

frc::Pose2d Drive::GetPosition() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds Drive::GetWheelSpeeds() {
  return {m_leftInfo.velocity, m_rightInfo.velocity};
}

void Drive::ResetOdometry(frc::Pose2d start, bool calibrateImu) {
  m_imu.Reset();

  if (calibrateImu) {
    m_imu.Calibrate();
  }

  for (auto &motor : m_motors) {
    motor.SetSelectedSensorPosition(0.0);
  }

  m_leftInfo.Reset();
  m_rightInfo.Reset();

  m_odometry.ResetPosition(m_imu.GetAngle(), 0.0_m, 0.0_m, start);
}

void Drive::HandleController() {
  const auto x = Util::thresholded(m_driverControllerA->GetLeftY(), -0.1, 0.1);
  const auto r = Util::thresholded(m_driverControllerA->GetRightX(), -0.1, 0.1);

  // x is negative because joystick y-axis is inverted
  SetPower(-x, r, Drive::kCoeffDriveTrain);
}

units::meter_t Drive::GetEncoderTicksToMeterFactor(){
  return (
    DriveConstants::kDriveBaseEncoderDistancePerPulse /
    (m_pneumatics->IsHighGear() ? DriveConstants::kDriveHighGearRatio : DriveConstants::kDriveLowGearRatio)
  );
}

double Drive::GetAverageMotorPosition(int front, int rear) {
  return (m_motors[front].GetSelectedSensorPosition() + m_motors[rear].GetSelectedSensorPosition()) / 2.0;
}

double Drive::GetAverageMotorVelocity(int front, int rear) {
  return (m_motors[front].GetSelectedSensorVelocity() + m_motors[rear].GetSelectedSensorVelocity()) / 2.0;
}

void WheelOdometryInfo::Calculate(double curPosition, double curVelocity, units::meter_t tickToMeterFactor) {
  lastPosition = curPosition;
  distance += (curPosition - lastPosition) * tickToMeterFactor;
  velocity = curVelocity * tickToMeterFactor / 100.0_ms;
}

void WheelOdometryInfo::Reset(double encoderPosition) {
  lastPosition = encoderPosition;
  distance = 0.0_m;
  velocity = 0.0_mps;
}

std::string WheelOdometryInfo::Stringify() {
  std::stringstream ss;
  ss <<
    "WheelOdometryInfo {" <<
    "lastPosition=" << lastPosition << ", " <<
    "distance=" << distance.to<double>() << ", " <<
    "velocity=" << velocity.to<double>() <<
    "}";
  return ss.str();
}
