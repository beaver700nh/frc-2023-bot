// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Drive.h"

#include "Constants.h"

#include "Util.h"

Drive::Drive(bool invertL, bool invertR, double rampX, double rampR)
  : m_rampX(rampX), m_rampR(rampR) {
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
  if (!m_driverControllerA || !m_driverControllerB) {
    std::cerr << "ERROR in Drive: driverController is null." << std::endl;
    return;
  }

  if (m_controllerControllable) {
    HandleController();
  }

  if (m_driverControllerA->GetAButtonPressed()) {
    ResetOdometry(kStartPos, true);
  }

  Calculate();

  m_odometry.Update(
    frc::Rotation2d(units::radian_t(m_imu.GetAngle())),
    m_leftInfo.distance,
    m_rightInfo.distance
  );

  frc::SmartDashboard::PutNumber("left dist", m_leftInfo.distance.to<double>());
  frc::SmartDashboard::PutNumber("right dist", m_rightInfo.distance.to<double>());

  // frc::SmartDashboard::PutNumber("encoder left", m_motors[0].GetSelectedSensorPosition());

  frc::SmartDashboard::PutNumber("position x", m_odometry.GetPose().Translation().X().to<double>());
  frc::SmartDashboard::PutNumber("position y", m_odometry.GetPose().Translation().Y().to<double>());
  frc::SmartDashboard::PutNumber("position angle", m_odometry.GetPose().Rotation().Degrees().to<double>());

  frc::SmartDashboard::PutString("left info", m_leftInfo.toString());
  frc::SmartDashboard::PutString("right info", m_rightInfo.toString());

  frc::SmartDashboard::PutNumber("wheel left", GetWheelSpeeds().left.to<double>());
  frc::SmartDashboard::PutNumber("wheel right", GetWheelSpeeds().right.to<double>());
}

void Drive::HandleController() {
  const auto x = Util::thresholded(m_driverControllerA->GetLeftY(), -0.1, 0.1);
  const auto r = Util::thresholded(m_driverControllerA->GetRightX(), -0.1, 0.1);

  // x is negative because joystick y-axis is inverted
  SetPower(-x, r, Drive::kCoeffDriveTrain);
}

units::meter_t Drive::EncoderTicksToMeterFactor(){
  return (
    OperatorConstants::kDriveBaseEncoderDistancePerPulse /
    (m_pneumatics->IsHighGear() ? OperatorConstants::kDriveHighGearRatio : OperatorConstants::kDriveLowGearRatio)
  );
}

void Drive::Calculate() {
  m_leftInfo.calculate(m_motors + 0, m_motors + 1, EncoderTicksToMeterFactor());
  m_rightInfo.calculate(m_motors + 2, m_motors + 3, EncoderTicksToMeterFactor());
}

const WheelOdometryInfo &Drive::GetLeftInfo() {
  return m_leftInfo;
}

const WheelOdometryInfo &Drive::GetRightInfo() {
  return m_rightInfo;
}

frc::DifferentialDriveWheelSpeeds Drive::GetWheelSpeeds() {
  return {m_leftInfo.velocity, m_rightInfo.velocity};
}

frc::Pose2d Drive::GetPosition() {
  return m_odometry.GetPose();
}

void Drive::ResetOdometry(frc::Pose2d start, bool calibrateImu) {
  m_imu.Reset();

  if (calibrateImu) {
    m_imu.Calibrate();
  }

  for (auto &motor : m_motors) {
    motor.SetSelectedSensorPosition(0.0);
  }

  m_leftInfo.reset();
  m_rightInfo.reset();

  m_odometry.ResetPosition(m_imu.GetAngle(), 0.0_m, 0.0_m, start);
}

void WheelOdometryInfo::calculate(MotorDriver *motorFront, MotorDriver *motorRear, units::meter_t tickToMeterFactor) {
  double encoderTicks = (motorFront->GetSelectedSensorPosition() + motorRear->GetSelectedSensorPosition()) / 2.0;
  units::meter_t dist = (encoderTicks - lastEncoderTicks) * tickToMeterFactor;
  distance += dist;

  velocity = (motorFront->GetSelectedSensorVelocity() + motorRear->GetSelectedSensorVelocity()) / 2.0 * tickToMeterFactor / 100_ms;

  lastEncoderTicks = encoderTicks;
}

void WheelOdometryInfo::reset(double encoderPosition) {
  lastEncoderTicks = encoderPosition;
  distance = 0_m;
  velocity = 0_mps;
}