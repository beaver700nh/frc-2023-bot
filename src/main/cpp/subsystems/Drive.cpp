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

  if (m_driverControllerA->GetLeftBumper()){
    m_pneumatics->SetGear(false);
  } 
  else{
     m_pneumatics->SetGear(std::abs(m_curX) > 0.8);
  }

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

  // if (m_driverControllerA->GetAButtonPressed()) {
  //   ResetOdometry(kStartPos, true);
  // }

  if(m_driverControllerA->GetBButtonPressed()){
    SetUsePosition(!m_usePosition);
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

  frc::SmartDashboard::PutBoolean("Balance Mode", m_usePosition);
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
    if(m_usePosition)
      AddPositionToMotor(&motor, 0);
  }

  m_leftInfo.Reset();
  m_rightInfo.Reset();

  m_odometry.ResetPosition(m_imu.GetAngle(), 0.0_m, 0.0_m, start);
}

void Drive::HandleController() {
  const auto x = Util::thresholded(m_driverControllerA->GetLeftY() * (1 - 0.95 * m_driverControllerA->GetLeftTriggerAxis()), -0.1, 0.1);
  const auto r = Util::thresholded(m_driverControllerA->GetRightX() * (1 - 0.95 * m_driverControllerA->GetRightTriggerAxis()), -0.1, 0.1);

  // x is negative because joystick y-axis is inverted
  if(m_usePosition){
    // if(x == 0 && r == 0){
      
    //   if(!m_motorsSetToPosition){
    //     for(MotorDriver &motor : m_motors)
    //       AddPositionToMotor(&motor, 0);
    //     m_motorsSetToPosition = true;
    //   }

    //   m_drive.Feed();
    // } else {
    //   SetPower(-x, r, Drive::kCoeffDriveTrain);
    //   m_motorsSetToPosition = false;
    // }

    MoveUsingPosition(-x,r);
  } else {
    SetPower(-x, -r, Drive::kCoeffDriveTrain);
    m_motorsSetToPosition = false;
  }
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

void Drive::InitializeMotorPID(MotorDriver *motor){
  std::cout << "config motor :( \n";


  motor->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);

		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 */
    //motor->SetSensorPhase(false);

		/* Config the peak and nominal outputs, 12V means full */
		motor->ConfigNominalOutputForward(0);
		motor->ConfigNominalOutputReverse(0);
		motor->ConfigPeakOutputForward(kDriveGains.peakOutput);
		motor->ConfigPeakOutputReverse(-kDriveGains.peakOutput);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		motor->ConfigAllowableClosedloopError(0, 512);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		motor->Config_kF(0, kDriveGains.f);
		motor->Config_kP(0, kDriveGains.p);
		motor->Config_kI(0, kDriveGains.i);
		motor->Config_kD(0, kDriveGains.d);
    motor->Config_IntegralZone(0, kDriveGains.iZone);

    //sets target position to current position
    AddPositionToMotor(motor, 0);
}

void Drive::InitializePID(){
  std::cout << "initialize pid \n";
  for(MotorDriver &motor : m_motors) {
    InitializeMotorPID(&motor);
  }
}

void Drive::SetUsePosition(bool value){
  if(value) InitializePID();

  m_usePosition = value;
}

bool Drive::GetUsePosition(){
  return m_usePosition;
}

void Drive::AddPositionToMotor(MotorDriver* motor, int amount){
  std::cout << "add to motor, amount: " << amount << ", curr pos: " << motor->GetSelectedSensorPosition() << " \n";
  motor -> Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position, motor->GetSelectedSensorPosition() + amount);
}

void Drive::MoveUsingPosition(double x, double r){
  m_drive.Feed();
  
  if(x == 0 && r == 0) return;

  x *= maxEncoderSpeed;
  r *= maxEncoderSpeed;

  AddPositionToMotor(m_motors + 0, x + r);
  AddPositionToMotor(m_motors + 1, x + r);
  AddPositionToMotor(m_motors + 2, x - r);
  AddPositionToMotor(m_motors + 3, x - r);
}

void WheelOdometryInfo::Calculate(double curPosition, double curVelocity, units::meter_t tickToMeterFactor) {
  distance += (curPosition - lastPosition) * tickToMeterFactor;
  velocity = curVelocity * tickToMeterFactor / 100.0_ms;
  lastPosition = curPosition;
}

void WheelOdometryInfo::Reset(double encoderPosition) {
  distance = 0.0_m;
  velocity = 0.0_mps;
  lastPosition = encoderPosition;
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
