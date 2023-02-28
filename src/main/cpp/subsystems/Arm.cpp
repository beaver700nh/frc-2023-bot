// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Arm.h"

#include "Util.h"

Arm::Arm(bool invertTilt, bool invertRotate, bool invertExtend) {
  m_tilt  .Initialize(invertTilt,   1.4e-1, 1.0e-5, 1.0e+0, 0.0, 0.0, -0.5, 0.5);
  m_rotate.Initialize(invertRotate, 1.0e-1, 1.0e-5, 1.0e+0, 0.0, 0.0, -0.35, 0.35);
  m_extend.Initialize(invertExtend, 1.0e-1, 1.0e-4, 1.0e+0, 0.0, 0.0, -0.5, 0.5);
}

void Arm::AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB) {
  m_driverControllerA = driverControllerA;
  m_driverControllerB = driverControllerB;
}

void Arm::AttachPneumatics(Pneumatics *pneumatics) {
  m_pneumatics = pneumatics;
}

void Arm::AttachDrive(Drive *drive) {
  m_drive = drive;
}

void Arm::Periodic() {
  const double tilt = m_driverControllerB->GetLeftY();
  auto info = m_tilt.Set(tilt, true);
  AutoShoe(info);

  const double rotate = m_driverControllerB->GetRightX();
  m_rotate.Set(rotate, false);

  const double extend = m_driverControllerB->GetRightTriggerAxis() - m_driverControllerB->GetLeftTriggerAxis();
  m_extend.Set(extend, false);

  // m_tilt.CheckTolerance();

  // if (m_driverControllerB->GetYButton()) {
  //   PointToZero();
  // }
  // else if (m_driverControllerB->GetYButtonReleased()) {
  //   m_rotate.SetAbsolute(m_rotate.encoder.GetPosition());
  // }

  frc::SmartDashboard::PutBoolean("Lim.Sw. Tilt",   m_tilt  .lmsw.Get());
  frc::SmartDashboard::PutBoolean("Lim.Sw. Rotate", m_rotate.lmsw.Get());
  frc::SmartDashboard::PutBoolean("Lim.Sw. Extend", m_extend.lmsw.Get());

  frc::SmartDashboard::PutNumber("Enc. Tilt", m_tilt.encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Enc. Rotate", m_rotate.encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Enc. Extend", m_extend.encoder.GetPosition());

  frc::SmartDashboard::PutBoolean("Arm In Position", InTolerance());
}

bool Arm::InTolerance(){
  bool tiltIn = m_tilt.InTolerance();
  bool rotateIn = m_rotate.InTolerance();
  bool extendIn = m_extend.InTolerance();

  frc::SmartDashboard::PutBoolean("tilt", tiltIn);
  frc::SmartDashboard::PutBoolean("rotate", rotateIn);
  frc::SmartDashboard::PutBoolean("extend", extendIn);

  return m_tilt.InTolerance() && m_rotate.InTolerance() && m_extend.InTolerance();
}

void Arm::AutoShoe(std::optional<ArmComponent::MoveInfo> info) {
  if (!info.has_value()) {
    return;
  }

  if (std::abs(info.value().position - info.value().constrained) >= 2.2) {
    m_pneumatics->Unshoe();
  }
  else {
    m_pneumatics->Shoe();
  }
}

void Arm::PointTo(frc::Translation2d target){
  frc::Pose2d pos = m_drive->GetPosition();

  // Convert to distance from robot
  frc::Translation2d targetCord = target - pos.Translation();

  units::radian_t baseAngle {-std::atan2(targetCord.Y().to<double>(), targetCord.X().to<double>())};

  TurnArmToAngle(baseAngle + pos.Rotation().Radians());
}

void Arm::PointToZero() {
  PointTo({});
}

void Arm::PointToAngle(units::radian_t angle){
  TurnArmToAngle(angle + (m_drive->GetPosition().Rotation().Radians() - units::radian_t{M_PI_2}));
}

double Arm::GetEncoderForAngle(units::radian_t angle){
  return Util::scaleAngle(angle).to<double>() * rotateScaleFactor;
}

void Arm::TurnArmToAngle(units::radian_t angle) {
  m_rotate.SetAbsolute(GetEncoderForAngle(angle));
}

ArmComponent::ArmComponent(int motorCanId, int lmswPort, double coeff, double minPos, double maxPos, double tolerance, int timesInTolReq) :
  motor(motorCanId, MotorArmType::kBrushless), encoder(motor.GetEncoder()), pidCtrl(motor.GetPIDController()),
  lmsw(lmswPort), coeff(coeff), minPos(minPos), maxPos(maxPos), tolerance(tolerance), timesInTolReq(timesInTolReq) {
  // empty
}

void ArmComponent::Initialize(bool invert, double p, double i, double d, double iz, double ff, double min, double max) {
  motor.SetInverted(invert);

  encoder.SetPosition(0.0);

  SetAbsolute(0.0);
  pidCtrl.SetP(p);
  pidCtrl.SetI(i);
  pidCtrl.SetD(d);
  pidCtrl.SetIZone(iz);
  pidCtrl.SetFF(ff);
  pidCtrl.SetOutputRange(min, max);
}

std::optional<ArmComponent::MoveInfo> ArmComponent::Set(double rawControllerInput, bool inverted) {
  const double thresholded = Util::thresholded(rawControllerInput, -0.08, 0.08);

  if (thresholded == 0) {
    return std::nullopt;
  }

  const double position = encoder.GetPosition();
  const double adjusted = position + coeff * thresholded * (inverted ? -1.0 : 1.0);

  return MoveInfo {position, adjusted, SetAbsolute(adjusted)};
}

double ArmComponent::SetAbsolute(double pos) { 
  const double constrained = Util::constrained(pos, minPos, maxPos);

  pidCtrl.SetReference(constrained, SparkMaxCtrlType::kPosition);
  targetPosition = constrained;

  return constrained;
}

void ArmComponent::Reset(double pos) {
  motor.Set(0.0);
  encoder.SetPosition(SetAbsolute(pos));
}

void ArmComponent::CheckTolerance(){
  if(std::abs(targetPosition - encoder.GetPosition()) <= tolerance)
    timesInTol ++;
  else 
    timesInTol = 0;
}

/**
 * This will call CheckTolerance so if you are calling this repetedly there is no need to call CheckTolerance
*/
bool ArmComponent::InTolerance(){
  CheckTolerance();
  return timesInTol >= timesInTolReq;
}
