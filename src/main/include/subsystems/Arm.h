// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>

#include <frc/DigitalInput.h>

#include <rev/SparkMaxPIDController.h>
#include <rev/CANSparkMax.h>

#include <units/angle.h>

#include "commands/SetArmPosition.h"

#include "subsystems/Drive.h"
#include "subsystems/Pneumatics.h" 

#include "Constants.h"

using MotorArm = rev::CANSparkMax;
using MotorArmType = rev::CANSparkMaxLowLevel::MotorType;
using SparkMaxCtrlType = rev::CANSparkMax::ControlType;

struct ArmComponent {
public:
  ArmComponent(int motorCanId, int lmswPort, double coeff, double minPos, double maxPos, double tolerance = 1, int timesInTolReq = 3);

  struct MoveInfo {
    double position, adjusted, constrained;
  };

  void Initialize(bool invert, double p, double i, double d, double iz, double ff, double min, double max);
  std::optional<MoveInfo> Set(double rawControllerInput, bool inverted);
  double SetAbsolute(double pos);
  void Reset(double pos = 0.0);
  void CheckTolerance();
  bool InTolerance();

  MotorArm motor;
  rev::SparkMaxRelativeEncoder encoder;
  rev::SparkMaxPIDController pidCtrl;
  frc::DigitalInput lmsw;
  double targetPosition;
  int timesInTol;

  const double coeff, minPos, maxPos, tolerance;
  const int timesInTolReq;
};

class Arm : public frc2::SubsystemBase {
public:
  Arm(bool invertTilt, bool invertRotate, bool invertExtend);

  void AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB);
  void AttachPneumatics(Pneumatics *pneumatics);
  void AttachDrive(Drive *drive);

  void Periodic() override;

  void PointTo(frc::Translation2d target);
  void PointToZero();
  void PointToAngle(units::radian_t angle);

  double GetEncoderForAngle(units::radian_t angle);
  void TurnArmToAngle(units::radian_t angle);

  bool InTolerance();

  ArmComponent m_tilt   {CanIds::kArmTilt,   PortsDIO::kArmLmswTilt,   2.5, 0.0, 160.0};
  ArmComponent m_rotate {CanIds::kArmRotate, PortsDIO::kArmLmswRotate, 3.5, -125.0, 125.0};
  ArmComponent m_extend {CanIds::kArmExtend, PortsDIO::kArmLmswExtend, 1.5, 0.0, 115.0};
  
  const double rotateScaleFactor = (m_rotate.maxPos - m_rotate.minPos) / (2 * M_PI);


  SetArmPosition m_position_zero {this, ArmPosition(0.0, 0.0, 0.0)};
  SetArmPosition m_position_pickup {this, ArmPosition(100, 0, 0)};
  SetArmPositionExWait m_position_test {this,
    {
      ArmPosition(std::nullopt,0,std::nullopt),
      ArmPosition(std::nullopt,GetEncoderForAngle(30_deg),std::nullopt),
      ArmPosition(std::nullopt,0,std::nullopt)
    }
  }; 

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandXboxController *m_driverControllerA = nullptr;
  frc2::CommandXboxController *m_driverControllerB = nullptr;

  Pneumatics *m_pneumatics = nullptr;

  Drive *m_drive = nullptr;

  void AutoShoe(std::optional<ArmComponent::MoveInfo> info);
};