// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <string>

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
  ArmComponent(bool unlock, int motorCanId, int lmswPort, double coeff, double minPos, double maxPos, double tolerance = 2, int timesInTolReq = 3);

  struct MoveInfo {
    double position, adjusted, constrained;
  };

  void Initialize(bool invert, double p, double i, double d, double iz, double ff, double min, double max);
  std::optional<MoveInfo> Set(double rawControllerInput, bool inverted);
  double SetAbsolute(double pos);
  void Reset(double pos = 0.0);
  void CheckTolerance();
  bool InTolerance();
  void AttachPneumatics(Pneumatics *pneu);

  Pneumatics *pneu = nullptr;
  bool unlock;

  MotorArm motor;
  rev::SparkMaxRelativeEncoder encoder;
  rev::SparkMaxPIDController pidCtrl;
  frc::DigitalInput lmsw;
  double targetPosition;
  int timesInTol;

  const double coeff, minPos, maxPos;

  const double tolerance;
  const int timesInTolReq;
};

// struct VisionCubePipeline {
//   VisionCubePipeline(std::string cameraName);

//   bool IsCubeFound();
//   double GetCubeX();
//   double GetCubeY();
//   double GetCubeW();
//   double GetCubeH();

//   void SetPipelineEnabled(bool enabled);
//   bool IsPilelineEnabled();

//   private:
//     const std::string m_cameraName;
// };

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
  bool IsForwards();

  ArmComponent m_tilt   {true, CanIds::kArmTilt,   PortsDIO::kArmLmswTilt,   2.5, 0.0, 160.0, 3};
  ArmComponent m_rotate {false, CanIds::kArmRotate, PortsDIO::kArmLmswRotate, 3.5, -135.0, 135.0, 4, 6};
  ArmComponent m_extend {false, CanIds::kArmExtend, PortsDIO::kArmLmswExtend, 1.5, 0.0, 118.0};
  
  const double rotateScaleFactor = (m_rotate.maxPos - m_rotate.minPos) / (2 * M_PI);

  SetArmPositionWait m_position_up {this, {0, std::nullopt, 0}};
  SetArmPosition m_position_pickup {this, {150, std::nullopt, 0}};
  SetArmPosition m_position_pickupFront {this, {150, 0, 0}};
  SetArmPositionWait m_position_pickupFront_wait {this, {150, 0, 0}};
  SetArmPositionWait m_position_loConeSide {this, {50.0, std::nullopt,  48.0}};
  SetArmPositionWait m_position_hiConeSide {this, {52.7, std::nullopt, 115.0}};
  SetArmPositionWait m_position_loConeBack {this, {35.0, std::nullopt,  27.0}};
  SetArmPositionWait m_position_hiConeBack {this, {45.0, std::nullopt, 115.0}};
  SetArmPositionWait m_position_loCubeSide {this, {64.8, std::nullopt,  22.0}};
  SetArmPositionWait m_position_hiCubeSide {this, {67.5, std::nullopt,  95.8}};
  SetArmPositionWait m_position_loCubeBack {this, {55.0, std::nullopt,   6.4}};
  SetArmPositionWait m_position_hiCubeBack {this, {65.0, std::nullopt,  85.0}};
  SetArmPositionWait m_position_hiCubeFront{this, {65, std::nullopt,  114}};
  SetArmPositionExWait m_position_slideShelf {
    this, 
    {
      {std::nullopt, -0, std::nullopt},
      {36.5, std::nullopt, 3.1}
    }
  };
  SetArmPositionExWait m_position_test {this,
    {
      ArmPosition(50,0,std::nullopt),
      ArmPosition(0,0,std::nullopt),
      ArmPosition(50,0,std::nullopt)
    }
  }; 
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandXboxController *m_driverControllerA = nullptr;
  frc2::CommandXboxController *m_driverControllerB = nullptr;

  Pneumatics *m_pneumatics = nullptr;

  Drive *m_drive = nullptr;

  void AutoShoe(std::optional<ArmComponent::MoveInfo> info);
};