#pragma once

#include <functional>
#include <string>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Button.h>

#include <rev/SparkMaxPIDController.h>
#include <rev/CANSparkMax.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <units/angle.h>

#include "commands/SetArmPosition.h"

#include "Constants.h"
#include "subsystems/Pneumatics.h"

using IntakeMotor = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

class RobotContainer;

class Intake : public frc2::SubsystemBase {
  public:
    Intake(bool invertRotate, bool invertLeft, bool invertRight);

    void SetWheelPower(double power);
    // bool IsWheelOn();
    // void StopWheel();

    void Periodic() override;
    void TeleopLoop();

    // void LiftIntake();
    // void LowerIntake();

    // bool isDown();

  void AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB);
  void AttachPneumatics(Pneumatics *pneumatics);


  private:
    IntakeMotor m_rotateMotor {CanIds::kIntakeRotate};
    IntakeMotor m_LeftWheelMotor {CanIds::kIntakeLeftWheel};
    IntakeMotor m_RightWheelMotor {CanIds::kIntakeRightWheel};

    Pneumatics *m_pneumatics = nullptr;
    frc2::CommandXboxController *m_driverControllerA = nullptr;
    frc2::CommandXboxController *m_driverControllerB = nullptr;    

    //bool isDown;    
};