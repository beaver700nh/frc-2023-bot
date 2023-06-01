#include <iostream>

#include <subsystems/Intake.h>

#include <frc2/command/Commands.h>


Intake::Intake(bool invertRotate, bool invertLeft, bool invertRight){
  m_rotateMotor.SetInverted(invertRotate);
  m_LeftWheelMotor.SetInverted(invertLeft);
  m_RightWheelMotor.SetInverted(invertRight);
}

void Intake::AttachController(frc2::CommandXboxController *driverControllerA, frc2::CommandXboxController *driverControllerB) {
  m_driverControllerA = driverControllerA;
  m_driverControllerB = driverControllerB;
}

void Intake::AttachPneumatics(Pneumatics *pneumatics) {
  m_pneumatics = pneumatics;

  // SetDefaultCommand(
  //   frc2::cmd::Run(
  //     [this]{TeleopLoop();}
  //   )
  // );
}

void Intake::Periodic(){
  TeleopLoop();
}

void Intake::TeleopLoop(){
  //std::cout << "hello \n";

  if(m_driverControllerA->GetPOV() == 90){
    SetWheelPower(.5);
    //std::cout << "wheel .5 \n";
  } else if(m_driverControllerA->GetPOV() == 270) {
    SetWheelPower(-.5);
    //std::cout << "wheel -.5 \n";
  } else {
    SetWheelPower(0);
    //std::cout << "wheel 0 \n";
  }

  if(m_driverControllerA->GetPOV() == 0){
    m_rotateMotor.Set(-0.5);
    //std::cout << "rotate -.5 \n";
  } else if(m_driverControllerA->GetPOV() == 180) {
    m_rotateMotor.Set(0.5);
    //    std::cout << "rotate .5 \n";
  } else {
     m_rotateMotor.Set(0);
    //    std::cout << "rotate 0 \n";
  }

  if(m_driverControllerB->GetAButtonPressed()){
    m_pneumatics->SetIntake(!m_pneumatics->IsIntakeForward());
  }
}

void Intake::SetWheelPower(double power){
  m_LeftWheelMotor.Set(power);
  m_RightWheelMotor.Set(power);
}