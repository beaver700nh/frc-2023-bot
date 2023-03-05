// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/POVButton.h>

#include "commands/Autos.h"
#include "commands/ClawControl.h"
#include "commands/HomeArm.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  std::cerr << "HELLO WORLD! - Brian" << std::endl;

  // Initialize all of your commands and subsystems here
  m_arm  .AttachController(&m_driverControllerA, &m_driverControllerB);
  m_drive.AttachController(&m_driverControllerA, &m_driverControllerB);
  m_pneu .AttachController(&m_driverControllerA, &m_driverControllerB);

  m_arm  .AttachPneumatics(&m_pneu);
  m_drive.AttachPneumatics(&m_pneu);

  m_arm  .AttachDrive(&m_drive);

  autos::populate(&m_arm, &m_drive, &m_pneu);
  for(unsigned int i = 0; i < autos::autos.size(); i++){
    autoChooser.AddOption(autos::autos.at(i).first, i);
  }

  frc::SmartDashboard::PutData("choose auto", &autoChooser);

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  m_driverControllerA.A().OnTrue(
    frc2::cmd::Either(
      frc2::cmd::Sequence(m_arm.m_position_loCubeSide), 
      frc2::cmd::Sequence(m_arm.m_position_loCubeBack), 
      [this](){
        return m_arm.IsForwards();
      }
    )
  );

  m_driverControllerA.B().OnTrue(
    frc2::cmd::Either(
      frc2::cmd::Sequence(m_arm.m_position_hiCubeSide), 
      frc2::cmd::Sequence(m_arm.m_position_hiCubeBack), 
      [this](){
        return m_arm.IsForwards();
      }
    )
  );

    m_driverControllerA.X().OnTrue(
    frc2::cmd::Either(
      frc2::cmd::Sequence(m_arm.m_position_loConeSide), 
      frc2::cmd::Sequence(m_arm.m_position_loConeBack), 
      [this](){
        return m_arm.IsForwards();
      }
    )
  );

  m_driverControllerA.Y().OnTrue(
    frc2::cmd::Either(
      frc2::cmd::Sequence(m_arm.m_position_hiConeSide), 
      frc2::cmd::Sequence(m_arm.m_position_hiConeBack), 
      [this](){
        return m_arm.IsForwards();
      }
    )
  );

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverControllerB.Y().OnTrue(
    // frc2::cmd::Parallel(
      frc2::cmd::Sequence(
        HomeArmExtend(&m_arm).ToPtr(),
        HomeArmTilt(&m_arm).ToPtr(),
      // ),
      // frc2::cmd::Sequence(
        HomeArmRotate(&m_arm).ToPtr()
      )
    // )
  );

  frc2::POVButton up {&m_driverControllerB, 0};
  up.OnTrue(
    SetArmPosition(
      &m_arm,
      {std::nullopt, 0, std::nullopt}
    ).ToPtr()
  );

  frc2::POVButton left {&m_driverControllerB, 270};
  left.OnTrue(
    SetArmPosition(
      &m_arm,
      {std::nullopt, -60, std::nullopt}
    ).ToPtr()
  );

  frc2::POVButton right {&m_driverControllerB, 90};
  right.OnTrue(
    SetArmPosition(
      &m_arm,
      {std::nullopt, 60, std::nullopt}
    ).ToPtr()
  );

  frc2::POVButton down {&m_driverControllerB, 180};
  down.OnTrue(
    SetArmPosition(
      &m_arm,
      {std::nullopt, m_arm.m_rotate.encoder.GetPosition() < 0 ? -125 : 125, std::nullopt}
    ).ToPtr()
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return std::move(
    autos::autos.at(autoChooser.GetSelected()).second
  );
  //.AndThen(std::move(armDown).ToPtr())
  //.AndThen([this] {m_pneu.SetClaw(true);})
  //.AndThen([this] {m_pneu.SetClaw(false);})
  //.AndThen(Movement::GenerateCommand(&m_drive, {4.75_m, 0.0_m, 0_deg}, {}, {0_m, 0.0_m, 0_deg}, true));
}
