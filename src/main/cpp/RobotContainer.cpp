// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
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

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverControllerB.X().OnTrue(HomeArmTilt  (&m_arm).ToPtr());
  m_driverControllerB.A().OnTrue(HomeArmRotate(&m_arm).ToPtr());
  m_driverControllerB.B().OnTrue(HomeArmExtend(&m_arm).ToPtr());

  //m_driverControllerA.B().OnTrue(std::move(m_arm.m_position_test).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto armDown = m_arm.m_position_pickup;

  std::cout << "getting auto command \n";

  auto currentMovement = Movement::GenerateCommand(&m_drive, {-0.5_m, 0.0_m, 0_deg}, true);

  return std::move(currentMovement).AndThen(std::function([this, &currentMovement]{currentMovement = Movement::GenerateCommand(&m_drive, {-0.5_m, 0.0_m, 0_deg}, true);}))
    //.AndThen(std::move(armDown).ToPtr())
    //.AndThen([this] {m_pneu.SetClaw(true);})
    .AndThen(Movement::GenerateCommand(&m_drive, {-0.5_m, 0.0_m, 0_deg}, {}, {4.75_m, 0.0_m, 0_deg}))
    //.AndThen([this] {m_pneu.SetClaw(false);})
    .AndThen(Movement::GenerateCommand(&m_drive, {4.75_m, 0.0_m, 0_deg}, {}, {0_m, 0.0_m, 0_deg}, true));
}
