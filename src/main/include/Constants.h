// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>

#define POV_UP    0
#define POV_RIGHT 90
#define POV_DOWN  180
#define POV_LEFT  270

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {
  constexpr int kDriverControllerAPort = 0;
  constexpr int kDriverControllerBPort = 1;

  constexpr bool kArmInvertTilt   = true;
  constexpr bool kArmInvertRotate = true;
  constexpr bool kArmInvertExtend = false;

  constexpr bool kDriveInvertL = false;
  constexpr bool kDriveInvertR = true;
  constexpr double kDriveRampX = 0.04;
  constexpr double kDriveRampR = 0.04;

  constexpr int kDriveEncoderTicksPerRev = 2048;
  constexpr double kDriveHighGearRatio = 20.833; // ratio for slow speed
  constexpr double kDriveLowGearRatio = 9.167; // ratio for fast speed
  constexpr auto kDriveWheelDiameter = 6.0_in;

  constexpr auto kDriveBaseEncoderDistancePerPulse = units::meter_t(kDriveWheelDiameter * M_PI / kDriveEncoderTicksPerRev);
}

namespace CanIds {
  constexpr int kDriveL1 = 1;
  constexpr int kDriveL2 = 2;
  constexpr int kDriveR1 = 3;
  constexpr int kDriveR2 = 4;

  constexpr int kArmTilt   = 6;
  constexpr int kArmRotate = 7;
  constexpr int kArmExtend = 8;

  constexpr int kPneuCtrlHub = 11;
}

namespace PortsDIO {
  constexpr int kArmLmswTilt   = 0;
  constexpr int kArmLmswRotate = 1;
  constexpr int kArmLmswExtend = 2;
}

namespace PortsPCH {
  constexpr int kPneuSlndShoe1 = 2;
  constexpr int kPneuSlndShoe2 = 10;
  constexpr int kPneuSlndClaw1 = 3;
  constexpr int kPneuSlndClaw2 = 11;
  constexpr int kPneuSlndGear1 = 7;
  constexpr int kPneuSlndGear2 = 15;
}
