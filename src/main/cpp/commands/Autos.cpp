// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"
#include "commands/AutoBalance.h"

#include <frc2/command/Commands.h>


namespace autos {

std::vector<std::pair<std::string, frc2::CommandPtr>> autos;

void populate(Arm *arm, Drive *drive, Pneumatics *pneu) {
  autos.emplace_back(
    "2 cube low & high (CCW)",
    frc2::cmd::Sequence(
      Movement::GenerateCommand(drive, {-0.6_m, 0.0_m, 0.0_deg}, true),
      ClawControl(pneu, true),
      arm->m_position_pickup,
      //
      Movement::GenerateCommand(drive, {5.2_m, 0.0_m, 0.0_deg}),
      //close the claw
      ClawControl(pneu, false),
      //wait .5 seconds
      frc2::WaitCommand(500_ms),
      //Set arm + lock in place
      SetArmPositionWait(arm, {25, std::nullopt, 0}),
      //Set arm then move on (does not wait for arm to be in position)
      SetArmPosition(arm, {std::nullopt, -120.0, std::nullopt}),
      //Drives backward 5.1 meters(the trus tells it to drive backwards)
      Movement::GenerateCommand(drive, {-5.1_m, 0_m, 0_deg}, true),
      arm->m_position_hiCubeBack,
      //open claw
      ClawControl(pneu, true)
    )
  );

    autos.emplace_back(
    "2 cube low & high (CW)",
    frc2::cmd::Sequence(
      Movement::GenerateCommand(drive, {-0.6_m, 0.0_m, 0.0_deg}, true),
      ClawControl(pneu, true),
      arm->m_position_pickup,
      //
      Movement::GenerateCommand(drive, {5.2_m, 0.0_m, 0.0_deg}),
      //close the claw
      ClawControl(pneu, false),
      //wait .5 seconds
      frc2::WaitCommand(500_ms),
      //Set arm + lock in place
      SetArmPositionWait(arm, {25, std::nullopt, 0}),
      //Set arm then move on (does not wait for arm to be in position)
      SetArmPosition(arm, {std::nullopt, 120.0, std::nullopt}),
      //Drives backward 5.1 meters(the trus tells it to drive backwards)
      Movement::GenerateCommand(drive, {-5.1_m, 0_m, 0_deg}, true),
      arm->m_position_hiCubeBack,
      //open claw
      ClawControl(pneu, true)
    )
  );

  autos.emplace_back(
    "2 cube high & mid (CCW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, -120, std::nullopt}),
      arm->m_position_hiCubeBack,
      ClawControl(pneu, true),
      frc2::WaitCommand(500_ms),
      arm->m_position_up,
      frc2::cmd::Parallel(
        frc2::cmd::Sequence(
          SetArmPositionWait(arm, {std::nullopt, 0, std::nullopt}),
          arm->m_position_pickupFront_wait
        ),
        Movement::GenerateCommand(drive, {5.2_m, 0.0_m, 0.0_deg})
      ),
      ClawControl(pneu, false),
      frc2::WaitCommand(500_ms),
      arm->m_position_up,
      frc2::cmd::Parallel(
        frc2::cmd::Sequence(
          SetArmPositionWait(arm, {std::nullopt, -125.0, std::nullopt}),
          arm->m_position_loCubeBack
        ),
        Movement::GenerateCommand(drive, {-5.2_m, 0.2_m, -3.0_deg}, true)
      ),
      ClawControl(pneu, true)
    )
  );

    autos.emplace_back(
    "2 cube high & mid (CW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, 120, std::nullopt}),
      arm->m_position_hiCubeBack,
      ClawControl(pneu, true),
      frc2::WaitCommand(500_ms),
      arm->m_position_up,
      frc2::cmd::Parallel(
        frc2::cmd::Sequence(
          SetArmPositionWait(arm, {std::nullopt, 0, std::nullopt}),
          arm->m_position_pickupFront_wait
        ),
        Movement::GenerateCommand(drive, {5.2_m, 0.0_m, 0.0_deg})
      ),
      ClawControl(pneu, false),
      frc2::WaitCommand(500_ms),
      arm->m_position_up,
      frc2::cmd::Parallel(
        frc2::cmd::Sequence(
          SetArmPositionWait(arm, {std::nullopt, 125.0, std::nullopt}),
          arm->m_position_loCubeBack
        ),
        Movement::GenerateCommand(drive, {-5.2_m, -0.2_m, 3.0_deg}, true)
      ),
      ClawControl(pneu, true)
    )
  );

  autos.emplace_back(
    "1 cone high (CCW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, -125, std::nullopt}),
      arm->m_position_hiConeBack,
      SetArmPositionWait(arm, {35, std::nullopt, std::nullopt}),
      ClawControl(pneu, true),
      frc2::WaitCommand(1000_ms),
      SetArmPositionWait(arm, {std::nullopt, std::nullopt, 0}),
      arm->m_position_up,
      SetArmPosition(arm, {0,0,0})
      //Movement::GenerateCommand(drive, {5_m, 0.0_m, 0.0_deg})
    )
  );

    autos.emplace_back(
    "1 cone high (CW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, 125, std::nullopt}),
      arm->m_position_hiConeBack,
      SetArmPositionWait(arm, {35, std::nullopt, std::nullopt}),
      ClawControl(pneu, true),
      frc2::WaitCommand(1000_ms),
      SetArmPositionWait(arm, {std::nullopt, std::nullopt, 0}),
      arm->m_position_up,
      SetArmPosition(arm, {0,0,0})
      //Movement::GenerateCommand(drive, {5_m, 0.0_m, 0.0_deg})
    )
  );

  autos.emplace_back(
    "1 cone mid (CCW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, -125, std::nullopt}),
      arm->m_position_loConeBack,
      SetArmPositionWait(arm, {std::nullopt, std::nullopt, 50}),
      arm->m_position_loConeBack,
      frc2::WaitCommand(1000_ms),
      ClawControl(pneu, true),
      //frc2::WaitCommand(1000_ms),
      arm->m_position_up,
      SetArmPosition(arm, {0,0,0}),
      Movement::GenerateCommand(drive, {5_m, 0.0_m, 0.0_deg})
    )
  );

  autos.emplace_back(
    "1 cone mid (CW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, 125, std::nullopt}),
      arm->m_position_loConeBack,
      SetArmPositionWait(arm, {std::nullopt, std::nullopt, 50}),
      arm->m_position_loConeBack,
      frc2::WaitCommand(1000_ms),
      ClawControl(pneu, true),
      //frc2::WaitCommand(1000_ms),
      arm->m_position_up,
      SetArmPosition(arm, {0,0,0}),
      Movement::GenerateCommand(drive, {5_m, 0.0_m, 0.0_deg})
    )
  );

  //   autos.emplace_back(
  //   "2 cube high & mid (CW)(untested)",
  //   frc2::cmd::Sequence(
  //     SetArmPositionWait(arm, {std::nullopt, 120, std::nullopt}),
  //     arm->m_position_hiCubeBack,
  //     ClawControl(pneu, true),
  //     frc2::WaitCommand(1000_ms),
  //     arm->m_position_up,
  //     SetArmPositionWait(arm, {std::nullopt, 0, std::nullopt}),
  //     arm->m_position_pickupFront,
  //     Movement::GenerateCommand(drive, {4.9_m, 0.0_m, 0.0_deg}),
  //     ClawControl(pneu, false),
  //     frc2::WaitCommand(1000_ms),
  //     arm->m_position_up,
  //     SetArmPositionWait(arm, {std::nullopt, 125.0, std::nullopt}),
  //     arm->m_position_loCubeBack,
  //     Movement::GenerateCommand(drive, {-4.9_m, 0.2_m, -3.0_deg}, true),
  //     ClawControl(pneu, true)
  //   )
  // );

  autos.emplace_back(
    "1 cube low",
    frc2::cmd::Sequence(
      Movement::GenerateCommand(drive, {-0.6_m, 0.0_m, 0.0_deg}, true),
      Movement::GenerateCommand(drive, {5_m, 0.0_m, 0.0_deg})
    )
  );

  autos.emplace_back(
    "no move(home)",
    frc2::cmd::Sequence(
        HomeArmExtend(arm).ToPtr(),
        HomeArmTilt(arm).ToPtr(),
        HomeArmRotate(arm).ToPtr()
      )
  );

  autos.emplace_back(
    "1 cube high (CCW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, -125, std::nullopt}),
      arm->m_position_hiCubeBack,
      ClawControl(pneu, true),
      frc2::WaitCommand(1000_ms),
      SetArmPositionWait(arm, {0,std::nullopt,0}),
      SetArmPositionWait(arm, {0,0,0}),
      Movement::GenerateCommand(drive, {5_m, 0.0_m, 0.0_deg})
    )
  );

    autos.emplace_back(
    "1 cube high (CW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, 125, std::nullopt}),
      arm->m_position_hiCubeBack,
      ClawControl(pneu, true),
      frc2::WaitCommand(1000_ms),
      SetArmPositionWait(arm, {0,std::nullopt,0}),
      SetArmPositionWait(arm, {0,0,0}),
      Movement::GenerateCommand(drive, {5_m, 0.0_m, 0.0_deg})
    )
  );

  autos.emplace_back(
    "1 cube high and auto balance (CCW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, -125, std::nullopt}),
      arm->m_position_hiCubeBack,
      ClawControl(pneu, true),
      frc2::WaitCommand(1000_ms),
      SetArmPosition(arm, {std::nullopt, std::nullopt, 0}),
      SetArmPositionWait(arm, {25,std::nullopt,std::nullopt}),
      SetArmPosition(arm, {0,0,0}),
      Movement::GenerateCommand(drive, {2.5_m, 0_m, 0_deg}),
      //RunToRamp(drive, 0.5, 10_deg, 10),
      BalanceOnRamp(drive, .3, 10_deg, 10_deg, 10)
    )
  );

  autos.emplace_back(
    "1 cube high and auto balance (CW)",
    frc2::cmd::Sequence(
      SetArmPositionWait(arm, {std::nullopt, 125, std::nullopt}),
      arm->m_position_hiCubeBack,
      ClawControl(pneu, true),
      frc2::WaitCommand(1000_ms),
      SetArmPosition(arm, {std::nullopt, std::nullopt, 0}),
      SetArmPositionWait(arm, {25,std::nullopt,std::nullopt}),
      SetArmPosition(arm, {0,0,0}),
      Movement::GenerateCommand(drive, {2.5_m, 0_m, 0_deg}),
      //RunToRamp(drive, 0.5, 10_deg, 10),
      BalanceOnRamp(drive, .3, 10_deg, 10_deg, 10)
    )
  );

  autos.emplace_back(
    "1 cube high and auto balance TESTING!!",
    frc2::cmd::Sequence(
      arm->m_position_hiCubeFront,
      ClawControl(pneu, true),
      frc2::WaitCommand(500_ms),
      SetArmPosition(arm, {std::nullopt, std::nullopt, 0}),
      SetArmPositionWait(arm, {25,std::nullopt,std::nullopt}),
      SetArmPosition(arm, {0,0,0}),
      Movement::GenerateCommand(drive, {-4.5_m, 0_m, 0_deg}, true, 1.3_mps),
      Movement::GenerateCommand(drive, {2.75_m, 0_m, 0_deg}),
      BalanceOnRamp(drive, .3, 10_deg, 10_deg, 10)
    )
  );

  autos.emplace_back(
    "test balance",
    frc2::cmd::Sequence(
      //SetArmPositionWait(arm, {std::nullopt, 125, std::nullopt}),
      //arm->m_position_hiCubeBack,
      //ClawControl(pneu, true),
      //frc2::WaitCommand(1000_ms),
      //SetArmPosition(arm, {std::nullopt, std::nullopt, 0}),
      //SetArmPositionWait(arm, {25,std::nullopt,std::nullopt}),
      SetArmPosition(arm, {0,0,0}),
      Movement::GenerateCommand(drive, {2_m, 0_m, 0_deg}),
      //RunToRamp(drive, 0.5, 10_deg, 10),
      BalanceOnRamp(drive, .3, 10_deg, 10_deg, 10)
    )
  );
}

}