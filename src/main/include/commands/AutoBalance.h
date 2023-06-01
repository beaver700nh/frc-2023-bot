#pragma once

#include <units/length.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Drive.h"

class RunToRamp : public frc2::CommandHelper<frc2::CommandBase, RunToRamp> {
public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit RunToRamp(Drive *drive, double power, units::angle::degree_t minRegAngle, int reqTimesInAngle);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  Drive *m_drive;
  bool m_wasDriverControlled;
  const double m_power;
  const units::angle::degree_t m_minRegAngle;
  const int m_reqTimesInAngle;
  int m_timesInAngle;
};

class BalanceOnRamp : public frc2::CommandHelper<frc2::CommandBase, BalanceOnRamp> {
public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit BalanceOnRamp(Drive *drive, double speed, units::angle::degree_t minMovementAngle, units::angle::degree_t minRegAngle, int reqTimesInAngle);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  Drive *m_drive;
  bool m_wasDriverControlled;
  const double m_speed;
  const units::angle::degree_t m_minMovementAngle;
  const units::angle::degree_t m_minRegAngle;
  const int m_reqTimesInAngle;
  int m_timesInAngle;
};