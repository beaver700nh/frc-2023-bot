#include "iostream"

#include "commands/AutoBalance.h"

RunToRamp::RunToRamp(Drive *drive, double power, units::angle::degree_t minRegAngle, int reqTimesInAngle) : 
m_drive(drive), m_power(power), m_minRegAngle(minRegAngle), m_reqTimesInAngle(reqTimesInAngle){
  AddRequirements(drive);
}

void RunToRamp::Initialize(){
  m_wasDriverControlled = m_drive->m_controllerControllable;
  m_drive->m_controllerControllable = false;

  m_drive->SetPower(m_power, 0);
}

void RunToRamp::Execute(){
  // std::cout << "running to ramp with power " << m_power << "\n";

  m_drive->SetPower(m_power, 0);

  if(std::abs(m_drive->GetPitch().to<double>()) >= m_minRegAngle.to<double>()){
    m_timesInAngle ++;
  } else {
    m_timesInAngle = 0;
  }
}

void RunToRamp::End(bool interrupted){
  m_drive->m_controllerControllable = m_wasDriverControlled;
  
  m_drive->SetPower(0,0);
}

bool RunToRamp::IsFinished(){
  return m_timesInAngle >= m_reqTimesInAngle;
}


BalanceOnRamp::BalanceOnRamp(Drive *drive, double speed, units::angle::degree_t minMovementAngle, units::angle::degree_t minRegAngle, int reqTimesInAngle) : 
m_drive(drive),m_speed(speed), m_minMovementAngle(minMovementAngle), m_minRegAngle(minRegAngle), m_reqTimesInAngle(reqTimesInAngle){
  AddRequirements(drive);
}

void BalanceOnRamp::Initialize(){
  m_wasDriverControlled = m_drive->m_controllerControllable;
  m_drive->m_controllerControllable = false;

  m_drive->MoveUsingPosition(0, 0);
}

void BalanceOnRamp::Execute(){
  // std::cout << "balancing on ramp \n";

  units::degree_t pitch = m_drive->GetPitch();
  if(pitch <= -m_minMovementAngle)
    m_drive->MoveUsingPosition(-m_speed, 0);
  else if(pitch >= m_minMovementAngle)
    m_drive->MoveUsingPosition(m_speed, 0);
  
  if(pitch > -m_minRegAngle && pitch < m_minRegAngle)
    m_timesInAngle ++;
  else
    m_timesInAngle = 0;
}

void BalanceOnRamp::End(bool interrupted){
  m_drive->m_controllerControllable = m_wasDriverControlled;
  
  m_drive->MoveUsingPosition(0,0);
}

bool BalanceOnRamp::IsFinished(){
  return false;
}