// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"n

void Robot::RobotInit() {
  //frc::Encoder encoder{0, 1, false, frc::Encoder::EncodingType::k1X};
  //frc::Encoder encoder{0, 1, false, frc::Encoder::EncodingType::k2X};
  frc::Encoder encoder{0, 1, false, frc::Encoder::EncodingType::k4X};
  //frc::Encoder encoder{0, 1};

}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  //encoder.SetMaxPeriod(.1);
  //encoder.SetMinRate(10);
  //encoder.SetDistancePerPulse(4./256.);
  leftEncoder.SetDistancePerPulse(1./256.);
  rightEncoder.SetDistancePerPulse(1./256.);
}
void Robot::AutonomousPeriodic() {
  // Assuming no wheel slip, the difference in encoder distances is proportional to the heading error
    double error = leftEncoder.GetDistance() - rightEncoder.GetDistance();

    // Drives forward continuously at half speed, using the encoders to stabilize the heading
    drive.TankDrive(.5 + kP * error, .5 - kP * error);
}


void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
