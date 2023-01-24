#include "Robot.h"

#include <photonlib/PhotonUtils.h>

void Robot::TeleopPeriodic() {
  double forwardSpeed;
  double rotationSpeed;

  if (xboxController.GetAButton()) {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    const auto& result = camera.GetLatestResult();

    if (result.HasTargets()) {
      // First calculate range
      units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
          units::degree_t{result.GetBestTarget().GetPitch()});

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_
      // range
      forwardSpeed = -forwardController.Calculate(range.value(),
                                                  GOAL_RANGE_METERS.value());

      // Also calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed =
          -turnController.Calculate(result.GetBestTarget().GetYaw(), 0);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
      rotationSpeed = 0;
    }
  } else {
    // Manual Driver Mode
    forwardSpeed = -xboxController.GetRightY();
    rotationSpeed = xboxController.GetLeftX();
  }

  // Use our forward/turn speeds to control the drivetrain
  drive.ArcadeDrive(forwardSpeed, rotationSpeed);
}