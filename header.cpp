#include <photonlib/PhotonCamera.h>

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <units/angle.h>
#include <units/length.h>

class Robot : public frc::TimedRobot {
 public:
  void TeleopPeriodic() override;

 private:
  // Constants such as camera and target height stored. Change per robot and
  // goal!
  const units::meter_t CAMERA_HEIGHT = 24_in;
  const units::meter_t TARGET_HEIGHT = 5_ft;

  // Angle between horizontal and the camera.
  const units::radian_t CAMERA_PITCH = 0_deg;

  // How far from the target we want to be
  const units::meter_t GOAL_RANGE_METERS = 3_ft;

  // PID constants should be tuned per robot
  const double LINEAR_P = 0.1;
  const double LINEAR_D = 0.0;
  frc2::PIDController forwardController{LINEAR_P, 0.0, LINEAR_D};

  const double ANGULAR_P = 0.1;
  const double ANGULAR_D = 0.0;
  frc2::PIDController turnController{ANGULAR_P, 0.0, ANGULAR_D};

  // Change this to match the name of your camera
  photonlib::PhotonCamera camera{"photonvision"};

  frc::XboxController xboxController{0};

  // Drive motors
  frc::PWMVictorSPX leftMotor{0};
  frc::PWMVictorSPX rightMotor{1};
  frc::DifferentialDrive drive{leftMotor, rightMotor};
};