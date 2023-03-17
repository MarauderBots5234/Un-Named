#include <frc/Solenoid.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <cameraserver/CameraServer.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/ControlType.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <units/math.h>

using namespace frc;
using namespace ctre;
using namespace rev;
using namespace units;
using namespace std;

// Define motor controller CAN IDs
const int kLeftFrontID = 1;
const int kRightFrontID = 2;
const int kLeftRearID = 3;
const int kRightRearID = 4;
const int kExtensionMotorID = 6;
const int kWristRotationMotorID = 7;
const int kWristPivotMotorID = 8;
// Define Pidgin 2.0 CAN ID
const int kPidginID = 15;
// Define solenoid IDs
const int kPCMID = 0;
const int k12vPCMID = 1;
const int kSolenoid1Channel = 0;
const int kSolenoid2Channel = 1;
const int kSolenoid3Channel = 2;
const int kSolenoid4Channel = 0;
const int kSolenoid5Channel = 1;
// Define drive station controller IDs
const int kDriverPort = 0;
const int kGrabberPort = 1;
const double kThrottleCap = 0.6;
const double kGripperCap = 0.15;

// Define motor controller objects
rev::CANSparkMax leftFront{kLeftFrontID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax leftRear{kLeftRearID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax rightFront{kRightFrontID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax rightRear{kRightRearID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax extendArm{kExtensionMotorID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax rotateWrist{kWristRotationMotorID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax pivotWrist{kWristPivotMotorID, rev::CANSparkMax::MotorType::kBrushless};
// Define solenoid objects
frc::Solenoid mechanumsolenoid1{kPCMID, frc::PneumaticsModuleType::CTREPCM, kSolenoid1Channel};
frc::Solenoid mechanumsolenoid2{kPCMID, frc::PneumaticsModuleType::CTREPCM, kSolenoid2Channel};
frc::Solenoid grippersolenoid3{kPCMID, frc::PneumaticsModuleType::CTREPCM, kSolenoid3Channel};
frc::Solenoid liftsolenoid4{k12vPCMID, frc::PneumaticsModuleType::CTREPCM, kSolenoid4Channel};
frc::Solenoid liftsolenoid5{k12vPCMID, frc::PneumaticsModuleType::CTREPCM, kSolenoid5Channel};
// Define Pigeon IMU object
PigeonIMU pigeon{kPidginID};
// Define controller objects
frc::XboxController driverController{kDriverPort};
frc::XboxController grabberController{kGrabberPort};
// Define Limelight object
nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
std::shared_ptr<nt::NetworkTable> limelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
// Define Limelight variables
double targetOffsetAngle_Horizontal = limelightTable -> GetNumber("tx",0.0);
double targetOffsetAngle_Vertical = limelightTable -> GetNumber("ty",0.0);
double targetArea = limelightTable -> GetNumber("ta",0.0);
double targetSkew = limelightTable -> GetNumber("ts",0.0);
double kP = 0.1;
double maxCommand = 0.045;
double minCommand = 0.001;
bool llHasTarget = false;
double llTargetSize = 0.0;
// Define drive mode variables
enum DriveMode{kArcadeDrive, kMecanumDrive};
DriveMode driveMode = kArcadeDrive;
bool togglePressed = false;
// Define gripper mode variables
enum GripperState{kClosed, kOpen};
GripperState gripperState = kClosed;
bool grabberOpen = false;

enum LiftMode{kUp, kDown};
LiftMode LiftState = kUp;
bool armLifted = true;

class Robot : public frc::TimedRobot {
  public:
  void TeleopPeriodic() override {
    rightFront.SetInverted(true);
    rightRear.SetInverted(true);
    // Get Driver controller inputs
    double leftX = driverController.GetRawAxis(0) * kThrottleCap;
    double leftY = driverController.GetRawAxis(1) * kThrottleCap;
    double rightX = driverController.GetRawAxis(4) * kThrottleCap;
    bool toggleButton = driverController.GetLeftStickButton();
    bool targetButton = driverController.GetRightStickButton();
    bool brakeButton = driverController.GetBButton();
    // Get Grabber controller inputs
    double extendArmX = grabberController.GetRawAxis(0);
    double rotateWristX = grabberController.GetRawAxis(4) * kGripperCap;
    double pivotWristY = grabberController.GetRawAxis(5) * kGripperCap;
    bool grabberButton = grabberController.GetBButton();
    bool liftButton = grabberController.GetAButton();

    // Toggle drive mode if toggle button is pressed
    if (toggleButton && !togglePressed) {
      driveMode = (driveMode == kArcadeDrive) ? kMecanumDrive : kArcadeDrive;
      togglePressed = true;
      // Toggle solenoids based on drive mode
      if (driveMode == kArcadeDrive) {
        mechanumsolenoid1.Set(true);
        mechanumsolenoid2.Set(false);
      } else {
        mechanumsolenoid1.Set(false);
        mechanumsolenoid2.Set(true);
      }
    } else if (!toggleButton) {
      togglePressed = false;
    }
    // Set motor controller outputs based on drive mode
    if (driveMode == kArcadeDrive) {
      leftFront.Set(leftY - leftX);
      leftRear.Set(leftY - leftX);
      rightFront.Set(leftY + leftX);
      rightRear.Set(leftY + leftX);
    } else {
      double denominator = units::math::max(std::abs(leftY) + std::abs(-leftX) + std::abs(-rightX),1);
      leftFront.Set((leftY + -leftX + -rightX) / denominator);
      leftRear.Set((leftY - -leftX + -rightX) / denominator);
      rightFront.Set((leftY - -leftX - -rightX) / denominator);
      rightRear.Set((leftY + -leftX - -rightX) / denominator);
    }

    // Target reflective object if target button is pressed
    if (targetButton) {
      double targetOffset = limelightTable->GetNumber("tx", 0.0);
      double steeringAdjust = 0.0;
      if (targetOffset != 0.0) {
        steeringAdjust = kP * targetOffset;
        if (std::abs(steeringAdjust) < minCommand) {
            steeringAdjust = std::copysign(minCommand, steeringAdjust);
        }
        if (std::abs(steeringAdjust) > maxCommand){
            steeringAdjust = std::copysign(maxCommand, steeringAdjust);
        }
      }
      leftFront.Set(-steeringAdjust);
      leftRear.Set(-steeringAdjust);
      rightFront.Set(steeringAdjust);
      rightRear.Set(steeringAdjust);
    }

    // Adjust extension arm
    extendArm.Set(extendArmX);
    // Rotate wrist
    rotateWrist.Set(rotateWristX);
    // Pivot wrist
    pivotWrist.Set(pivotWristY);
    // Actuate grabber
    if (grabberButton && !grabberOpen) {
      gripperState = (gripperState == kClosed) ? kOpen : kClosed;
      grabberOpen = true;
      if (gripperState == kClosed) {
        grippersolenoid3.Set(true);
      } else {
        grippersolenoid3.Set(false);
      }
    } else if (!grabberButton) {
      grabberOpen = false;
    }
    
    if (liftButton && !armLifted) {
      LiftState = (LiftState == kUp) ? kDown : kUp;
      armLifted = true;
      // Toggle solenoids based on drive mode
      if (LiftState == kArcadeDrive) {
        liftsolenoid4.Set(true);
        liftsolenoid5.Set(false);
      } else {
        liftsolenoid4.Set(false);
        liftsolenoid5.Set(true);
      }
    } else if (!liftButton) {
      armLifted = false;
    }
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif