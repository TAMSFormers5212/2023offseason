// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
// #include <frc/AnalogInput.h>
// #include <frc/AnalogEncoder.h>
// #include <frc/controller/ArmFeedforward.h>
#include <frc/DigitalInput.h>
// #include <frc/motorcontrol/

#include <ctre/Phoenix.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>

#include <string>

#include "armPose.h"
#include "Constants.h"

class Superstructure : public frc2::SubsystemBase {
 public:
  virtual ~Superstructure();
  Superstructure();

  armPose getCurPose(); // get current position of arm
  void goToPose(armPose pose); // got to arm pose
  void resetPose(); // set arm pose to current pose
  void setPose(armPose pose); // set goal pose to new pose
  armPose getPose();
  void manualAdjust(double shoulder, double elbow); // adjust arm with joysticks
  void setGrabber(double speed); // spin intake
  void resetEncoders(); // set pose to 0,0

  void setManual(bool man);
  bool getManual();
  void setSoftLimitEnabled(bool enable);
  bool getSoftLimitEnabled();




  /*** Example command factory method.*/
//   frc2::CommandPtr ExampleMethodCommand();
  /*** An example method querying a boolean state of the subsystem (for example, a* digital sensor).** @return value of some boolean subsystem state, such as a digital sensor.*/
  // bool ExampleCondition();
  /*** Will be called periodically whenever the CommandScheduler runs.*/
  void Periodic() override;
  /*** Will be called periodically whenever the CommandScheduler runs during* simulation.*/
  // void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  // rev::CANSparkMax m_shoulderMotorLeft;
  rev::CANSparkMax m_shoulderMotorRight;
  rev::CANSparkMax m_elbowMotor;
  ctre::phoenix::motorcontrol::can::VictorSPX m_grabberMotor;

  // frc::AnalogEncoder m_absoluteShoulderEncoder;
  // frc::AnalogEncoder m_absoluteElbowEncoder;

  // double shoulderOffset;
  // double elbowOffset;

  rev::SparkMaxPIDController m_shoulderController = m_shoulderMotorRight.GetPIDController();
  rev::SparkMaxPIDController m_elbowController = m_elbowMotor.GetPIDController();

  rev::SparkMaxRelativeEncoder m_relativeShoulderEncoder;
  rev::SparkMaxRelativeEncoder m_relativeElbowEncoder;

  // frc::ArmFeedforward m_shoulderFeedforward();
  // frc::ArmFeedforward m_elbowFeedforward;

  armPose m_pose; // current pose
  int poseID;

  bool manual = false;
  bool softLimitEnabled = true;

  // frc::DigitalInput shoulderLimitSwtich;
  // frc::DigitalInput elbowLimitSwitch;

};


