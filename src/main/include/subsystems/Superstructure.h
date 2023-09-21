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

  armPose getCurPose();
  void goToPose(armPose pose);
  void resetPose();
  armPose getPose(string name);
  void manualAdjust(double shoulder, double elbow);
  void setGrabber(double speed);

  //intaking positions
  void groundCone();
  void groundCube();
  void singleStation();
  void doubleStation();

  //low
  void hybridNode();

  //mid cone, cube
  void midCone(); // move arm to right above cone pole
  void midCube();

  //high cone, cube
  void highCone(); // move arm to right above cone pole
  void highCube();

  void scoreCone();// place cone down
  



  /*** Example command factory method.*/
  frc2::CommandPtr ExampleMethodCommand();
  /*** An example method querying a boolean state of the subsystem (for example, a* digital sensor).** @return value of some boolean subsystem state, such as a digital sensor.*/
  // bool ExampleCondition();
  /*** Will be called periodically whenever the CommandScheduler runs.*/
  void Periodic() override;
  /*** Will be called periodically whenever the CommandScheduler runs during* simulation.*/
  // void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_shoulderMotorLeft;
  rev::CANSparkMax m_shoulderMotorRight;
  rev::CANSparkMax m_elbowMotor;
  ctre::phoenix::motorcontrol::can::VictorSPX m_grabberMotor;

  // frc::AnalogEncoder m_absoluteShoulderEncoder;
  // frc::AnalogEncoder m_absoluteElbowEncoder;

  // double shoulderOffset;
  // double elbowOffset;

  rev::SparkMaxPIDController m_shoulderController = m_shoulderMotorLeft.GetPIDController();
  rev::SparkMaxPIDController m_elbowController = m_elbowMotor.GetPIDController();

  rev::SparkMaxRelativeEncoder m_relativeShoulderEncoder;
  rev::SparkMaxRelativeEncoder m_relativeElbowEncoder;

  // frc::ArmFeedforward m_shoulderFeedforward();
  // frc::ArmFeedforward m_elbowFeedforward;

  armPose m_pose; // current pose
  int poseID;

  frc::DigitalInput shoulderLimitSwtich;

  std::vector<armPose> poses;

  std::string names[11] = {"stow",
                           "ground cone",
                           "ground cube",
                           "single station",
                           "double station",
                           "mid cone align",
                           "mid cone score",
                           "mid cube",
                           "high cone align",
                           "high cone score",
                           "high cube"};
};


