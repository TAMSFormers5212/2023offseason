// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>
#include <frc/AnalogInput.h>
#include <frc/AnalogEncoder.h>
#include <frc/controller/ArmFeedforward.h>
#include "armPose.h"
#include "Constants.h"

class Superstructure : public frc2::SubsystemBase {
 public:
  Superstructure(int shoulderLeftPort, int shoulderRightPort, int elbowPort, int shoulderEncoderPort, int elbowEncoderPort);

  armPose getPose();
  void setPose();

  
  



  /*** Example command factory method.*/
  frc2::CommandPtr ExampleMethodCommand();
  /*** An example method querying a boolean state of the subsystem (for example, a* digital sensor).** @return value of some boolean subsystem state, such as a digital sensor.*/
  bool ExampleCondition();
  /*** Will be called periodically whenever the CommandScheduler runs.*/
  void Periodic() override;
  /*** Will be called periodically whenever the CommandScheduler runs during* simulation.*/
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_shoulderMotorLeft;
  rev::CANSparkMax m_shoulderMotorRight;
  rev::CANSparkMax m_elbowMotor;

  frc::AnalogEncoder m_absoluteShoulderEncoder;
  frc::AnalogEncoder m_absoluteElbowEncoder;

  rev::SparkMaxPIDController m_shoulderController = m_shoulderMotorLeft.GetPIDController();
  rev::SparkMaxPIDController m_elbowController = m_elbowMotor.GetPIDController();

  rev::SparkMaxRelativeEncoder m_relativeShoulderEncoder;
  rev::SparkMaxRelativeEncoder m_relativeElbowEncoder;

  // frc::ArmFeedforward m_shoulderFeedforward();
  // frc::ArmFeedforward m_elbowFeedforward;

  armPose m_pose;




};


