// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax>

class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(int driveMotorPort, int turningMotorPort, int encoderPort, double offset);

  void resetModule();
  void resetDriveMotor();
  void resetTurningMotor();
  void resetDriveEncoder();
  void resetTurningEncoder();
  double getDrivePosition();
  double getTurningPosition();
  double getDriveVelocity();
  double getTurningVelocity();
  

  /**
   * Example command factory method.
   */
  frc2::CommandPtr ExampleMethodCommand();






  bool ExampleCondition();  /*** An example method querying a boolean state of the subsystem (for example, a* digital sensor).** @return value of some boolean subsystem state, such as a digital sensor.*/
  void Periodic() override;  /*** Will be called periodically whenever the CommandScheduler runs.*/
  void SimulationPeriodic() override;  /*** Will be called periodically whenever the CommandScheduler runs during* simulation.*/

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  double encoffset;

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::SparkMaxRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_turningEncoder = m_turningMotor.GetEncoder();

  rev::SparkMaxPIDController m_turningController = m_turningMotor.GetPIDController();
  rev::SparkMaxPIDController m_driveController = m_driveMotor.GetPIDController();

  frc::AnalogInput m_absoluteEncoder;






};
