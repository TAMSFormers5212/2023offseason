// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/SwerveModule.h"
#include <AHRS.h>
#include <Constants.hpp>
#include <frc/StateSpaceUtil.h>
#include <frc/Timer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/smartdashboard/Field2d.h>

class Swerve : public frc2::SubsystemBase {
 public:
  Swerve();

  frc::Pose2d AveragePose(); //vision and odometry pose

  frc::Pose2d OdometryPose(); //odometry pose

  frc::Rotation2d getGyroHeading();

  void resetOdometry(const frc::Pose2d pose);

  void swerveDrive(double x, double y, double theta, bool fieldCentric, bool team);

  void brake();

  void Periodic() override; //update pose using gyro, vision, and odometry

  /*** Example command factory method.*/frc2::CommandPtr ExampleMethodCommand();
  /*** An example method querying a boolean state of the subsystem (for example, a* digital sensor).** @return value of some boolean subsystem state, such as a digital sensor.*/bool ExampleCondition();
  /*** Will be called periodically whenever the CommandScheduler runs.*void Periodic() override;*/
  /*** Will be called periodically whenever the CommandScheduler runs during* simulation.*/void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
