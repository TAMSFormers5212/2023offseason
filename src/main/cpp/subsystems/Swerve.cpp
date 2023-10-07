#include "subsystems/Swerve.h"
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>



using namespace SwerveModuleConstants;
using namespace units;



  Swerve::Swerve()
  : m_modules{{SwerveModule(topleft::driveMotor, topleft::turningMotor, topleft::absencoder, topleft::offset), 
               SwerveModule(topright::driveMotor, topright::turningMotor, topright::absencoder, topright::offset),
               SwerveModule(bottomleft::driveMotor, bottomleft::turningMotor, bottomleft::absencoder, bottomleft::offset),
               SwerveModule(bottomright::driveMotor, bottomright::turningMotor, bottomright::absencoder, bottomright::offset)
    }},
    m_driveKinematics{{frc::Translation2d{drivebase::WheelBase/2, drivebase::TrackWidth/2},
                       frc::Translation2d{drivebase::WheelBase/2, -drivebase::TrackWidth/2},
                       frc::Translation2d{-drivebase::WheelBase/2, drivebase::TrackWidth/2},
                       frc::Translation2d{-drivebase::WheelBase/2,-drivebase::TrackWidth/2}
    }},
    m_odometry{m_driveKinematics, 
               frc::Rotation2d(getGyroHeading()), 
               {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()},
               frc::Pose2d()
    },
    m_poseEstimator{m_driveKinematics,
                    frc::Rotation2d(getGyroHeading()),
                    {m_modules[0].getPosition(), m_modules[1].getPosition(), m_modules[2].getPosition(), m_modules[3].getPosition()},
                    frc::Pose2d()
    }
  {
    m_gyro.Calibrate();
    m_gyro.ZeroYaw();
    heading = frc::Rotation2d(degree_t{-m_gyro.GetYaw()});
    lastAngle = -m_gyro.GetYaw();
    resetOdometry(m_poseEstimator.GetEstimatedPosition());
    std::cout<<"Swerve subsystem initalized correctly"<<std::endl;
  }

  frc::Pose2d Swerve::AveragePose(){
    return m_poseEstimator.GetEstimatedPosition();
  } //vision and odometry pose

  frc::Pose2d Swerve::OdometryPose(){
    return m_odometry.GetPose();
  } //odometry pose

  frc::Rotation2d Swerve::getGyroHeading(){//i have no f*cking clue how this works
    double newAngle = -m_gyro.GetYaw();
    double delta =
      std::fmod(std::fmod((newAngle - lastAngle + 180), 360) + 360, 360) -
      180;  // NOLINT
    lastAngle = newAngle;
    heading = heading + frc::Rotation2d(degree_t{delta * 1.02466666667});
    return heading;
  }

  void Swerve::resetHeading(){
    m_gyro.Reset();
  }

  void Swerve::resetOdometry(const frc::Pose2d pose){
    m_odometry.ResetPosition(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), 
                             m_modules[2].getPosition(), m_modules[3].getPosition()}, pose);
    m_poseEstimator.ResetPosition(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), 
                                  m_modules[2].getPosition(), m_modules[3].getPosition()}, pose);
  }

  void Swerve::swerveDrive(double x, double y, double theta, bool fieldCentric){
    frc::ChassisSpeeds speeds = fieldCentric ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                                x * SwerveModuleConstants::maxSpeed, 
                                                y * SwerveModuleConstants::maxSpeed,
                                                theta * SwerveModuleConstants::maxRotation,
                                                units::degree_t{-m_gyro.GetYaw()}
                                                ) 
                                             :  frc::ChassisSpeeds{
                                                x * SwerveModuleConstants::maxSpeed,
                                                y * SwerveModuleConstants::maxSpeed, 
                                                theta * SwerveModuleConstants::maxRotation
                                             };

    auto states = m_driveKinematics.ToSwerveModuleStates(speeds);
    //scale the values based on the largest one
    // double scale = std::max({x, y, theta});
    // //adjust speeds
    // auto maxWheelSpeed = 0.0_mps;
    // for (auto& moduleState : states) { // find max wheel speeds
    //   maxWheelSpeed = meters_per_second_t{std::max(
    //       maxWheelSpeed.value(), std::abs(moduleState.speed.value()))};
    // }

    // if (maxWheelSpeed.value() != 0.0 && // scale wheel speeds accordingly
    //   (maxWheelSpeed / scale).value() > SwerveModuleConstants::maxSpeed.value()) {
    // for (auto moduleState : states) {
    //   moduleState.speed *= scale * SwerveModuleConstants::maxSpeed / maxWheelSpeed;
    // }
    for(size_t i  = 0; i<states.size(); ++i){
      // if(i==2){
      //   frc::SmartDashboard::PutNumber(i+" speed", (double)states[i]);
      // }
      m_modules[i].setState(states[i]);
      // m_modules[i].setState(states[i], openloop);
    }
  // }



  }

  void Swerve::brake(){
    for (auto& module : m_modules) {
      module.setState(frc::SwerveModuleState{0.0_mps, module.getState().angle});
    }
  }

  void Swerve::Periodic(){
    m_odometry.Update(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), 
                                         m_modules[2].getPosition(), m_modules[3].getPosition()});
    m_poseEstimator.Update(getGyroHeading(), {m_modules[0].getPosition(), m_modules[1].getPosition(), 
                                              m_modules[2].getPosition(), m_modules[3].getPosition()});

    auto pose = m_poseEstimator.GetEstimatedPosition();
    // frc::SmartDashboard::PutNumber("Drive/Pose Estimate/X", pose.X().value());
    // frc::SmartDashboard::PutNumber("Drive/Pose Estimate/Y", pose.Y().value());
    // frc::SmartDashboard::PutNumber("Drive/Pose Estimate/Theta",pose.Rotation().Radians().value());

    // frc::SmartDashboard::PutNumber("Top Left Module Value", m_modules[0].getPosition().angle());
    // frc::SmartDashboard::PutNumber("Top Left Value", m_modules[0].getTurningPosition());
    // frc::SmartDashboard::PutNumber("Top Left Value v2", m_modules[0].getAbsolutePosition());
    frc::SmartDashboard::PutNumber("Heading", -m_gyro.GetYaw());
    frc::SmartDashboard::PutNumber("Pose X", (double) AveragePose().X());
    frc::SmartDashboard::PutNumber("Pose Y", (double) AveragePose().Y());

    frc::SmartDashboard::PutNumber("tl rotation", m_modules[0].getTurningPosition());
    frc::SmartDashboard::PutNumber("tr rotation", m_modules[1].getTurningPosition());
    frc::SmartDashboard::PutNumber("bl rotation", m_modules[2].getTurningPosition());
    frc::SmartDashboard::PutNumber("br rotation", m_modules[3].getTurningPosition());
    frc::SmartDashboard::PutNumber("tl apos", m_modules[0].getAbsolutePosition());
    frc::SmartDashboard::PutNumber("tr apos", m_modules[1].getAbsolutePosition());
    frc::SmartDashboard::PutNumber("bl apos", m_modules[2].getAbsolutePosition());
    frc::SmartDashboard::PutNumber("br apos", m_modules[3].getAbsolutePosition());
    // frc::SmartDashboard::PutNumber("bl t velocity", m_modules[2].getTurningVelocity());
    // frc::SmartDashboard::PutNumber("tl t velocity", m_modules[0].getTurningVelocity());
    frc::SmartDashboard::PutNumber("tl speed", m_modules[0].getDriveVelocity());
    frc::SmartDashboard::PutNumber("tr speed", m_modules[1].getDriveVelocity());
    frc::SmartDashboard::PutNumber("bl speed", m_modules[2].getDriveVelocity());
    frc::SmartDashboard::PutNumber("br speed", m_modules[3].getDriveVelocity());
    

  } //update pose using gyro, vision, and odometry

  void Swerve::resetAbsoluteEncoders(){
    for(auto& module : m_modules){
      module.resetDriveEncoder();
      module.resetTurningEncoder();
    }
  }

  void Swerve::SyncAbsoluteEncoders(){
    for(auto& module : m_modules){
      module.resetTurningEncoder();
    }
  }

  void Swerve::SetVisionBeginning(){
    //idk
  }
