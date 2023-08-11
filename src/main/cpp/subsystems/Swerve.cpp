#include "subsystems/Swerve.h"



using namespace SwerveModuleConstants;
using namespace units;


  Swerve::Swerve()
  : m_modules{{SwerveModule(topleft::driveMotor, topleft::turningMotor, topleft::absencoder, topleft::offset), 
               SwerveModule(topright::driveMotor, topright::turningMotor, topright::absencoder, topright::offset),
               SwerveModule(bottomleft::driveMotor, bottomleft::turningMotor, bottomleft::absencoder, bottomright::offset),
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
    heading = frc::Rotation2d(degree_t{-m_gyro.GetYaw()});
    lastAngle = -m_gyro.GetYaw();
  }

  frc::Pose2d Swerve::AveragePose(){
    return m_poseEstimator.GetEstimatedPosition();
  } //vision and odometry pose

  frc::Pose2d Swerve::OdometryPose(){
    return m_odometry.GetPose();
  } //odometry pose

  frc::Rotation2d Swerve::getGyroHeading(){
    double newAngle = -m_gyro.GetYaw();
    double delta =
      std::fmod(std::fmod((newAngle - lastAngle + 180), 360) + 360, 360) -
      180;  // NOLINT
    lastAngle = newAngle;
    heading = heading + frc::Rotation2d(degree_t{delta * 1.02466666667});
    //SmartDashboard::PutNumber("Raw angle", newAngle);
    //SmartDashboard::PutNumber("Fused angle", -m_gyro.GetFusedHeading());
    //SmartDashboard::PutNumber("Angle", angle.Degrees().value());
    return heading;
  }

  void Swerve::resetOdometry(const frc::Pose2d pose){
    
  }

  void Swerve::swerveDrive(double x, double y, double theta, bool fieldCentric, bool team){

  }

  void Swerve::brake(){

  }

  void Swerve::Periodic(){

  } //update pose using gyro, vision, and odometry

  void Swerve::resetAbsoluteEncoders(){

  }

  void Swerve::SyncAbsoluteEncoders(){

  }

  void Swerve::SetVisionBeginning(){

  }
