#include "subsystems/Superstructure/Superstructure.h"

using enum rev::CANSparkMaxLowLevel::MotorType;
using namespace rev;
using namespace ArmConstants;

Superstructure::Superstructure(int shoulderLeftPort, int shoulderRightPort, int elbowPort, int shoulderEncoderPort, int elbowEncoderPort)
  : m_shoulderMotorLeft(shoulderLeftPort, kBrushless),
    m_shoulderMotorRight(shoulderRightPort, kBrushless),
    m_elbowMotor(elbowEncoderPort, kBrushless),
    m_absoluteShoulderEncoder(shoulderEncoderPort),
    m_absoluteElbowEncoder(elbowEncoderPort),
    m_relativeShoulderEncoder(m_shoulderMotorLeft.GetEncoder()),
    m_relativeElbowEncoder(m_elbowMotor.GetEncoder()),
    m_shoulderController(m_shoulderMotorLeft.GetPIDController()),
    m_elbowController(m_elbowMotor.GetPIDController())
{
    m_shoulderController.SetP(shoulderConstants::kP);
    m_shoulderController.SetI(shoulderConstants::kI);
    m_shoulderController.SetD(shoulderConstants::kD);
    m_shoulderController.SetFF(shoulderConstants::kFF);
    
    m_shoulderMotorLeft.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_shoulderMotorLeft.EnableVoltageCompensation(12.0);
    m_shoulderMotorLeft.SetSmartCurrentLimit(40);

    m_shoulderMotorRight.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_shoulderMotorRight.EnableVoltageCompensation(12.0);
    m_shoulderMotorRight.SetSmartCurrentLimit(40);

    m_elbowController.SetP(elbowConstants::kP);
    m_elbowController.SetI(elbowConstants::kI);
    m_elbowController.SetD(elbowConstants::kD);
    m_elbowController.SetFF(elbowConstants::kFF);
    
    m_elbowMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_elbowMotor.EnableVoltageCompensation(12.0);
    m_elbowMotor.SetSmartCurrentLimit(40);
}

armPose Superstructure::getPose(){
    return m_pose;
}

