#pragma once

#include <subsystems/Superstructure.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class SetArmPose : public frc2::CommandHelper<frc2::CommandBase, SetArmPose>{
    public:

    explicit SetArmPose(Superstructure* arm, armPose pose);

    void Initialize() override;

    void End(bool interrupted) override;

    private:
    Superstructure* m_arm;
    armPose m_pose;


};