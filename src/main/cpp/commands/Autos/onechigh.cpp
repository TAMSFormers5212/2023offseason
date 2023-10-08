#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>

#include "commands/Autos/onechigh.h"
#include "commands/SetArmPose.h"
#include "Constants.h"

using namespace ArmConstants::poseConstants;

onechigh::onechigh(Swerve* drive, Superstructure* arm) {    
    AddCommands(
        frc2::ParallelRaceGroup{
            SetArmPose(arm, armPose(elbowPositions[highcube], shoulderPositions[highcube])),
            frc2::WaitCommand(4_s)
        },
        frc2::ParallelRaceGroup{
            frc2::RunCommand(
                [this, arm] {
                    arm->setGrabber(1.0);  
                },
                {}
            ),
            frc2::WaitCommand(2_s)
        }
    //     frc2::ParallelCommandGroup{
    //         frc2::RunCommand([arm]() {arm->goToPose(arm->getPose("stow"));}),
    //         frc2::RunCommand([drive]() {drive->swerveDrive(0, -0.3, 0, true);}),
    //         frc2::WaitCommand(1.0_s)
    //     }
    );
    
}