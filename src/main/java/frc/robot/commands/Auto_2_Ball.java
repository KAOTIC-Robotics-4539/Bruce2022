// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.*;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Auto_2_Ball extends SequentialCommandGroup {

    // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public Auto_2_Ball(DriveTrain driveTrain, intakeArms m_intakeArms, DriveTrain m_driveTrain, intakeRollers m_intakeRollers, ballManagement m_ballManagement, shooter m_shooter, trigger m_trigger){

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    addCommands(
        // Add Commands here:
        // Also add parallel commands using the
        //
        // addCommands(
        //      new command1(argsN, subsystem),
        //      parallel(
        //          new command2(argsN, subsystem),
        //          new command3(argsN, subsystem)
        //      )    
        //  );

        new intakeArmsOut(true, m_intakeArms),
            new ParallelDeadlineGroup(new DriveDistance(50, -.5, m_driveTrain),
                                        new intakeRun(-.75, m_intakeRollers),
                                        new ballmanageRun(.75, m_ballManagement)
            ),
            new DriveDistance(50, .5, m_driveTrain),
            parallel(new shooterRun(.75, m_shooter).withTimeout(6),
                            new SequentialCommandGroup(new WaitCommand(1),
                                            new ParallelDeadlineGroup(new triggerRun(.75, m_trigger).withTimeout(3),
                                                new ballmanageRun(.75, m_ballManagement) 
                                                      )
                                            )
                )
               

        );
        
    }

    public Auto_2_Ball(DriveTrain m_driveTrain, intakeArms m_intakeArms, intakeRollers m_intakeRollers,
            ballManagement m_ballManagement, ballManagement m_ballManagement2, shooter m_shooter, trigger m_trigger) {
    }

    @Override
    public boolean runsWhenDisabled() {
        // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
