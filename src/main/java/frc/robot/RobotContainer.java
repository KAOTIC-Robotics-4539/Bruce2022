// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

    // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems
    public final ClimberUp m_climberUp = new ClimberUp();
    public final intakeArms m_intakeArms = new intakeArms();
    public final intakeRollers m_intakeRollers = new intakeRollers();
    public final ballManagement m_ballManagement = new ballManagement();
    public final trigger m_trigger = new trigger();
    public final shooter m_shooter = new shooter();
    public final DriveTrain m_driveTrain = new DriveTrain();
    public final ClimberAux m_climberAux = new ClimberAux();
// Joysticks
private final XboxController coDriverControler = new XboxController(1);
private final XboxController driverContorler = new XboxController(0);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
        // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems
    SmartDashboard.putData(m_intakeArms);
    SmartDashboard.putData(m_intakeRollers);
    SmartDashboard.putData(m_ballManagement);
    SmartDashboard.putData(m_trigger);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_driveTrain);


    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("intakeRun: intakeRunForward", new intakeRun(.75, m_intakeRollers));
    SmartDashboard.putData("intakeRun: intakeRunReverse", new intakeRun(-.75, m_intakeRollers));
    SmartDashboard.putData("Auto_2_Ball", new Auto_2_Ball( m_driveTrain, m_intakeArms, m_driveTrain, m_intakeRollers, m_ballManagement, m_shooter, m_trigger ));
    SmartDashboard.putData("DriveDistance: distance", new DriveDistance(3, 1.0, m_driveTrain));
    SmartDashboard.putData("DriveResetEncoder", new DriveResetEncoder( m_driveTrain ));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    SmartDashboard.putData("Auto_Shoot_N_Drive", new Auto_Shoot_N_Drive( m_shooter, m_trigger, m_ballManagement, m_driveTrain ));
   
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
        // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    //m_driveTrain.setDefaultCommand(new DriverWithJoysticks(0, 0, m_driveTrain) );


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_driveTrain.setDefaultCommand(new DriverWithJoysticks(() -> -driverContorler.getRawAxis(1), () -> -driverContorler.getRawAxis(4), m_driveTrain) );
   m_climberUp.setDefaultCommand(new ClimberRun(() ->  coDriverControler.getRawAxis(1), m_climberUp));   
   m_climberAux.setDefaultCommand(new ClimberAuxRun(() ->  -coDriverControler.getRawAxis(5), m_climberAux));   
   
   // Configure autonomous sendable chooser
    m_chooser.addOption("Auto_Shoot_N_Drive", new Auto_Shoot_N_Drive( m_shooter, m_trigger, m_ballManagement, m_driveTrain ));
        // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.setDefaultOption("Auto_2_Ball", new Auto_2_Ball( m_driveTrain, m_intakeArms, m_intakeRollers, m_ballManagement, m_ballManagement, m_shooter, m_trigger ));
    m_chooser.addOption("DriveDistance", new DriveDistance( 18, .5, m_driveTrain ));
    //m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMreturn Robot.m_drivetrain.getDriveEncoderDistance() == distance;OUS

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
// Create some buttons
final JoystickButton intakeIn_X = new JoystickButton(coDriverControler, XboxController.Button.kX.value);        
intakeIn_X.whenPressed(new intakeArmsOut(false, m_intakeArms) ,true);
    //SmartDashboard.putData("intakeIn_X",new intakeArmsOut(false, m_intakeArms) );

final JoystickButton intakeOut_B = new JoystickButton(coDriverControler, XboxController.Button.kB.value);        
intakeOut_B.whenPressed(new intakeArmsOut(true, m_intakeArms) ,true);
    //SmartDashboard.putData("IntakeOut_B",new intakeArmsOut(true, m_intakeArms) );

final JoystickButton intakeRollerRunOut_A = new JoystickButton(coDriverControler, XboxController.Button.kA.value);        
intakeRollerRunOut_A.whileHeld(new intakeRun(-.75, m_intakeRollers) ,true);
    //SmartDashboard.putData("intakeRollerRunOut_A",new intakeRun(-.75, m_intakeRollers) );

final JoystickButton intakeRollerRunIn_RB = new JoystickButton(coDriverControler, XboxController.Button.kRightBumper.value);        
intakeRollerRunIn_RB.whileHeld(new intakeRun(0.75, m_intakeRollers) ,true);
    //SmartDashboard.putData("intakeRollerRunIn_RB",new intakeRun(0.75, m_intakeRollers) );

final JoystickButton ballMangeReverse_Y = new JoystickButton(coDriverControler, XboxController.Button.kY.value);        
ballMangeReverse_Y.whileHeld(new ballmanageRun(-.75, m_ballManagement) ,true);
    //SmartDashboard.putData("ballMangeReverse_Y",new ballmanageRun(-.75, m_ballManagement) );

final JoystickButton ballMangement_RunLB = new JoystickButton(coDriverControler, XboxController.Button.kLeftBumper.value);        
ballMangement_RunLB.whileHeld(new ballmanageRun(.75, m_ballManagement) ,true);
    //SmartDashboard.putData("ballMangement_RunLB",new ballmanageRun(.75, m_ballManagement) );

final JoystickButton shiftLow_Back = new JoystickButton(driverContorler, XboxController.Button.kBack.value);        
shiftLow_Back.whenPressed(new driveShiftHigh(false, m_driveTrain) ,true);
    SmartDashboard.putData("ShiftLow_Back",new driveShiftHigh(false, m_driveTrain) );

final JoystickButton shiftHigh_start = new JoystickButton(driverContorler, XboxController.Button.kStart.value);        
shiftHigh_start.whenPressed(new driveShiftHigh(true, m_driveTrain) ,true);
    SmartDashboard.putData("ShiftHigh_start",new driveShiftHigh(true, m_driveTrain) );

final JoystickButton triggerRun_LB = new JoystickButton(driverContorler, XboxController.Button.kLeftBumper.value);        
triggerRun_LB.whileHeld(new triggerRun(.75, m_trigger) ,true);
    //SmartDashboard.putData("TriggerRun_LB",new triggerRun(.75, m_trigger) );

final JoystickButton shooterRun_RB = new JoystickButton(driverContorler, XboxController.Button.kRightBumper.value);        
shooterRun_RB.whileHeld(new shooterRun(.75, m_shooter) ,true);
    //SmartDashboard.putData("ShooterRun_RB",new shooterRun(.75, m_shooter) );



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
  }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public XboxController getDriverContorler() {
      return driverContorler;
    }

public XboxController getCoDriverControler() {
      return coDriverControler;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
  

}

