// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
    import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonFX leftMaster;
private WPI_TalonFX leftSlave;
private MotorControllerGroup leftMotorGroup;
private WPI_TalonFX rightMaster;
private WPI_TalonFX rightSlave;
private MotorControllerGroup rightMotorGroup;
private DifferentialDrive differentialDrive1;
private AnalogGyro gyro;
private Encoder driveEncoder;
private DoubleSolenoid shifter;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
//Hello 2
    /**
    *
    */
    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftMaster = new WPI_TalonFX(1);
 
    /* Factory default hardware to prevent unexpected behavior */
leftMaster.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
leftMaster.setInverted(true);
leftMaster.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
leftMaster.configNominalOutputForward(0, 30);
leftMaster.configNominalOutputReverse(0, 30);
leftMaster.configPeakOutputForward(1, 30);
leftMaster.configPeakOutputReverse(-1, 30);
        


leftSlave = new WPI_TalonFX(2);
 
    /* Factory default hardware to prevent unexpected behavior */
leftSlave.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
leftSlave.setInverted(true);
leftSlave.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
leftSlave.configNominalOutputForward(0, 30);
leftSlave.configNominalOutputReverse(0, 30);
leftSlave.configPeakOutputForward(1, 30);
leftSlave.configPeakOutputReverse(-1, 30);
        


leftMotorGroup = new MotorControllerGroup(leftMaster, leftSlave  );
 addChild("LeftMotorGroup",leftMotorGroup);
 

rightMaster = new WPI_TalonFX(3);
 
    /* Factory default hardware to prevent unexpected behavior */
rightMaster.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
rightMaster.setInverted(false);
rightMaster.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
rightMaster.configNominalOutputForward(0, 30);
rightMaster.configNominalOutputReverse(0, 30);
rightMaster.configPeakOutputForward(1, 30);
rightMaster.configPeakOutputReverse(-1, 30);
        


rightSlave = new WPI_TalonFX(4);
 
    /* Factory default hardware to prevent unexpected behavior */
rightSlave.configFactoryDefault();

        /* Invert Motor? and set Break Mode */
rightSlave.setInverted(false);
rightSlave.setNeutralMode(NeutralMode.Coast);

        /* Set the peak and nominal outputs */
rightSlave.configNominalOutputForward(0, 30);
rightSlave.configNominalOutputReverse(0, 30);
rightSlave.configPeakOutputForward(1, 30);
rightSlave.configPeakOutputReverse(-1, 30);
        


rightMotorGroup = new MotorControllerGroup(rightMaster, rightSlave  );
 addChild("RightMotorGroup",rightMotorGroup);
 

differentialDrive1 = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
 addChild("Differential Drive 1",differentialDrive1);
 differentialDrive1.setSafetyEnabled(true);
differentialDrive1.setExpiration(0.1);
differentialDrive1.setMaxOutput(1.0);


gyro = new AnalogGyro(0);
 addChild("gyro",gyro);
 gyro.setSensitivity(0.007);

driveEncoder = new Encoder(0, 1, false);
 addChild("DriveEncoder",driveEncoder);
 driveEncoder.setDistancePerPulse(1.0);
driveEncoder.setIndexSource(2, IndexingType.kResetOnRisingEdge);

shifter = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, 0, 1);
 addChild("shifter", shifter);
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void my_DriveArcade(double xSpeed, double zRotation){
        differentialDrive1.arcadeDrive(xSpeed, zRotation);

    }
    public void my_resetGyro(){
        gyro.reset();
    }

    public double my_getGyro(){
        return gyro.getAngle();
    }
public void my_shiftHigh(){
    shifter.set(Value.kForward);
}
public void my_ShiftLow(){
    shifter.set(Value.kReverse);
}
public double getDriveEncoderCount() {
    return driveEncoder.get();
}

public void resetDriveEncoder() {
    driveEncoder.reset();
}
    //Encoder Data

    //supper code

}

