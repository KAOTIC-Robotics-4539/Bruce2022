// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberAux extends SubsystemBase {

  private WPI_VictorSPX climberMotorRotate;

  /** Creates a new ClimberAux. */
  public ClimberAux() {
    climberMotorRotate = new WPI_VictorSPX(7);
    climberMotorRotate.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void my_Climb_Rotate(double speed){
    climberMotorRotate.set(speed);
  }
}
