// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberAux;
import frc.robot.subsystems.ClimberUp;

public class ClimberAuxRun extends CommandBase {
  private final ClimberAux m_climberAux;
  private DoubleSupplier m_setpoint;
  /** Creates a new ClimberAuxRun. */
  public ClimberAuxRun(DoubleSupplier setpoint, ClimberAux subsystem) {
    m_setpoint = setpoint;
    m_climberAux = subsystem;

    addRequirements(m_climberAux);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentcommand = m_setpoint.getAsDouble();
    if (Math.abs(currentcommand) < .25){
        currentcommand = 0;
    }
    m_climberAux.my_Climb_Rotate(currentcommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberAux.my_Climb_Rotate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
