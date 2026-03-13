// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;


public class collectingCommand extends Command {
  private final Intake intake;
  private final double intakeDutyCycle;

  public collectingCommand(Intake intake, double intakeDutyCycle)
  {
    this.intake = intake;
    this.intakeDutyCycle = intakeDutyCycle;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    intake.setRollersMotorDutyCycle(intakeDutyCycle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
