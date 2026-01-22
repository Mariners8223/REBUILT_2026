// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;


public class Position extends Command {
  private final Intake intake;

  public Position(Intake intake) 
  {
    this.intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    intake.setPositionMotorVoltage(IntakeConstants.PositionMotor.VOLTAGE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return (Math.abs(intake.getCurrentPosition() - intake.getDesiredPosition()) <= IntakeConstants.PositionMotor.POSITION_TOLERANCE);
  }
}
