// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;


public class Position extends Command {
  private final Intake intake;
  private final IntakePosition position;

  public Position(Intake intake, IntakePosition position) 
  {
    this.intake = intake;
    this.position = position;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    intake.setPositionMotorState(position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
    // return Constants.CALCULATIONS.epsilonEquals(intake.getCurrentPosition(), position.getAngle(), IntakeConstants.PositionMotor.POSITION_TOLERANCE);
  }
}
