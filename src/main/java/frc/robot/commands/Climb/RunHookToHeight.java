// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunHookToHeight extends Command {
  Climb climb;
  double dutyCycle;
  Heights desiredHeight;

  /** Creates a new RaiseRobot. */
  public RunHookToHeight(Climb climb, Heights desiredHeight, double dutyCycle) {
    this.climb = climb;
    this.desiredHeight = desiredHeight;
    this.dutyCycle = dutyCycle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dutyCycle = Math.signum(desiredHeight.getHeight() - climb.getPosition()) * Math.abs(dutyCycle);
    climb.setMotorPower(dutyCycle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopClimbMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climb.isAtPosition(desiredHeight.getHeight());
  }
}
