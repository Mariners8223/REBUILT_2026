// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Liran.kicker;
import frc.robot.subsystems.Liran.kickerconstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class liran extends Command {

    public static kicker a;
  public liran() {
     a=new kicker();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    a.wheelturn(kickerconstants.LEADMOTOR.SPEED, kickerconstants.FUNNELMOTOR.SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    a.stopmotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    
  }
}
