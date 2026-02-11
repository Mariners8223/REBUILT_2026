// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants;

public class ClimbCommand extends Command {
    private final Climb climb;
    private final ClimbConstants.HEIGHTS desired_height;

    public ClimbCommand(Climb climb,ClimbConstants.HEIGHTS height) {
        this.climb = climb;
        this.desired_height = height;
        
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climb.setMotorPower(Math.signum(desired_height.GetHeight() - climb.getPosition()) * ClimbConstants.CLIMB_POWER);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.setMotorPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(climb.getPosition() - desired_height.GetHeight()) <= ClimbConstants.CLIMB_TOLERANCE;
    }
}
