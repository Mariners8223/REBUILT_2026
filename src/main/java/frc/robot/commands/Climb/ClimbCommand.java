// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;

public class ClimbCommand extends Command {
    private final Climb climb;
    private final Heights desiredHeight;

    public ClimbCommand(Climb climb, Heights height) {
        this.climb = climb;
        this.desiredHeight = height;

        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // climb.setMotorPower(Math.signum(desiredHeight.getHeight() - climb.getPosition()) * ClimbConstants.CLIMB_POWER);
        climb.setMotorHeight(desiredHeight);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.setMotorPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return climb.isAtPosition(desiredHeight.getHeight());
        return false;
    }
}
