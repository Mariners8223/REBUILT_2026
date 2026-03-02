// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToSetAngle extends Command {
  DriveBase driveBase;
  Supplier<Angle> targetAngleSupplier;

  PIDController thetaController = DriveBaseConstants.PathPlanner.THETA_PID.createPIDController();

  /** Creates a new TurnToSetAngle. */
  public TurnToSetAngle(DriveBase driveBase, Supplier<Angle> targetAngleSupplier) {
    this.driveBase = driveBase;
    this.targetAngleSupplier = targetAngleSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setSetpoint(targetAngleSupplier.get().in(Radians));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaResult = thetaController.calculate(driveBase.getPose().getRotation().getRadians());
    driveBase.drive(new ChassisSpeeds(0, 0, thetaResult));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveBase.getRotation2d().getDegrees() - targetAngleSupplier.get().in(Degrees)) < 5;
  }
}
