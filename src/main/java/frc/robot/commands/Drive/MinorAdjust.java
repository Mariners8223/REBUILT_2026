// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.DriveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MinorAdjust extends Command {
  public static enum AdjustmentDirection{
    LEFT(0, 0.3),
    RIGHT(0, -0.3),
    FORWARD(0.3, 0),
    BACKWARDS(-0.3, 0),
    FRONT_RIGHT(0.3, -0.3),
    FRONT_LEFT(0.3, 0.3),
    BACK_RIGHT(-0.3, -0.3),
    BACK_LEFT(-0.3, 0.3);

    private final double vX;
    private final double vY;

    AdjustmentDirection(double vX, double vY){
      this.vX = vX;
      this.vY = vY;
    }

    public double getVX(){
      return vX;
    }

    public double getVY(){
      return vY;
    }
  }

  private final DriveBase driveBase;
  private final AdjustmentDirection direction;

  /** Creates a new MinorAdjust. */
  public MinorAdjust(DriveBase driveBase, AdjustmentDirection direcation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;
    this.direction = direcation;

    addRequirements(driveBase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d gyroAngle = driveBase.getRotation2d();
    if(Robot.isRedAlliance) gyroAngle = gyroAngle.plus(Rotation2d.fromDegrees(180));

    ChassisSpeeds fieldRelative = new ChassisSpeeds(direction.getVX(), direction.getVY(), 0);
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, gyroAngle);

    driveBase.drive(robotRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
