// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
    private final DriveBase driveBase;
    private Pose2d targetPose;

    private final PIDController XController = DriveBaseConstants.DrivePID.X_PID;
    private final PIDController YController = DriveBaseConstants.DrivePID.Y_PID;
    private final PIDController ThetaController = DriveBaseConstants.DrivePID.THETA_PID;

    private final List<PIDController> controllers = List.of(XController, YController, ThetaController);

  /** Creates a new HookToTower. */
  public DriveToPose(DriveBase driveBase, Pose2d targetPose) {
    this.driveBase = driveBase;
    this.targetPose = 
        Robot.isRedAlliance ?
        targetPose :
        FlippingUtil.flipFieldPose(targetPose);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        controllers.forEach(t -> t.reset());

        XController.setSetpoint(targetPose.getX());
        YController.setSetpoint(targetPose.getY());
        ThetaController.setSetpoint(targetPose.getRotation().getRadians());

        Pose2d currentPose = driveBase.getPose();

        XController.calculate(currentPose.getX(), targetPose.getX());
        YController.calculate(currentPose.getY(), targetPose.getY());
        ThetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        Logger.recordOutput("Drive To Pose/Target Pose", targetPose);

        Logger.recordOutput("Drive To Pose/target Theta", ThetaController.getSetpoint());
        Logger.recordOutput("Drive To Pose/target X", XController.getSetpoint());
        Logger.recordOutput("Drive To Pose/target Y", YController.getSetpoint());

        driveBase.drive(new ChassisSpeeds());
  }

  public boolean isOutOfTolerance(){
    Pose2d currentPose = driveBase.getPose();

    return Math.abs(targetPose.getX() - currentPose.getX()) > XController.getErrorTolerance() ||
        Math.abs(targetPose.getY() - currentPose.getY()) > YController.getErrorTolerance() ||
        Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians())
        > ThetaController.getErrorTolerance();
  }

    private double getClampValue(double error, double upperLimit, double lowerLimit){
      double value = Math.abs(error) * upperLimit;

      return Math.max(value, lowerLimit);
  }

  public ChassisSpeeds calculatePID(){
    double XOutput = XController.calculate(driveBase.getPose().getX());
    double YOutput = YController.calculate(driveBase.getPose().getY());
    double ThetaOutput = ThetaController.calculate(driveBase.getRotation2d().getRadians());

    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(XOutput, YOutput, ThetaOutput);
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, driveBase.getPose().getRotation());

    double upperLimitXY = DriveBaseConstants.DrivePID.UPPER_SPEED_LIMIT_XY;
    double lowerLimitXY = DriveBaseConstants.DrivePID.LOWER_SPEED_LIMIT_XY;

    double upperLimitTheta = DriveBaseConstants.DrivePID.UPPER_SPEED_LIMIT_THETA;
    double lowerLimitTheta = DriveBaseConstants.DrivePID.LOWER_SPEED_LIMIT_THETA;

    double maxXOutput = getClampValue(XController.getError(), upperLimitXY, lowerLimitXY);
    double maxYOutput = getClampValue(YController.getError(), upperLimitXY, lowerLimitXY);

    double maxThetaOutput = getClampValue(ThetaController.getError(), upperLimitTheta, lowerLimitTheta);

    robotRelativeSpeeds.vxMetersPerSecond = MathUtil.clamp(robotRelativeSpeeds.vxMetersPerSecond, -maxXOutput, maxXOutput);
    robotRelativeSpeeds.vyMetersPerSecond = MathUtil.clamp(robotRelativeSpeeds.vyMetersPerSecond, -maxYOutput, maxYOutput);
    robotRelativeSpeeds.omegaRadiansPerSecond = MathUtil.clamp(robotRelativeSpeeds.omegaRadiansPerSecond, -maxThetaOutput, maxThetaOutput);

    Logger.recordOutput("Drive To Pose/X Error", XController.getError());
    Logger.recordOutput("Drive To Pose/Y Error", YController.getError());

    return robotRelativeSpeeds;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.drive(calculatePID());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !isOutOfTolerance();
  }
}
