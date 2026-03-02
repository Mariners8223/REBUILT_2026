// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;

import java.util.function.BiFunction;

import org.ejml.equation.Sequence;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Climb.RunHookToHeight;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.TurnToSetAngle;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.commands.Kicker.BlinkingKicker;
import frc.robot.commands.Shooter.ShootVelocity;
import frc.robot.commands.SuperShoot.SuperShootCommand;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;
import frc.robot.subsystems.DriveTrain.DriveBaseSYSID;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterSysID;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;

public class RobotContainer {
    public static CommandPS5Controller driveController;
    public static Constants.autoConstats.TrenchLocations trenchLocations = new Constants.autoConstats.TrenchLocations();


    public static DriveBase driveBase;
    public static Funnel funnel;
    public static Intake intake;
    public static Shooter shooter;
    public static Kicker kicker;
    public static Climb climb;

    public static DriveBaseSYSID driveSysID;
    public static ShooterSysID shooterSysID;
    public static CommandXboxController driveXboxController;
    public static Vision vision;


    public RobotContainer() {
        driveController = new CommandPS5Controller(0);

        driveBase = new DriveBase();
        climb = new Climb();
        funnel = new Funnel();
        intake = new Intake();
        shooter = new Shooter();
        kicker = new Kicker();

        driveSysID = new DriveBaseSYSID(driveBase, driveController);
        shooterSysID = new ShooterSysID(shooter);
        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose);

        configureDriveBindings();

        driveController.square().toggleOnTrue(new SuperShootCommand(driveBase, shooter, kicker, funnel, Constants.autoConstats.hub));
    }

    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() ->driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));

        driveController.povUp().onTrue(driveBase.resetOnlyDirection());
    }
}