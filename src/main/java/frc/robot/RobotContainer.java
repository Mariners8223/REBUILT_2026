// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.BiFunction;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Climb.RunHookToHeight;
import frc.robot.commands.Drive.AimDriving;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.HookToTower;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.commands.Shooter.ShootDistance;
import frc.robot.commands.Shooter.ShootVelocity;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;
import frc.robot.subsystems.DriveTrain.DriveBaseSYSID;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSysID;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
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
        driveSysID = new DriveBaseSYSID(driveBase, driveController);
        climb = new Climb();
        funnel = new Funnel();
        intake = new Intake();
        shooter = new Shooter();
        kicker = new Kicker();

        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose);

        //configureDriveBindings();
        configureDriveBindings();
        configureDriveSysidBindings();
    }

    public static Distance distanceFromHub(){
        Pose2d hubLocation = Constants.FieldConstants.HUB_POSITION;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            hubLocation = FlippingUtil.flipFieldPose(hubLocation);
        }
        return Meters.of(
            driveBase.getPose().getTranslation().getDistance(hubLocation.getTranslation())
        );
    }

     public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() ->driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));

        driveController.povUp().onTrue(driveBase.resetOnlyDirection());

        // driveController.PS().onTrue(Commands.run(() -> driveBase.Rota))
    }

    public void configureDriveSysidBindings(){
        SendableChooser<String> type = new SendableChooser<String>();
        type.addOption("Drive", "Drive");
        type.addOption("Steer", "Steer");
        type.addOption("Theta", "Theta");
        type.setDefaultOption("Drive", "Drive");

        SmartDashboard.putData(type);

        String chosen = "Theta";

        driveController.cross().whileTrue(
            new ProxyCommand(chooseCommand(chosen).apply(true, Direction.kReverse))
        );
        driveController.triangle().whileTrue(
            new ProxyCommand(chooseCommand(chosen).apply(true, Direction.kForward))
        );
        driveController.square().whileTrue(
            new ProxyCommand(chooseCommand(chosen).apply(false, Direction.kReverse))
        );
        driveController.circle().whileTrue(
            new ProxyCommand(chooseCommand(chosen).apply(false, Direction.kForward))
        );
    }


    public BiFunction<Boolean, Direction, Command> chooseCommand(String type){
        switch (type) {
            case "Drive":
                return (isDynamic, direction) ->
                    isDynamic ? driveSysID.getDriveMotorsRoutineDynamic(direction) : driveSysID.getDriveMotorsRoutineQuasistatic(direction);
            case "Steer":
                return (isDynamic, direction) ->
                    isDynamic ? driveSysID.getSteerMotorsRoutineDynamic(direction) : driveSysID.getSteerMotorsRoutineQuasistatic(direction);
            case "Theta":
                return (isDynamic, direction) ->
                    isDynamic ? driveSysID.getThetaRoutineDynamic(direction) : driveSysID.getThetaRoutineQuasistatic(direction);
            default:
                return (isDynamic, direction) -> new InstantCommand();
        }
    }

   //#region auto
    public Command passTrench(){
    Pose2d pose = driveBase.getPose();

    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        pose = FlippingUtil.flipFieldPose(driveBase.getPose());
    }
    if (pose.getY() >= 4){
        if (pose.getX() <= 4.5){
            return driveBase.findPath(trenchLocations.upRightTrench,2);
        }
        return driveBase.findPath(trenchLocations.upLeftTrench, 2);
    }
    if (pose.getX() <= 4.5){
        return driveBase.findPath(trenchLocations.downRightTrench, 2);
    }
    return driveBase.findPath(trenchLocations.downLeftTrench, 2);
   }


   public static boolean inRange(){
    Pose2d pose = driveBase.getPose();
    Pose2d hub = Constants.autoConstats.hub;
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        pose = FlippingUtil.flipFieldPose(driveBase.getPose());
        hub = FlippingUtil.flipFieldPose(hub);
    }
    double distanceFromHub = driveBase.getDistanceFromPoint2D(hub);

    if (pose.getX() > 4.5) return false;
    return distanceFromHub < Constants.autoConstats.maxRange;
   }


    public Command getInRange(){
        Pose2d pose = driveBase.getPose();
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            pose = FlippingUtil.flipFieldPose(driveBase.getPose());
        }
        if (pose.getY() >= 4.5){
            return driveBase.findPath(Constants.autoConstats.topShoot).until(RobotContainer::inRange);
        }
        return driveBase.findPath(Constants.autoConstats.bottomShoot).until(RobotContainer::inRange);
    }

   public static boolean inAllianceZone(){
        if (Robot.isRedAlliance) return driveBase.getPose().getX() > Constants.FieldConstants.RED_ALLIANCE_ZONE_X;
        else return driveBase.getPose().getX() < Constants.FieldConstants.BLUE_ALLIANCE_ZONE_X;
    }
   //#endregion
}
