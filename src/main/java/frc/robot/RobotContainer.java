// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;


import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.*;
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

        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose);

        // configureDriveBindings();
        configureTestBindings();
    }

    // for tests!!
    public void configureTestBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() ->driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));
        driveController.options().onTrue(driveBase.resetOnlyDirection());

        driveController.povRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACKWARDS));
        driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_LEFT));
        driveController.povUpRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_RIGHT));
        driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_LEFT));
        driveController.povDownRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_RIGHT));

        driveController.cross().onTrue(
            Commands.either(
                intake.moveToPositionCommand(IntakePosition.Closed),
                intake.moveToPositionCommand(IntakePosition.Open),
                () -> intake.getCurrentState() == IntakePosition.Open
            )
        );
        driveController.circle().toggleOnTrue(
            Commands.parallel(
                intake.spinRollersCommand(),
                funnel.funnelingCommand()
            )
        );
        driveController.square().toggleOnTrue(
            Commands.startEnd(
                () -> funnel.SpinCenterMotors(-0.3),
                () -> funnel.StopCenterMotors(),
                funnel)
        );

        driveController.R3().onTrue(new ProxyCommand(intake.bumpFuelCommand()));
        driveController.triangle().toggleOnTrue(
            Commands.parallel(
                new ShootVelocity(shooter, () -> RPM.of(1500)),
                Commands.waitUntil(() -> shooter.isAtRequiredVelocity(RPM.of(1500)))
                    .andThen(
                        Commands.parallel(
                            kicker.setKickerByDistance(RobotContainer::distanceFromHub),
                            funnel.toShooterCommand()
                        )
                )
            )
        );

    }

    //#region Drive
    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() ->driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));

        Command fullShootCommand =
            Commands.parallel(
                new ShootDistance(shooter, RobotContainer::distanceFromHub),
                Commands.waitUntil(() -> shooter.isAtRequiredVelocity(RobotContainer.distanceFromHub()))
                    .andThen(
                        Commands.parallel(
                            kicker.setKickerByDistance(RobotContainer::distanceFromHub),
                            funnel.toShooterCommand()
                        )
                    )
            );

        Command passCommand =
            Commands.parallel(
              new ShootVelocity(shooter, () -> RPM.of(ShooterConstants.PASSING_VELOCITY)),
                Commands.waitUntil(() -> shooter.isAtRequiredVelocity(RobotContainer.distanceFromHub()))
                    .andThen(
                        Commands.parallel(
                            kicker.setKickerCommand(1),
                            funnel.toShooterCommand()
                        )
                    )
            );

        Command fullClimb =
            Commands.sequence(
                climb.toPositionCommand(Heights.EXTENDED),
                new HookToTower(driveBase),
                new RunHookToHeight(climb, Heights.RESET, 1)
            );

        driveController.PS().onTrue(driveBase.resetOnlyDirection());

        driveController.options().whileTrue(climb.dutyCycleCommand(0.1));
        driveController.create().whileTrue(climb.dutyCycleCommand(-0.1));

        driveController.povRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACKWARDS));
        driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_LEFT));
        driveController.povUpRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_RIGHT));
        driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_LEFT));
        driveController.povDownRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_RIGHT));

        driveController.R1().toggleOnTrue(
            new AimDriving(driveBase, driveController, () -> Constants.FieldConstants.HUB_POSITION)
            .onlyIf(RobotContainer::inAllianceZone)
        );

        driveController.L1().toggleOnTrue(
            new ConditionalCommand(
                fullShootCommand,
                passCommand,
                RobotContainer::inAllianceZone
            )
        );

        driveController.cross().onTrue(
            Commands.either(
                intake.moveToPositionCommand(IntakePosition.Closed),
                intake.moveToPositionCommand(IntakePosition.Open),
                () -> intake.getCurrentState() == IntakePosition.Open
            )
        );
        driveController.circle().toggleOnTrue(intake.spinRollersCommand());

        driveController.triangle().whileTrue(fullClimb);
        driveController.square().toggleOnTrue(funnel.funnelingCommand());

        driveController.R3().onTrue(intake.bumpFuelCommand());
    }
    //#endregion

    public static Distance distanceFromHub(){
        Pose2d hubLocation = Constants.FieldConstants.HUB_POSITION;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            hubLocation = FlippingUtil.flipFieldPose(hubLocation);
        }
        return Meters.of(
            driveBase.getPose().getTranslation().getDistance(hubLocation.getTranslation())
        );
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
