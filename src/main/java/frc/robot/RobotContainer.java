// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import frc.robot.commands.Climb.RunHookToHeight;
import frc.robot.commands.Drive.AimDriving;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.DriveToPose;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.commands.Intake.collectingCommand;
import frc.robot.commands.Intake.pivotCommand;
import frc.robot.commands.Shooter.ShootDistance;
import frc.robot.commands.Shooter.ShootVelocity;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Kicker.KickerConstants;


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

    public static LoggedDashboardChooser<Command> autoChooser;
    public static LoggedDashboardChooser<String> sideChooser;
    public static HashMap<String, Command> mirroredAutoMap = new HashMap<>();


    public RobotContainer() {
        driveController = new CommandPS5Controller(0);

        driveBase = new DriveBase();
        climb = new Climb();
        funnel = new Funnel();
        intake = new Intake();
        shooter = new Shooter();
        kicker = new Kicker();

        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose);

        configureNamedCommands();
        configureDriveBindings();
        // configureTestBindings();
        configureChoosers();
        configureMirroredAutosMap();
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

        driveController.triangle().onTrue(climb.toPositionCommand(Heights.EXTENDED));
        driveController.cross().onTrue(new RunHookToHeight(climb, Heights.RESET, 0.7));

        driveController.options().whileTrue(climb.dutyCycleCommand(0.1));
        driveController.create().whileTrue(climb.dutyCycleCommand(-0.1));

        driveController.R1().onTrue(new InstantCommand(() -> climb.resetPosition()));
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

        Command hookToTower = 
            Commands.sequence(
                new DriveToPose(driveBase, Constants.FieldConstants.PRE_TOWER_POSITION).withTimeout(2),
                new DriveToPose(driveBase, Constants.FieldConstants.TOWER_POSITION).withTimeout(2)
            );

        Command fullClimb =
            Commands.sequence(
                climb.toPositionCommand(Heights.EXTENDED),
                hookToTower,
                new RunHookToHeight(climb, Heights.RESET, 1)
            );

        Command ejectCommand = Commands.parallel(
            Commands.startEnd(
                () -> kicker.setKickerCommand(KickerConstants.KICKER_EJECT_SPEED), 
                () -> kicker.stopMotors(),
                kicker
            ),
            Commands.startEnd(
                () -> funnel.SpinCenterMotors(FunnelConstants.CenteringMotor.CENTERING_EJECT_SPEED),
                () -> funnel.stopAllMotors(),
                funnel
            )
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
            new AimDriving(driveBase, driveController, RobotContainer::getShootingAngle)
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
        
        driveController.circle().toggleOnTrue(
            Commands.parallel(
                intake.spinRollersCommand(),
                funnel.funnelingCommand()
            )
        );

        driveController.triangle().whileTrue(fullClimb);
        
        driveController.square().toggleOnTrue(ejectCommand);

        driveController.R3().onTrue(intake.bumpFuelCommand());
    }
    //#endregion

    public static void configureNamedCommands(){
        NamedCommands.registerCommand("close intake", new pivotCommand(intake, IntakePosition.Closed));
        NamedCommands.registerCommand("open intake", new pivotCommand(intake, IntakePosition.Open));
        
        NamedCommands.registerCommand("collect fuel", new collectingCommand(intake));

        Command hookToTower = 
            Commands.sequence(
                new DriveToPose(driveBase, Constants.FieldConstants.PRE_TOWER_POSITION).withTimeout(2),
                new DriveToPose(driveBase, Constants.FieldConstants.TOWER_POSITION).withTimeout(2)
            );
        Command fullClimb =
            Commands.sequence(
                climb.toPositionCommand(Heights.EXTENDED),
                hookToTower,
                new RunHookToHeight(climb, Heights.RESET, 1)
            );
        NamedCommands.registerCommand("climb auto", fullClimb);

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
        NamedCommands.registerCommand("shoot to hub", fullShootCommand);
        NamedCommands.registerCommand("shoot to pass", passCommand);
    }

    //#regionchoosers
    public static void configureChoosers(){
        List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
        List<PathPlannerAuto> autosOfAutos = new ArrayList<>();

        autoChooser = new LoggedDashboardChooser<>("chooser");
        for (String autoName : namesOfAutos) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            autosOfAutos.add(auto);
        }

        autosOfAutos.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

        //configure the chooser for the side relative to the driver station.
        //the default is right because the routes are planned for right.
        sideChooser = new LoggedDashboardChooser<>("sideChooser");

        sideChooser.addDefaultOption("right", "right");
        sideChooser.addOption("left", "left");

        SmartDashboard.putData("sideChooser",sideChooser.getSendableChooser());   
    }

    public static void configureMirroredAutosMap(){
        List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
        List<PathPlannerAuto> autosOfAutos = new ArrayList<>();

        for (String autoName : namesOfAutos) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName,true);
            autosOfAutos.add(auto);
        }
        
        for (int i = 0; i < namesOfAutos.size(); i++){
            mirroredAutoMap.put(namesOfAutos.get(i), autosOfAutos.get(i));
        }
    }

    public static Command getAuto(){
        return getSide().equals("right") ? autoChooser.get() : mirroredAutoMap.get(autoChooser.get().getName());
    }

    public static String getSide(){
        return sideChooser.get();
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

    public static double angleToHub(){
        Pose2d hubLocation = Constants.FieldConstants.HUB_POSITION;
        if(Robot.isRedAlliance) hubLocation = FlippingUtil.flipFieldPose(hubLocation);
        return driveBase.getAngleToPose(hubLocation);
    }

    public static double angleToAllianceZone(){
        return Robot.isRedAlliance ? 0 : Math.PI / 2;
    }

    public static double getShootingAngle(){
        return inAllianceZone() ? angleToHub() : angleToAllianceZone();
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
