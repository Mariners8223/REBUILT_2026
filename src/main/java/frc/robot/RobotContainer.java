// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.Set;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.controls.StaticBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Shoot;
import frc.robot.commands.Drive.AimDriving;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Vision.Vision;
import frc.util.HubTracker;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Pivot.Pivot;
import frc.robot.subsystems.Intake.Pivot.PivotConstants.PivotStates;
import frc.robot.subsystems.Intake.Rollers.Rollers;
import frc.robot.subsystems.Kicker.Kicker;


public class RobotContainer {
    public static Alert controllerDisconnected = new Alert("Drive Controller Disconnect", AlertType.kError);
    public static Alert brownoutBattery = new Alert("BROWNOUT", AlertType.kError);

    public static CommandPS5Controller driveController;
    public static Constants.TrenchLocations trenchLocations = new Constants.TrenchLocations();

    public static DriveBase driveBase;
    public static Funnel funnel;
    public static Pivot pivot;
    public static Rollers rollers;
    public static Shooter shooter;
    public static Kicker kicker;

    public static Feeder feeder;
    public static Vision vision;

    public static LoggedDashboardChooser<Command> autoChooser;
    public static LoggedDashboardChooser<String> sideChooser;
    public static HashMap<String, Command> mirroredAutoMap = new HashMap<>();

    public static PathPlannerPath upLeftTrenchPath;
    public static PathPlannerPath upRightTrenchPath;

    public static Runnable fuelIncrementer;
    public static int fuelApproximation = 0;

    //#region Premade Commands
    public static Supplier<Command> warmupShooter;
    public static Supplier<Command> preShooting;
    public static Supplier<Command> shootCommand;
    public static Supplier<Command> passCommand;
    public static Supplier<Command> conditionalShootCommand;
    public static Supplier<Command> shootWithBump;
    public static Supplier<Command> swapIntakeStateClosed;
    public static Supplier<Command> swapIntakeStateMiddle;
    public static Supplier<Command> resetPositionPivot;
    public static Supplier<Command> shootCommandWithNoVision;
    public static Supplier<Command> shootCommandWithNoVision2;
    public static Supplier<Command> preShootingNV;
    public static Supplier<Command> warmUpShooterNV;
    //#endregion

    public static boolean withAutoEject = false;

    public RobotContainer() {
        driveController = new CommandPS5Controller(0);

        driveBase = new DriveBase();
        funnel = new Funnel();
        pivot = new Pivot();
        rollers = new Rollers();
        shooter = new Shooter();
        kicker = new Kicker();

        feeder = new Feeder(rollers, funnel, kicker);
        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose);

        configureCommands();
        configureNamedCommands();
        configureDriveBindings();
        configureChoosers();
        configureTrenchPaths();
    }

    //#region Commands
    public static void configureCommands(){
        fuelIncrementer = () -> fuelApproximation += 3;

        warmupShooter = () ->
            Commands.either(
                Shoot.ShootDistance(shooter, RobotContainer::distanceFromHub, fuelIncrementer),
                new Shoot(shooter, () -> ShooterConstants.PASSING_VELOCITY, fuelIncrementer),
                RobotContainer::inAllianceZone
            ).withName("Warmup Shooter");

        warmUpShooterNV = () ->
        Shoot.ShootDistance(shooter, RobotContainer::glida, fuelIncrementer)
        .withName("Warmup Shooter NV");
                

        preShooting = () ->
            warmupShooter.get().alongWith(feeder.ejectCommand()).withTimeout(0.5)
            .withName("Pre Shooting");

        preShootingNV = () ->
        warmUpShooterNV.get().alongWith(feeder.ejectCommand()).withTimeout(0.5)
        .withName("pre shoot NV");

        shootCommand = () ->
            Commands.parallel(
                Shoot.ShootDistance(shooter, RobotContainer::distanceFromHub, fuelIncrementer),
                Commands.waitUntil(() -> shooter.isAtRequiredVelocity(RobotContainer.distanceFromHub()))
                        .andThen(feeder.smartFeedingShootCommand(RobotContainer::distanceFromHub, () -> withAutoEject))
            ).withName("Shooting");
        
            
        shootCommandWithNoVision = () ->
        Commands.parallel(
            Shoot.ShootDistance(shooter,RobotContainer::glida, fuelIncrementer),
            Commands.waitUntil(() -> shooter.isAtRequiredVelocity(glida()))
                    .andThen(feeder.smartFeedingShootCommand(RobotContainer::glida, () -> withAutoEject)),
            Commands.waitSeconds(0.5).andThen(pivot.raisePivot(PivotStates.Middle))
        ).withName("ShootingWithoutVision");

        shootCommandWithNoVision2 = () ->
        Commands.parallel(
            Shoot.ShootDistance(shooter,RobotContainer::glida2, fuelIncrementer),
            Commands.waitUntil(() -> shooter.isAtRequiredVelocity(glida2()))
                    .andThen(feeder.smartFeedingShootCommand(RobotContainer::glida2, () -> withAutoEject)),
            Commands.waitSeconds(0.5).andThen(pivot.raisePivot(PivotStates.Middle))
        ).withName("ShootingWithoutVision");


        passCommand = () ->
            Commands.parallel(
                new Shoot(shooter, () -> ShooterConstants.PASSING_VELOCITY, fuelIncrementer),
                Commands.waitUntil(() -> shooter.isAtRequiredVelocity(RobotContainer.distanceFromHub()))
                .andThen(feeder.smartFeedingPassCommand(() -> withAutoEject))
            ).withName("Passing");

        conditionalShootCommand = () ->
            Commands.either(
                shootCommand.get(),
                passCommand.get(),
                RobotContainer::inAllianceZone
            ).withName("Conditional Shooting");

        shootWithBump = () ->
            Commands.parallel(
                conditionalShootCommand.get(),
                Commands.waitSeconds(0.5).andThen(pivot.raisePivot(PivotStates.Middle))
            ).withName("Shoot with Bump");

        swapIntakeStateClosed = () ->
            Commands.either(
                pivot.moveToStateCommand(PivotStates.Closed),
                pivot.moveToStateCommand(PivotStates.Open),
                () -> pivot.getState() == PivotStates.Open
            ).withName("Swap Pivot State to and from Closed");

        swapIntakeStateMiddle = () ->
            Commands.either(
                pivot.moveToStateCommand(PivotStates.Middle),
                pivot.moveToStateCommand(PivotStates.Open),
                () -> pivot.getState() == PivotStates.Open
            ).withName("Swap Pivot State to and from Middle");

        resetPositionPivot = () ->
            Commands.parallel(
                pivot.resetPivot()
            ).withName("reseting");
    }


    public static void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() ->driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));

        driveController.PS().onTrue(driveBase.resetOnlyDirection());

        driveController.povRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACKWARDS));
        driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_LEFT));
        driveController.povUpRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_RIGHT));
        driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_LEFT));
        driveController.povDownRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_RIGHT));
        

        driveController.R1().whileTrue(
            Commands.parallel(
                new AimDriving(driveBase, driveController, RobotContainer::getShootingAngle),
                preShooting.get().andThen(shootWithBump.get())
            ).withName("Full Shooting")
        );
        driveController.cross().whileTrue(preShooting.get().andThen(shootCommandWithNoVision.get()));

        driveController.circle().whileTrue(preShooting.get().andThen(shootCommandWithNoVision2.get()));

        driveController.create().onTrue(new InstantCommand(() -> withAutoEject = !withAutoEject));
        driveController.options().onTrue(swapIntakeStateClosed.get());

        driveController.L1().whileTrue(feeder.intakeCommand().alongWith(Commands.startEnd(DriveCommand::slowSpeed, DriveCommand::fullSpeed)));
        driveController.square().whileTrue(feeder.passEjectCommand());
    }

    public static void configureNamedCommands(){
        NamedCommands.registerCommand("close intake", pivot.moveToStateCommand(PivotStates.Closed));
        NamedCommands.registerCommand("open intake", pivot.moveToStateCommand(PivotStates.Open).andThen(Commands.waitSeconds(0.6))); // was .6 but matan calistenics said it was too much but i dont know i dont want to destroy the intake so its not my responsibility 

        NamedCommands.registerCommand("start rollers", new InstantCommand(() -> rollers.setDutyCycle(1)));
        NamedCommands.registerCommand("stop rollers", new InstantCommand(() -> rollers.stopMotor()));

        NamedCommands.registerCommand("shoot to hub", shootWithBump.get().withTimeout(6));
        NamedCommands.registerCommand("warm up shooter", warmupShooter.get());
        NamedCommands.registerCommand("shoot to pass", passCommand.get());
        NamedCommands.registerCommand("aim to hub", new AimDriving(driveBase, driveController, RobotContainer::angleToHub).withTimeout(0.5));

        NamedCommands.registerCommand("eject", feeder.ejectCommand());
    }
    //#endregion

    //#region Choosers
    public static void configureChoosers(){
        List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
        List<PathPlannerAuto> listOfAutos = new ArrayList<>();

        autoChooser = new LoggedDashboardChooser<>("chooser");
        for (String autoName : namesOfAutos) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
           listOfAutos.add(auto);
        }

        listOfAutos.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> Robot.clearObjectPoseField("AutoPath")).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate).onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));

        //configure the chooser for the side relative to the driver station.
        //the default is right because the routes are planned for right.

        List<PathPlannerAuto> mirroredListOfAutos = new ArrayList<>();
        for (String autoName : namesOfAutos) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName, true);
            mirroredListOfAutos.add(auto);
         }
        for (int i = 0; i < namesOfAutos.size(); i++){
            mirroredAutoMap.put(namesOfAutos.get(i), mirroredListOfAutos.get(i));
        }

        sideChooser = new LoggedDashboardChooser<>("sideChooser");

        sideChooser.addDefaultOption("right", "right");
        sideChooser.addOption("left", "left");

        SmartDashboard.putData("sideChooser",sideChooser.getSendableChooser());
    }

    private static void updateFieldFromAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();

        try {
            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
                poses.addAll(path.getPathPoses());
            });
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Error loading auto path", e.getStackTrace());
        }

        Robot.setTrajectoryField("AutoPath", poses);
    }

    private static final BooleanSupplier checkForPathChoiceUpdate = new BooleanSupplier() {
        private String lastAutoName = "InstantCommand";

        @Override
        public boolean getAsBoolean() {
            if (autoChooser.get() == null) return false;

            String currentAutoName = autoChooser.get().getName();

            try {
                return !Objects.equals(lastAutoName, currentAutoName);
            } finally {
                lastAutoName = currentAutoName;
            }

        }
    };

    public static Command getAuto(){
        return sideChooser.get().equals("right") ?
            autoChooser.get() : mirroredAutoMap.get(autoChooser.get().getName());
    }
    //#endregion

    //#region Shooting Helpers
    public static Distance distanceFromHub(){
        Pose2d hubLocation = Constants.FieldConstants.HUB_POSITION;
        if(Robot.isRedAlliance){
            hubLocation = FlippingUtil.flipFieldPose(hubLocation);
        }
        return Meters.of(
            driveBase.getPose().getTranslation().getDistance(hubLocation.getTranslation())
        );
    }
    public static Distance glida()
    {
        return Meters.of(2.15);
    }

    public static Distance glida2()
    {
        return Meters.of(3);
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
    //#endregion

    //#region Auto
    public static void configureTrenchPaths(){
        try{
            upLeftTrenchPath = PathPlannerPath.fromPathFile("UpLeft");
            upRightTrenchPath = PathPlannerPath.fromPathFile("UpRight");
        }
        catch (IOException | ParseException | FileVersionException e){
            DriverStation.reportError("Error loading trench path", e.getStackTrace());
            upLeftTrenchPath = PathPlannerPath.fromPathPoints(new ArrayList<>(), PathConstraints.unlimitedConstraints(12.0), new GoalEndState(0, Rotation2d.kZero));
            upRightTrenchPath = PathPlannerPath.fromPathPoints(new ArrayList<>(), PathConstraints.unlimitedConstraints(12.0), new GoalEndState(0, Rotation2d.kZero));
        }
    }

    public static Command passTrench(){
        Pose2d pose = driveBase.getPose();
        PathPlannerPath targetPath;

        if(Robot.isRedAlliance) pose = FlippingUtil.flipFieldPose(driveBase.getPose());
        Logger.recordOutput("Trench/Flipped Pose", pose);

        PathPlannerPath path = pose.getX() <= 4.5 ? upLeftTrenchPath : upRightTrenchPath;
        targetPath = pose.getY() >= 4 ? path : path.mirrorPath();
        Logger.recordOutput("Trench/Trench Path", targetPath.getPathPoses().toArray(new Pose2d[0]));

        PathPlannerPath flippedTargetPath = Robot.isRedAlliance ? targetPath.flipPath() : targetPath;
        Logger.recordOutput("Trench/Flipped Trench Path", flippedTargetPath.getPathPoses().toArray(new Pose2d[0]));

        return driveBase.pathFindToPathAndFollow(targetPath);
   }

    public static boolean inAllianceZone(){
        if (Robot.isRedAlliance) return driveBase.getPose().getX() > Constants.FieldConstants.RED_ALLIANCE_ZONE_X;
        else return driveBase.getPose().getX() < Constants.FieldConstants.BLUE_ALLIANCE_ZONE_X;
    }
   //#endregion

    //#region Periodic Updates
    public static void pollAlerts(){
        controllerDisconnected.set(!driveController.isConnected());
        brownoutBattery.set(RobotController.isBrownedOut());
    }

    public static void updateElastic(){
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot Velocity", driveBase.getVelocity());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("PDH Voltage", ConduitApi.getInstance().getPDPVoltage());

        SmartDashboard.putBoolean("Is Hub Active", HubTracker.isActive(DriverStation.getAlliance().orElseGet(() -> Alliance.Red)));
        SmartDashboard.putNumber("Time left in Shift", Math.floor(HubTracker.timeRemainingInCurrentShift().orElseGet(() -> Second.zero()).in(Second) * 100) / 100);
        SmartDashboard.putString("Current Shift", HubTracker.getCurrentShift().orElseGet(() -> HubTracker.Shift.AUTO).toString());

        SmartDashboard.putNumber("Fuel Approximation", fuelApproximation);
        SmartDashboard.putString("Distance to Hub", String.format("%.2f", distanceFromHub().in(Meters)));
        SmartDashboard.putBoolean("In Alliance Zone", inAllianceZone());
        SmartDashboard.putBoolean("Auto Ejecting", withAutoEject);
    }

    public static void updateLogging(){
        Logger.recordOutput("Shooting/Angle to Hub", angleToHub());
        Logger.recordOutput("Shooting/Distance To Hub", distanceFromHub().in(Meters));
        Logger.recordOutput("Shooting/In Alliance Zone", inAllianceZone());
        Logger.recordOutput("Shooting/Auto Ejecting", withAutoEject);
        Logger.recordOutput("Shooting/Fuel Approximation", fuelApproximation);
    }
    //#endregion
}
