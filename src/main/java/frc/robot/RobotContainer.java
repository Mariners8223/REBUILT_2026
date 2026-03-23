// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
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
import frc.robot.commands.Drive.AimDriving;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.commands.Shooter.ShootDistance;
import frc.robot.commands.Shooter.ShootVelocity;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Vision.Vision;
import frc.util.HubTracker;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakeStates;
import frc.robot.subsystems.Kicker.Kicker;


public class RobotContainer {
    public static Alert controllerDisconnected = new Alert("Drive Controller Disconnect", AlertType.kError);
    public static Alert brownoutBattery = new Alert("BROWNOUT", AlertType.kError);

    public static CommandPS5Controller driveController;
    public static Constants.AutoConstants.TrenchLocations trenchLocations = new Constants.AutoConstants.TrenchLocations();

    public static DriveBase driveBase;
    public static Funnel funnel;
    public static Intake intake;
    public static Shooter shooter;
    public static Kicker kicker;

    public static Feeder feeder;
    public static Vision vision;

    public static LoggedDashboardChooser<Command> autoChooser;
    public static LoggedDashboardChooser<String> sideChooser;
    public static HashMap<String, Command> mirroredAutoMap = new HashMap<>();

    public static Supplier<Command> shootCommand;
    public static Supplier<Command> passCommand;
    public static Supplier<Command> fullShootCommand;


    public RobotContainer() {
        driveController = new CommandPS5Controller(0);

        driveBase = new DriveBase();
        funnel = new Funnel();
        intake = new Intake();
        shooter = new Shooter();
        kicker = new Kicker();

        feeder = new Feeder(intake, funnel, kicker);
        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose);

        // TODO: Passing through rollers?

        configureCommands();
        configureNamedCommands();
        configureDriveBindings();
        configureChoosers();
        configureMirroredAutosMap();
    }

    public static void configureCommands(){
        shootCommand = () ->
            Commands.parallel(
                new ShootDistance(shooter, RobotContainer::distanceFromHub),
                Commands.waitUntil(() -> shooter.isAtRequiredVelocity(RobotContainer.distanceFromHub()))
                .andThen(feeder.smartFeedingShootCommand(RobotContainer::distanceFromHub))
            );

        passCommand = () ->
            Commands.parallel(
                new ShootVelocity(shooter, () -> RPM.of(ShooterConstants.PASSING_VELOCITY)),
                Commands.waitUntil(() -> shooter.isAtRequiredVelocity(RobotContainer.distanceFromHub()))
                .andThen(feeder.smartFeedingPassCommand())
            );

        fullShootCommand = () ->
            Commands.either(
                shootCommand.get(),
                passCommand.get(),
                RobotContainer::inAllianceZone
            );
    }

    //#region Periodic Updates
    public static void pollAlerts(){
        controllerDisconnected.set(!driveController.isConnected());
        brownoutBattery.set(!RobotController.isBrownedOut());
    }

    public static void updateElastic(){
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot Velocity", driveBase.getVelocity());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("PDH Voltage", ConduitApi.getInstance().getPDPVoltage());

        SmartDashboard.putBoolean("Is Hub Active", HubTracker.isActive(DriverStation.getAlliance().orElseGet(() -> Alliance.Red)));
        SmartDashboard.putNumber("Time left in Shift", Math.floor(HubTracker.timeRemainingInCurrentShift().orElseGet(() -> Second.zero()).in(Second) * 100) / 100);
        SmartDashboard.putString("Current Shift", HubTracker.getCurrentShift().orElseGet(() -> HubTracker.Shift.AUTO).toString());

        SmartDashboard.putString("Distance to Hub", String.format("%.2f", distanceFromHub().in(Meter)));
        SmartDashboard.putBoolean("In Alliance Zone", inAllianceZone());
    }

    public static void updateLogging(){
        Logger.recordOutput("Distance To Hub", distanceFromHub().in(Meters));
    }
    //#endregion

    //#region Drive
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

        driveController.L1().toggleOnTrue(
            new AimDriving(driveBase, driveController, RobotContainer::getShootingAngle).alongWith(
                feeder.ejectCommand().withTimeout(0.5).andThen(
                    Commands.parallel(
                        fullShootCommand.get(),
                        intake.bumpFuelCommand().repeatedly().beforeStarting(Commands.waitSeconds(2))
                    )
                )
            ).finallyDo(() -> intake.moveToPositionCommand(IntakeStates.Open))
        );

        driveController.circle().toggleOnTrue(feeder.intakeCommand());
        driveController.square().toggleOnTrue(feeder.ejectCommand());

        driveController.triangle().whileTrue(Commands.defer(RobotContainer::passTrench, Set.of(driveBase)));

        driveController.options().onTrue(
            Commands.either(
                intake.moveToPositionCommand(IntakeStates.Closed),
                intake.moveToPositionCommand(IntakeStates.Open),
                () -> intake.getCurrentState() == IntakeStates.Open
            )
        );
        driveController.cross().onTrue(intake.bumpFuelCommand());
    }
    //#endregion

    public static void configureNamedCommands(){
        NamedCommands.registerCommand("close intake", intake.moveToPositionCommand(IntakeStates.Closed));
        NamedCommands.registerCommand("open intake", intake.moveToPositionCommand(IntakeStates.Open).andThen(Commands.waitSeconds(0.6)));

        NamedCommands.registerCommand("start rollers", new InstantCommand(() -> intake.setRollersDutyCycle(IntakeConstants.RollersMotor.DUTY_CYCLE)));
        NamedCommands.registerCommand("stop rollers", new InstantCommand(() -> intake.setRollersDutyCycle(0)));

        NamedCommands.registerCommand("shoot to hub", shootCommand.get().
            // alongWith(intake.bumpFuelCommand().repeatedly().beforeStarting(Commands.waitSeconds(2))).
            withTimeout(4)
            );
        NamedCommands.registerCommand("warm up shooter", new InstantCommand(() -> shooter.setVelocityByDistance(distanceFromHub()), shooter));
        NamedCommands.registerCommand("shoot to pass", passCommand.get());
        NamedCommands.registerCommand("aim to hub", new AimDriving(driveBase, driveController, RobotContainer::angleToHub).withTimeout(0.2));

        NamedCommands.registerCommand("eject", feeder.ejectCommand());
    }

    //#region Choosers
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

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> Robot.clearObjectPoseField("AutoPath")).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate).onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));

        //configure the chooser for the side relative to the driver station.
        //the default is right because the routes are planned for right.
        sideChooser = new LoggedDashboardChooser<>("sideChooser");

        sideChooser.addDefaultOption("right", "right");
        sideChooser.addOption("middle", "middle");
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
        return getSide().equals("right") || getSide().equals("middle") ?
            autoChooser.get() : mirroredAutoMap.get(autoChooser.get().getName());
    }

    public static String getSide(){
        return sideChooser.get();
    }
    //#endregion

    //#region Shoot Helpers
    public static Distance distanceFromHub(){
        Pose2d hubLocation = Constants.FieldConstants.HUB_POSITION;
        if(Robot.isRedAlliance){
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

    public static boolean inRange(){
        Pose2d pose = driveBase.getPose();
        Pose2d hub = Constants.AutoConstants.hub;
        if(Robot.isRedAlliance){
            pose = FlippingUtil.flipFieldPose(driveBase.getPose());
            hub = FlippingUtil.flipFieldPose(hub);
        }
        double distanceFromHub = driveBase.getDistanceFromPoint2D(hub);

        if (pose.getX() > 4.5) return false;
        return distanceFromHub < Constants.AutoConstants.maxRange;
    }
    //#endregion

   //#region auto
    public static Command passTrench(){
        Pose2d pose = driveBase.getPose();
        PathPlannerPath targetPath;

        if(Robot.isRedAlliance) pose = FlippingUtil.flipFieldPose(driveBase.getPose());
        String pathName = pose.getX() <= 4.5 ? "UpLeft" : "UpRight";

        try{
            targetPath = PathPlannerPath.fromPathFile(pathName);
            targetPath = pose.getY() >= 4 ? targetPath : targetPath.mirrorPath();
        }
        catch (IOException | ParseException | FileVersionException e){
            DriverStation.reportError("Error loading trench path", e.getStackTrace());
            targetPath = PathPlannerPath.fromPathPoints(new ArrayList<>(), PathConstraints.unlimitedConstraints(12.0), new GoalEndState(0, Rotation2d.kZero));
        }

        PathPlannerPath flippedTargetPose = Robot.isRedAlliance ? targetPath.flipPath() : targetPath;
        return driveBase.pathFindToPathAndFollow(flippedTargetPose);
   }

    public static boolean inAllianceZone(){
        if (Robot.isRedAlliance) return driveBase.getPose().getX() > Constants.FieldConstants.RED_ALLIANCE_ZONE_X;
        else return driveBase.getPose().getX() < Constants.FieldConstants.BLUE_ALLIANCE_ZONE_X;
    }
   //#endregion
}
