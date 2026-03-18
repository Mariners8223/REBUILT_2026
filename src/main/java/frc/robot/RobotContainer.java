// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.Set;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Climb.RunHookToHeight;
import frc.robot.commands.Drive.AimDriving;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.DriveToPose;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.commands.Shooter.ShootDistance;
import frc.robot.commands.Shooter.ShootVelocity;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants.ClimbStates;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakeStates;
import frc.robot.subsystems.Kicker.Kicker;


public class RobotContainer {
    public static Alert controllerDisconnected = new Alert("Drive Controller Disconnect", AlertType.kError);

    public static CommandPS5Controller driveController;
    public static Constants.autoConstats.TrenchLocations trenchLocations = new Constants.autoConstats.TrenchLocations();

    public static DriveBase driveBase;
    public static Funnel funnel;
    public static Intake intake;
    public static Shooter shooter;
    public static Kicker kicker;
    public static Climb climb;

    public static Feeder feeder;
    public static Vision vision;

    public static LoggedDashboardChooser<Command> autoChooser;
    public static LoggedDashboardChooser<String> sideChooser;
    public static HashMap<String, Command> mirroredAutoMap = new HashMap<>();

    public static Supplier<Command> fullShootCommand;
    public static Supplier<Command> passCommand;
    public static Supplier<Command> fullClimb;


    public RobotContainer() {
        driveController = new CommandPS5Controller(0);

        driveBase = new DriveBase();
        climb = new Climb();
        funnel = new Funnel();
        intake = new Intake();
        shooter = new Shooter();
        kicker = new Kicker();

        feeder = new Feeder(intake, funnel, kicker);
        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose);

        configureCommands();
        configureNamedCommands();
        configureDriveBindings();
        //configureTestBindings();
        configureChoosers();
        configureMirroredAutosMap();
    }

    public static void configureCommands(){
        fullShootCommand = () ->
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

        Supplier<Command> hookToTower = () ->
            Commands.sequence(
                AutoBuilder.pathfindToPose(Constants.FieldConstants.PRE_PRE_TOWER_POSITION, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS),
                new DriveToPose(driveBase, Constants.FieldConstants.PRE_PRE_TOWER_POSITION, 0.1).withTimeout(2),
                new DriveToPose(driveBase, Constants.FieldConstants.PRE_TOWER_POSITION, 0.1).withTimeout(2),
                new DriveToPose(driveBase, Constants.FieldConstants.TOWER_POSITION, 0.1).withTimeout(2),
                new DriveToPose(driveBase, Constants.FieldConstants.INNER_TOWER_POSITION, 0.1).withTimeout(2)
            );

        fullClimb = () ->
            Commands.sequence(
                // new ResetHook(climb),
                climb.toStateCommand(ClimbStates.EXTENDED),
                hookToTower.get(),
                new RunHookToHeight(climb, ClimbStates.RESET, 1)
            );
    }

    public static void pollAlerts(){
        controllerDisconnected.set(!driveController.isConnected());
    }

    // for tests!!
    public static void configureTestBindings(){}

    //#region Drive
    public static void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() ->driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));

        driveController.PS().onTrue(driveBase.resetOnlyDirection());

        driveController.options().whileTrue(climb.dutyCycleCommand(0.1));
        driveController.create().whileTrue(climb.dutyCycleCommand(-0.1));
        driveController.create().multiPress(2, 0.5).onTrue(climb.toStateCommand(ClimbStates.RESET));

        driveController.povRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACKWARDS));
        driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_LEFT));
        driveController.povUpRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_RIGHT));
        driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_LEFT));
        driveController.povDownRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_RIGHT));


        driveController.L1().toggleOnTrue(
            Commands.either(
                fullShootCommand.get(),
                passCommand.get(),
                RobotContainer::inAllianceZone
            )
        );

        driveController.R1().toggleOnTrue(
            new AimDriving(driveBase, driveController, RobotContainer::getShootingAngle)
        );

        driveController.circle().toggleOnTrue(feeder.intakeCommand());
        driveController.circle().multiPress(2, 0.5).onTrue(feeder.ejectCommand());

        driveController.square().toggleOnTrue(Commands.defer(RobotContainer::passTrench, Set.of(driveBase)));
        driveController.triangle().whileTrue(fullClimb.get());

        driveController.cross().onTrue(
            Commands.either(
                intake.moveToPositionCommand(IntakeStates.Closed),
                intake.moveToPositionCommand(IntakeStates.Open),
                () -> intake.getCurrentState() == IntakeStates.Open
            )
        );
        driveController.R3().onTrue(intake.bumpFuelCommand());
    }
    //#endregion

    public static void configureNamedCommands(){
        NamedCommands.registerCommand("close intake", intake.moveToPositionCommand(IntakeStates.Closed));
        NamedCommands.registerCommand("open intake", intake.moveToPositionCommand(IntakeStates.Open).andThen(Commands.waitSeconds(0.7)));

        NamedCommands.registerCommand("start rollers", new InstantCommand(() -> intake.setRollersDutyCycle(IntakeConstants.RollersMotor.DUTY_CYCLE)));
        NamedCommands.registerCommand("stop rollers", new InstantCommand(() -> intake.setRollersDutyCycle(0)));

        NamedCommands.registerCommand("shoot to hub", fullShootCommand.get().withTimeout(3));
        NamedCommands.registerCommand("warm up shooter", new InstantCommand(() -> shooter.setVelocityByDistance(distanceFromHub()), shooter));
        NamedCommands.registerCommand("shoot to pass", passCommand.get());
        NamedCommands.registerCommand("aim to hub", new AimDriving(driveBase, driveController, RobotContainer::angleToHub).withTimeout(0.8));

        NamedCommands.registerCommand("climb auto", fullClimb.get());

        NamedCommands.registerCommand("eject", feeder.ejectCommand());
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
    public static Command passTrench(){
        Pose2d pose = driveBase.getPose();
        Pose2d targetPose;

        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            pose = FlippingUtil.flipFieldPose(driveBase.getPose());
        }
        if (pose.getY() >= 4){
            if (pose.getX() <= 4.5) targetPose = trenchLocations.upRightTrench;
            else targetPose = trenchLocations.upLeftTrench;
        }
        else{
            if (pose.getX() <= 4.5) targetPose = trenchLocations.downRightTrench;
            else targetPose = trenchLocations.downLeftTrench;
        }

        Pose2d flippedTargetPose = Robot.isRedAlliance ? FlippingUtil.flipFieldPose(targetPose) : targetPose;
        return driveBase.findPath(flippedTargetPose, 2);
   }

    public static boolean inAllianceZone(){
        if (Robot.isRedAlliance) return driveBase.getPose().getX() > Constants.FieldConstants.RED_ALLIANCE_ZONE_X;
        else return driveBase.getPose().getX() < Constants.FieldConstants.BLUE_ALLIANCE_ZONE_X;
    }
   //#endregion
}
