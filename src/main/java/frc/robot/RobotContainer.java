// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BiFunction;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Drive.AimDriving;
import frc.robot.commands.Climb.RunHookToHeight;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.commands.Kicker.BlinkingKicker;
import frc.robot.commands.Shooter.ShootVelocity;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;
import frc.robot.subsystems.DriveTrain.DriveBaseSYSID;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants.PathPlanner;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterSysID;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
import frc.robot.subsystems.Kicker.Kicker;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class RobotContainer {
    public static CommandPS5Controller driveController;

    public static DriveBase driveBase;
    public static Funnel funnel;
    public static Intake intake;
    public static Shooter shooter;
    public static Kicker kicker;
    public static Climb climb;

    public static DriveBaseSYSID driveSysID;
    public static CommandXboxController driveXboxController;
    public static Vision vision;

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
        //configureTestingBindings();
        configureDriveSysidBindings();

        Logger.recordOutput("Hub Pose", Constants.FieldConstants.HUB_POSITION);
    }

    public Distance distanceFromHub(){
        return Meters.of(driveBase.getPose().getTranslation().getDistance(Constants.FieldConstants.HUB_POSITION.getTranslation().toTranslation2d()));
    }

    public Distance distanceFromHub(){
        return Meters.of(
            driveBase.getPose().getTranslation().getDistance(Constants.FieldConstants.HUB_POSITION)
        );
    }
    public Distance distanceFromHubWithVelocity(){
        Distance distance = distanceFromHub();
        double chassisVelocityX = driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond;
        double chassisVelocityY = driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond;

        return Meters.zero();
    }


    //#region Drive
    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() -> driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));
        driveController.povUp().onTrue(driveBase.resetOnlyDirection());
        driveController.R1().toggleOnTrue(new AimDriving(driveBase, driveController, () -> {return Constants.FieldConstants.HUB_POSITION.toPose2d();} ));
        // Optional<PathPlannerPath> path = Optional.empty();
        // try {
        //     path = Optional.of(PathPlannerPath.fromPathFile("pointhub"));
        // } catch (FileVersionException | IOException | ParseException e) {
        //     System.err.println("Failed to load path: " + e.getMessage());
        // }

        // if (path.isPresent()) {
        //     driveController.square().onTrue(driveBase.pathFindToPathAndFollow(path.get()));
        // }

        driveController.options().onTrue(driveBase.resetOnlyDirection());

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
            driveSysID.getThetaRoutineDynamic(Direction.kForward)
        );

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
    //#endregion

    //#region Shooter
    public void configureShooterSysIDBindings(){
        driveController.cross().whileTrue(shooterSysID.getShooterDynamic(Direction.kReverse));
        driveController.triangle().whileTrue(shooterSysID.getShooterDynamic(Direction.kForward));

        driveController.circle().whileTrue(shooterSysID.getShooterQuasistatic(Direction.kForward));
        driveController.square().whileTrue(shooterSysID.getShooterQuasistatic(Direction.kReverse));
    }

    public void configureShooterTestBindings(){
        SmartDashboard.putNumber("Shooter RPM", 0);
        SmartDashboard.putNumber("Kicker Duty Cycle", 0);

        driveController.cross().toggleOnTrue(
            Commands.startEnd(
                () -> shooter.setVelocity(RPM.of(SmartDashboard.getNumber("Shooter RPM", 0))),
                () -> shooter.stopShooter(),
                shooter
            )
        );
        driveController.triangle().toggleOnTrue(
            kicker.setKickerCommand(0.5)
        );
    }
    //#endregion


    public void configureTestingBindings(){
        SmartDashboard.putNumber("Shooter Velocity", 0);
        SmartDashboard.putNumber("Kicker Duty Cycle", 0.6);

        driveController.square().toggleOnTrue(Commands.parallel(
            funnel.onlyCenterCommand(),
            kicker.setKickerCommand(SmartDashboard.getNumber("Kicker Duty Cycle", 0.6)),
            new ShootVelocity(shooter, () -> RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0)))
            ));
        
        driveController.circle().toggleOnTrue(Commands.parallel(
            funnel.toShooterCommand(),
            kicker.setKickerCommand(SmartDashboard.getNumber("Kicker Duty Cycle", 0.6)),
            new ShootVelocity(shooter, () -> RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0)))
        ));
        driveController.triangle().whileTrue(new ShootVelocity(shooter, () -> RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0))));
            
        driveController.L1().onTrue(intake.moveToPositionCommand(IntakePosition.Closed));
        driveController.R1().onTrue(intake.moveToPositionCommand(IntakePosition.Middle));
        driveController.L2().onTrue(intake.moveToPositionCommand(IntakePosition.Open));

        // driveController.R1().onTrue(intake.moveToPositionCommand(IntakePosition.Closed));
        // driveController.L1().onTrue(intake.moveToPositionCommand(IntakePosition.Open));

        // driveController.triangle().toggleOnTrue(
        //     Commands.parallel(funnel.funnelingCommand(), intake.spinRollersCommand())
        // );



        //Command kickerCommand = kicker.setKickerCommand(SmartDashboard.getNumber("Kicker Duty Cycle", 0.6));
        
        // driveController.PS().whileTrue(new InstantCommand(() -> kicker.setMotors(SmartDashboard.getNumber("Kicker RPM", 0.6))));

        // Command everythingIntoShooter = Commands.sequence(
                // intake.moveToPositionCommand(IntakePosition.Middle),
                // new WaitCommand(3),
                // Commands.parallel(
                //     funnel.toShooterCommand(),
                //     kicker.setKickerCommand(0.8),
                //     intake.spinRollersCommand()
                // ));
            // ).finallyDo(() -> intake.setPositionMotorState(IntakePosition.Open));

        // driveController.square().toggleOnTrue(
        //     Commands.parallel(
        //         new ShootVelocity(shooter, () -> RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0))),
        //         new ProxyCommand(everythingIntoShooter)
        //     )
        // );


        // driveController.povRight().whileTrue(Commands.run(() -> shooter.setVelocityWithFeedforward(RPM.of(1000), 4)));

        // driveController.cross().toggleOnTrue(
        //     Commands.parallel(
        //         Commands.run(() -> shooter.setVelocity(RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0))), shooter),
        //         new ProxyCommand(everythingIntoShooter)
        //     )
        // );

        // driveController.povUp().onTrue(Commands.parallel(
        //     new InstantCommand(() -> shooter.boostFeedForward(1)),
        //     new PrintCommand("Boost"))
        // );
        // driveController.cross().toggleOnTrue();
   }
    public void configureClimbTestBindings(){
        driveController.povRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACKWARDS));
        driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_LEFT));
        driveController.povUpRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_RIGHT));
        driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_LEFT));
        driveController.povDownRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_RIGHT));


        driveController.triangle().onTrue(climb.toPositionCommand(Heights.EXTENDED));

        driveController.R2().whileTrue(new RunHookToHeight(climb, Heights.RESET, 1));
        driveController.L2().whileTrue(new RunHookToHeight(climb, Heights.IN_AIR_AUTO, 1));

        driveController.square().whileTrue(new RunHookToHeight(climb, Heights.EXTENDED, ClimbConstants.GETTING_DOWN_DUTY_CYCLE));

        driveController.options().onTrue(new InstantCommand(() -> climb.resetPosition()));
   }
}
