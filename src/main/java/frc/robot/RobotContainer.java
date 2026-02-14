// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.BiFunction;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBaseSYSID;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
import frc.robot.subsystems.Kicker.Kicker;

public class RobotContainer {
    public static CommandPS5Controller driveController;

    public static DriveBase driveBase;
    public static Funnel funnel;
    public static Intake intake;
    public static Shooter shooter;
    public static Kicker kicker;

    public static DriveBaseSYSID driveSysID;

    public RobotContainer() {
        driveController = new CommandPS5Controller(0);

        driveBase = new DriveBase();
        funnel = new Funnel();
        intake = new Intake();
        shooter = new Shooter();
        kicker = new Kicker();

        driveSysID = new DriveBaseSYSID(driveBase, driveController);

        configureDriveBindings();
        // configureTestingBindings();
        configureDriveSysidBindings();
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

    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() ->driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));
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


    public void configureDriveSysidBindings(){
        SendableChooser<String> type = new SendableChooser<String>();
        type.addOption("Drive", "Drive");
        type.addOption("Steer", "Steer");
        type.addOption("Theta", "Theta");
        type.setDefaultOption("Drive", "Drive");

        SmartDashboard.putData(type);

        driveController.cross().whileTrue(
            new ProxyCommand(chooseCommand(type.getSelected()).apply(true, Direction.kReverse))
        );
        driveController.triangle().whileTrue(
            new ProxyCommand(chooseCommand(type.getSelected()).apply(true, Direction.kForward))
        );
        driveController.square().whileTrue(
            new ProxyCommand(chooseCommand(type.getSelected()).apply(false, Direction.kReverse))
        );
        driveController.circle().whileTrue(
            new ProxyCommand(chooseCommand(type.getSelected()).apply(false, Direction.kForward))
        );
    }



    public void configureTestingBindings(){
        SmartDashboard.putNumber("Shooter Velocity", 0);
        SmartDashboard.putNumber("Shooter Duty Cycle", 0);

        driveController.cross().toggleOnTrue(
            intake.moveToPositionCommand(
                intake.getCurrentState() == IntakePosition.Closed ?
                IntakePosition.Open : IntakePosition.Closed
            )
        );
        driveController.triangle().toggleOnTrue(
            intake.spinRollersCommand()
        );
        driveController.options().onTrue(new InstantCommand(() -> intake.resetPositionMotorEncoder()));

        driveController.square().toggleOnTrue(
            funnel.funnelingCommand()
        );
        driveController.circle().toggleOnTrue(
            funnel.toShooterCommand()
        );

        driveController.povUp().toggleOnTrue(
            // intake.moveToPositionCommand(IntakePosition.Open).andThen(
            Commands.parallel(
                // intake.spinRollersCommand(),
                // shooter.setDutyCycleCommand(SmartDashboard.getNumber("Shooter Duty Cycle", 0.2)),
                shooter.setDutyCycleCommand(0.75),
                // funnel.toShooterCommand(),a[\]


                kicker.setKickerCommand(0.2)
            )
        );

        driveController.povDown().toggleOnTrue(Commands.startEnd(
            () -> shooter.setDutyCycle(0.1),
            () -> shooter.setDutyCycle(0))
        );

        driveController.L1().toggleOnTrue(
                Commands.run(() -> shooter.setVelocity(RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0))))
                .until(() -> Constants.CALCULATIONS.epsilonEquals(
                        shooter.getShooterVelocity(),
                        RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0)),
                        RPM.of(60))
                ).
                andThen(
                    kicker.setKickerCommand(0.4)
                )
        );

        driveController.R1().toggleOnTrue(
            Commands.sequence(
                Commands.run(() -> shooter.setVelocity(RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0))))
                .until(() -> Constants.CALCULATIONS.epsilonEquals(
                        shooter.getShooterVelocity(),
                        RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0)),
                        RPM.of(60))
                ).withTimeout(1),
                Commands.parallel(
                    kicker.setKickerCommand(0.4),
                    funnel.toShooterCommand()
                )
            )
        );
   }
}
