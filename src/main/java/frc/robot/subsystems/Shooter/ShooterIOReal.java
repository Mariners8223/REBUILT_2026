// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersController.ControllerLocation;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {
    MarinersController shooterMotor1;
    MarinersController shooterMotor2;

    MarinersController kickerMotor1;
    MarinersController kickerMotor2;

    public ShooterIOReal(){
        configureShooterMotors();
        configureKickerMotors();
    }

    //#region Configuration
    public void configureShooterMotors(){
        shooterMotor1 = new MarinersTalonFX(
            "Shooter Motor 1", ShooterConstants.SHOOTER_MOTOR.CONTROLLER_LOCATION,
            ShooterConstants.SHOOTER_MOTOR.MOTOR_1_ID, ShooterConstants.SHOOTER_MOTOR.PID,
            ShooterConstants.SHOOTER_MOTOR.GEAR_RATIO
            );
        shooterMotor1.setMotorInverted(ShooterConstants.SHOOTER_MOTOR.MOTOR_1_IS_INVERTED);

        shooterMotor2 = new MarinersTalonFX(
            "Shooter Motor 2", ShooterConstants.SHOOTER_MOTOR.CONTROLLER_LOCATION,
            ShooterConstants.SHOOTER_MOTOR.MOTOR_2_ID);
        shooterMotor2.setMotorAsFollower(shooterMotor1, ShooterConstants.SHOOTER_MOTOR.MOTOR_2_IS_INVERTED);
    }

    public void configureKickerMotors(){
        kickerMotor1 = new MarinersTalonFX(
            "Kicker Motor 1", ShooterConstants.KICKER_MOTOR.CONTROLLER_LOCATION,
            ShooterConstants.KICKER_MOTOR.MOTOR_1_ID, ShooterConstants.KICKER_MOTOR.PID,
            ShooterConstants.KICKER_MOTOR.GEAR_RATIO
            );
        kickerMotor1.setMotorInverted(ShooterConstants.KICKER_MOTOR.MOTOR_1_IS_INVERTED);

        kickerMotor2 = new MarinersTalonFX(
            "Kicker Motor 2", ShooterConstants.KICKER_MOTOR.CONTROLLER_LOCATION,
            ShooterConstants.KICKER_MOTOR.MOTOR_2_ID);
        kickerMotor2.setMotorAsFollower(kickerMotor1, ShooterConstants.KICKER_MOTOR.MOTOR_2_IS_INVERTED);
    }
    //#endregion

    public double getShooterVelocity(){
        return shooterMotor1.getVelocity();
    }
    public void setShooterVelocity(double targetVelocity){
        shooterMotor1.setReference(targetVelocity, ControlMode.Velocity);
    }

    public double getKickerVelocity(){
        return kickerMotor1.getVelocity();
    }
    public void setKickerVelocity(double targetVelocity){
        kickerMotor1.setReference(targetVelocity, ControlMode.Velocity);
    }
    public void setKickerDutyCycle(double targetDutyCycle){
        kickerMotor1.setReference(targetDutyCycle, ControlMode.DutyCycle);
    }

    public void update(ShooterInputs inputs){
        inputs.shooterVelocity = RPM.of(getShooterVelocity());

        LinearVelocity shooterLinearVelocity = Meter.per(Minute).of(
            getShooterVelocity() * ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE.in(Meter)
        );
        inputs.shooterLinearVelocity = shooterLinearVelocity;

        
        inputs.kickerVelocity = RPM.of(getKickerVelocity());

        LinearVelocity kickerLinearVelocity = Meter.per(Minute).of(
            getKickerVelocity() * ShooterConstants.KICKER_WHEEL_CIRCUMFERENCE.in(Meter)
        );
        inputs.kickerLinearVelocity = kickerLinearVelocity;

        inputs.pose = ShooterConstants.POSITION;
    }
}
