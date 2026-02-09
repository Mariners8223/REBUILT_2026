// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.LinearVelocity;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {
    MarinersController leadMotor;
    MarinersController[] followMotors;

    MarinersController kickerMotor1;
    MarinersController kickerMotor2;

    public ShooterIOReal(){
        configureShooterMotors();
        configureKickerMotors();
    }

    //#region Configuration
    public void configureShooterMotors(){
        leadMotor = new MarinersTalonFX(
            "Shooter Motor 1", ShooterConstants.SHOOTER_MOTOR.CONTROLLER_LOCATION,
            ShooterConstants.SHOOTER_MOTOR.LEAD_MOTOR_ID, ShooterConstants.SHOOTER_MOTOR.PID,
            ShooterConstants.SHOOTER_MOTOR.GEAR_RATIO
            );
        leadMotor.setMotorInverted(ShooterConstants.SHOOTER_MOTOR.LEAD_MOTOR_IS_INVERTED);
        leadMotor.setMotorIdleMode(false);

        leadMotor.setStaticFeedForward(ShooterConstants.SHOOTER_MOTOR.Ks);
        leadMotor.setFeedForward(ShooterConstants.SHOOTER_MOTOR.Kv);

        leadMotor.startPIDTuning();

        MarinersTalonFX[] followMotors = new MarinersTalonFX[3];
        for (int i = 0; i < followMotors.length; i++){
            var follower = ShooterConstants.SHOOTER_MOTOR.FOLLOWERS[i];

            followMotors[i] = new MarinersTalonFX(
                "Shooter Follower Motor " + Integer.toString(i + 1),
                ShooterConstants.SHOOTER_MOTOR.CONTROLLER_LOCATION,
                follower.id()
            );
            followMotors[i].setMotorAsFollower(leadMotor, follower.inverted_from_leader());
        }
    }

    public void configureKickerMotors(){
        kickerMotor1 = new MarinersSparkBase(
            "Kicker Motor 1", ShooterConstants.KICKER_MOTOR.CONTROLLER_LOCATION,
            ShooterConstants.KICKER_MOTOR.MOTOR_1_ID, true, MotorType.SPARK_FLEX);
        kickerMotor1.setMotorInverted(ShooterConstants.KICKER_MOTOR.MOTOR_1_IS_INVERTED);

        kickerMotor2 = new MarinersSparkBase(
            "Kicker Motor 2", ShooterConstants.KICKER_MOTOR.CONTROLLER_LOCATION,
            ShooterConstants.KICKER_MOTOR.MOTOR_2_ID, true, MotorType.SPARK_FLEX);
        kickerMotor2.setMotorAsFollower(kickerMotor1, ShooterConstants.KICKER_MOTOR.MOTOR_2_INVERTED_FROM_LEADER);
    }
    //#endregion

    public double getShooterVelocity(){
        return leadMotor.getVelocity();
    }
    public void setShooterVelocity(double targetVelocity){
        leadMotor.setReference(targetVelocity, ControlMode.Velocity);
    }
    public void setShooterVoltage(double voltage){
        leadMotor.setVoltage(voltage);
    }
    public void setShooterDutyCycle(double targetDutyCycle){
        leadMotor.setReference(targetDutyCycle, ControlMode.DutyCycle);
    }
    public void shooterFeedForwardBoost(double boost){
        leadMotor.setStaticFeedForward(leadMotor.getPIDF().getF() + boost);
    }
    public void resetShooterFeedForward(){
        leadMotor.setStaticFeedForward(ShooterConstants.SHOOTER_MOTOR.PID.getF());
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
    public void setKickerVoltage(double voltage){
        kickerMotor1.setVoltage(voltage);
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
