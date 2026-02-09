// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.LinearVelocity;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {
    MarinersController leadMotor;
    MarinersController[] followMotors;


    public ShooterIOReal(){
        configureShooterMotors();
    }

    //#region Configuration
    public void configureShooterMotors(){
        leadMotor = new MarinersTalonFX(
            "Shooter Motor 1", ShooterConstants.MOTOR_CONSTANTS.CONTROLLER_LOCATION,
            ShooterConstants.MOTOR_CONSTANTS.LEAD_MOTOR_ID, ShooterConstants.MOTOR_CONSTANTS.PID,
            ShooterConstants.MOTOR_CONSTANTS.GEAR_RATIO
            );
        leadMotor.setMotorInverted(ShooterConstants.MOTOR_CONSTANTS.LEAD_MOTOR_IS_INVERTED);
        leadMotor.setMotorIdleMode(false);

        leadMotor.setStaticFeedForward(ShooterConstants.MOTOR_CONSTANTS.Ks);
        leadMotor.setFeedForward(ShooterConstants.MOTOR_CONSTANTS.Kv);

        leadMotor.startPIDTuning();

        MarinersTalonFX[] followMotors = new MarinersTalonFX[3];
        for (int i = 0; i < followMotors.length; i++){
            var follower = ShooterConstants.MOTOR_CONSTANTS.FOLLOWERS[i];

            followMotors[i] = new MarinersTalonFX(
                "Shooter Follower Motor " + Integer.toString(i + 1),
                ShooterConstants.MOTOR_CONSTANTS.CONTROLLER_LOCATION,
                follower.id()
            );
            followMotors[i].setMotorAsFollower(leadMotor, follower.inverted_from_leader());
        }
    }

    //#endregion

    public double getVelocity(){
        return leadMotor.getVelocity();
    }
    public void setVelocity(double targetVelocity){
        leadMotor.setReference(targetVelocity, ControlMode.Velocity);
    }
    public void setVoltage(double voltage){
        leadMotor.setVoltage(voltage);
    }
    public void setDutyCycle(double targetDutyCycle){
        leadMotor.setReference(targetDutyCycle, ControlMode.DutyCycle);
    }
    public void feedForwardBoost(double boost){
        leadMotor.setStaticFeedForward(leadMotor.getPIDF().getF() + boost);
    }
    public void resetFeedForward(){
        leadMotor.setStaticFeedForward(ShooterConstants.MOTOR_CONSTANTS.PID.getF());
    }


    public void update(ShooterInputs inputs){
        inputs.shooterVelocity = RPM.of(getVelocity());

        LinearVelocity shooterLinearVelocity = Meter.per(Minute).of(
            getVelocity() * ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE.in(Meter)
        );
        inputs.shooterLinearVelocity = shooterLinearVelocity;

        inputs.feedForward = leadMotor.getPIDF().getF();

        inputs.pose = ShooterConstants.POSITION;
    }
}
