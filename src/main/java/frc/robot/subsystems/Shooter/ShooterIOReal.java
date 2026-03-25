// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.robot.subsystems.Shooter.ShooterConstants.MOTOR_CONSTANTS;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {
    MarinersController leadMotor;
    MarinersController followMotor;


    public ShooterIOReal(){
        configureShooterMotors();
    }

    //#region Configuration
    public void configureShooterMotors(){
        leadMotor = new MarinersTalonFX(
            "Shooter/Lead Motor", ShooterConstants.MOTOR_CONSTANTS.CONTROLLER_LOCATION,
            ShooterConstants.MOTOR_CONSTANTS.LEAD_MOTOR_ID, ShooterConstants.MOTOR_CONSTANTS.PID,
            ShooterConstants.MOTOR_CONSTANTS.GEAR_RATIO
            );
        leadMotor.setMotorInverted(ShooterConstants.MOTOR_CONSTANTS.LEAD_MOTOR_IS_INVERTED);
        leadMotor.setMotorIdleMode(false);

        leadMotor.setStaticFeedForward(ShooterConstants.MOTOR_CONSTANTS.Ks);
        leadMotor.setFeedForward(ShooterConstants.MOTOR_CONSTANTS.Kv);

        leadMotor.setCurrentLimits(MOTOR_CONSTANTS.SHOOTER_MOTOR_CURRENT_LIMIT, MOTOR_CONSTANTS.SHOOTER_MOTOR_CURRENT_THRESHOLD);

        leadMotor.startPIDTuning();

        followMotor = new MarinersTalonFX("Shooter/Follower Motor", MOTOR_CONSTANTS.CONTROLLER_LOCATION, MOTOR_CONSTANTS.FOLLOW_MOTOR_ID);
        followMotor.setMotorAsFollower(leadMotor, MOTOR_CONSTANTS.IS_INVERTED_FROM_LEADER);
    }
    //#endregion

    public double getVelocity(){
        return leadMotor.getVelocity();
    }
    public double getAcceleration(){
        return leadMotor.getAcceleration();
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

    public void update(ShooterInputs inputs){}
}
