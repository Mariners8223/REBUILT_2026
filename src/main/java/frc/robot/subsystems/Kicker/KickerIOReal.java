// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.Kicker.KickerConstants.MOTOR_CONSTANTS;
import frc.util.MarinersController.MarinersSparkBase;

/** Add your docs here. */
public class KickerIOReal implements KickerIO {
    private static MarinersSparkBase leadMotor;
    private static MarinersSparkBase followMotor;

    public KickerIOReal(){
        leadMotor = new MarinersSparkBase(
            "Kicker Lead Motor", 
            KickerConstants.MOTOR_CONSTANTS.CONTROLLER_LOCATION, 
            KickerConstants.MOTOR_CONSTANTS.LEAD_MOTOR_ID, 
            true,
            KickerConstants.MOTOR_CONSTANTS.MOTOR_TYPE);
        leadMotor.setMotorInverted(KickerConstants.MOTOR_CONSTANTS.LEAD_MOTOR_IS_INVERTED);
        leadMotor.setMotorIdleMode(true);
        
        followMotor = new MarinersSparkBase(
            "Kicker Follow Motor", 
            KickerConstants.MOTOR_CONSTANTS.CONTROLLER_LOCATION,
            KickerConstants.MOTOR_CONSTANTS.FOLLOW_MOTOR_ID,
            true,
            KickerConstants.MOTOR_CONSTANTS.MOTOR_TYPE);
        followMotor.setMotorAsFollower(leadMotor, KickerConstants.MOTOR_CONSTANTS.FOLLOW_MOTOR_INVERTED_FROM_LEAD);
    }

    public void setDutyCycle(double targetDutyCycle){
        leadMotor.setDutyCycle(targetDutyCycle);
    }

    public void update(KickerInputs inputs){
        inputs.dutyCycle = leadMotor.getMotor().get();
        inputs.velocity = RotationsPerSecond.of(leadMotor.getVelocity());
    }

}
