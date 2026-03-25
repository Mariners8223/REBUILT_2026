// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Rollers;

import frc.util.MarinersController.MarinersTalonFX;

/** Add your docs here. */
public class RollersIOReal implements RollersIO{
    private final MarinersTalonFX motor;

    public RollersIOReal(){
        motor = new MarinersTalonFX("Intake/Rollers Motor", RollerConstants.CONTROLLER_LOCATION,
                                    RollerConstants.MOTOR_ID);

        motor.setMotorInverted(RollerConstants.IS_INVERTED);
        motor.getMeasurements().setGearRatio(RollerConstants.GEAR_RATIO);
    }
    
    public void setDutyCycle(double dutyCycle){
        motor.setDutyCycle(dutyCycle);
    }

    public double getSetpoint(){
        return motor.getSetpoint();
    }
    public double getVelocity(){
        return motor.getVelocity();
    }

    public void update(RollersInputs inputs){}
}
