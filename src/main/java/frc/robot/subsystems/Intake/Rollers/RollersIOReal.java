// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Rollers;

import frc.util.MarinersController.MarinersTalonFX;

/** Add your docs here. */
public class RollersIOReal implements RollersIO{
    private final MarinersTalonFX FirstMotor;
    private final MarinersTalonFX SecMotor;

    public RollersIOReal(){
        FirstMotor = new MarinersTalonFX("Intake/First Rollers Motor", RollerConstants.CONTROLLER_LOCATION,
                                    RollerConstants.MOTOR_ID);

        FirstMotor.setCurrentLimits(RollerConstants.CURRENT_LIMIT, RollerConstants.CURRENT_THRESHOLD);
        FirstMotor.setMotorInverted(RollerConstants.IS_INVERTED);
        FirstMotor.getMeasurements().setGearRatio(RollerConstants.GEAR_RATIO);

        SecMotor = new MarinersTalonFX("Intake/Sec Rollers Motor", RollerConstants.CONTROLLER_LOCATION,
                                    RollerConstants.SEC_MOTOR_ID);

        SecMotor.setCurrentLimits(RollerConstants.CURRENT_LIMIT, RollerConstants.CURRENT_THRESHOLD);
        SecMotor.setMotorInverted(RollerConstants.IS_SEC_INVERTED);
        SecMotor.getMeasurements().setGearRatio(RollerConstants.GEAR_RATIO);
    }

    public void setDutyCycle(double dutyCycle){
        FirstMotor.setDutyCycle(dutyCycle);
        SecMotor.setDutyCycle(dutyCycle);
    }

    public double getSetpoint(){
        return FirstMotor.getSetpoint();
    }
    public double getVelocity(){
        return FirstMotor.getVelocity();
    }

    public void update(RollersInputs inputs){}
}
