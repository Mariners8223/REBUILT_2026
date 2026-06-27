// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Pivot;

import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;

/** Add your docs here. */
public class PivotIOReal implements PivotIO {
     private final MarinersTalonFX motor;

    public PivotIOReal(){
         motor = new MarinersTalonFX("Intake/Pivot Motor", PivotConstants.CONTROLLER_LOCATION,
                                     PivotConstants.MOTOR_ID, PivotConstants.PID_GAINS,
                                      PivotConstants.GEAR_RATIO);
         motor.setMotorInverted(PivotConstants.IS_INVERTED);
         motor.enableSoftLimits(PivotConstants.SOFT_MINIMUM, PivotConstants.SOFT_MAXIMUM);
         motor.setMotorIdleMode(true);
    }

    public void setRotation(double rotation){
         motor.setReference(rotation, ControlMode.Position);
    }
    public double getRotation(){
         return motor.getPosition();
        //return 3.4;
    }

    public double getSupplyCurrent(){
         return motor.getMotor().getSupplyCurrent().getValueAsDouble();
        //return 4.5;
    }
    public void resetPositionMotorEncoder(double angle){
         motor.setMotorEncoderPosition(angle);
    }

    public void update(PivotInputs inputs){ 
         //inputs.pivotstate = 
    }
}
