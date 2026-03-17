// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Funnel;

import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;

/** Add your docs here. */
public class FunnelIOReal implements FunnelIO{
    private final MarinersSparkBase funnelMotor;
    private final MarinersTalonFX centerMotor;

    public FunnelIOReal(){
        funnelMotor = configureFunnelMotor();
        centerMotor = configureCenterMotor();
    }

    private MarinersSparkBase configureFunnelMotor(){
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Funnel/FunnelingMotor", FunnelConstants.funnelMotor.CONTROLLER_LOCATION,
         FunnelConstants.funnelMotor.Lead_ID, FunnelConstants.funnelMotor.IS_BRUSHLESS,
         FunnelConstants.funnelMotor.MOTOR_TYPE);

         motor.setMotorInverted(FunnelConstants.funnelMotor.IS_INVERTED);
        //  Motor.setCurrentLimits(FunnelConstants.LeadingMotor.LEAD_MOTOR_CURRENT_LIMIT,
        //      FunnelConstants.LeadingMotor.LEAD_MOTOR_CURRENT_THRESHOLD);
         return motor;
    }

    private MarinersTalonFX configureCenterMotor(){
        MarinersTalonFX Motor;
        Motor = new MarinersTalonFX("Funnel/CenteringMotor", FunnelConstants.CenteringMotor.CONTROLLER_LOCATION,
         FunnelConstants.CenteringMotor.CENTERING_ID, new PIDFGains(0,0,0),
         FunnelConstants.CenteringMotor.GEAR_RATIO);

         Motor.setMotorInverted(FunnelConstants.CenteringMotor.IS_INVERTED);
        //  Motor.setCurrentLimits(FunnelConstants.CenteringMotor.CENTER_MOTOR_CURRENT_LIMIT,
        //      FunnelConstants.CenteringMotor.CENTER_MOTOR_CURRENT_THRESHOLD);

          return Motor;
    }

    public void SetDutyCycleCenter(double HighDutyCycle){
        centerMotor.setDutyCycle(HighDutyCycle);
    }
    public void SetDutyCycleLead(double LeadDutyCycle){
        funnelMotor.setDutyCycle(LeadDutyCycle);
    }

    public double getFunnelVelocity(){
        return funnelMotor.getVelocity();
    }
    public double getCenterringVelocity(){
        return centerMotor.getVelocity();
    }

    public double getFunnelSetpoint(){
        return funnelMotor.getSetpoint();
    }
    public double getCenterringSetpoint(){
        return centerMotor.getSetpoint();
    }
}
