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
        MarinersSparkBase Motor;
        Motor = new MarinersSparkBase("FunnelingMotor", FunnelConstants.LeadingMotor.CONTROLLER_LOCATION,
         FunnelConstants.LeadingMotor.Lead_ID, FunnelConstants.LeadingMotor.Is_Brushless,
         FunnelConstants.LeadingMotor.MOTOR_TYPE);

         Motor.setMotorInverted(FunnelConstants.LeadingMotor.IS_INVERTED);
        //  Motor.setCurrentLimits(FunnelConstants.LeadingMotor.LEAD_MOTOR_CURRENT_LIMIT,
        //      FunnelConstants.LeadingMotor.LEAD_MOTOR_CURRENT_THRESHOLD);
         return Motor;
    }

    private MarinersTalonFX configureCenterMotor(){
        MarinersTalonFX Motor;
        Motor = new MarinersTalonFX("CenteringHighMotor", FunnelConstants.CenteringMotor.CONTROLLER_LOCATION,
         FunnelConstants.CenteringMotor.CenterHIGH_ID, new PIDFGains(0,0,0),
         FunnelConstants.CenteringMotor.GearRatio);

         Motor.setMotorInverted(FunnelConstants.CenteringMotor.IS_INVERTED);
         Motor.setCurrentLimits(FunnelConstants.CenteringMotor.CENTER_MOTOR_CURRENT_LIMIT,
             FunnelConstants.CenteringMotor.CENTER_MOTOR_CURRENT_THRESHOLD);

          return Motor;
    }

    public void SetDutyCycleCenter(double HighDutyCycle){
        centerMotor.setDutyCycle(HighDutyCycle);
    }
    public void SetDutyCycleLead(double LeadDutyCycle){
        funnelMotor.setDutyCycle(LeadDutyCycle);
    }
}
