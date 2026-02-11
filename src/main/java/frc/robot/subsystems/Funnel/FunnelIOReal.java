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
    // private final MarinersController centerLowMotor;

    public FunnelIOReal(){
        funnelMotor = configureFunnelMotor();
        centerMotor = configureCenterMotor();
        // centerLowMotor = configureCenterLowMotor();
    }

    private MarinersSparkBase configureFunnelMotor(){
        MarinersSparkBase Motor;
        Motor = new MarinersSparkBase("FunnelingMotor", FunnelingConstents.LeadingMotor.CONTROLLER_LOCATION,
         FunnelingConstents.LeadingMotor.Lead_ID, FunnelingConstents.LeadingMotor.Is_Brushless,
         FunnelingConstents.LeadingMotor.MOTOR_TYPE);

         Motor.setMotorInverted(FunnelingConstents.LeadingMotor.IS_INVERTED);
         return Motor;
    }

    private MarinersTalonFX configureCenterMotor(){
        MarinersTalonFX Motor;
        Motor = new MarinersTalonFX("CenteringHighMotor", FunnelingConstents.CenteringMotor.CONTROLLER_LOCATION,
         FunnelingConstents.CenteringMotor.CenterHIGH_ID, new PIDFGains(0,0,0),
         FunnelingConstents.CenteringMotor.GearRatio);

         Motor.setMotorInverted(FunnelingConstents.CenteringMotor.IS_INVERTED);

          return Motor;
    }

    /*private MarinersSparkBase configureCenterLowMotor(){
        MarinersSparkBase Motor;
        Motor = new MarinersSparkBase("CenteringLowMotor", FunnelingConstents.CenteringLOWMotor.CONTROLLER_LOCATION,
         FunnelingConstents.CenteringLOWMotor.CenterLOW_ID,FunnelingConstents.CenteringLOWMotor.Is_Brushless,
          FunnelingConstents.CenteringLOWMotor.MOTOR_TYPE);
        Motor.setMotorInverted(false);
        
          return Motor;
    
    }*/

    // public void setDutyCycleCenterLow(double LowDutyCycle){
    //     centerLowMotor.setDutyCycle(LowDutyCycle);
    // }
    public void SetDutyCycleCenter(double HighDutyCycle){
        centerMotor.setDutyCycle(HighDutyCycle);
    }
    public void SetDutyCycleLead(double LeadDutyCycle){
        funnelMotor.setDutyCycle(LeadDutyCycle);
    }
    public void update(FunnelIOInputs inputs){

        }
}

    