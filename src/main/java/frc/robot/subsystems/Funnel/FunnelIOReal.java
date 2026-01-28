// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Funnel;


import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;

/** Add your docs here. */
public class FunnelIOReal implements FunnelIO{
    private final MarinersSparkBase funnelMotor;
    private final MarinersSparkBase centerHighMotor;
    // private final MarinersController centerLowMotor;

    public FunnelIOReal(){
        funnelMotor = configureFunnelMotor();
        centerHighMotor = configureCenterHighMotor();
        // centerLowMotor = configureCenterLowMotor();
    }

    private MarinersSparkBase configureFunnelMotor(){
        MarinersSparkBase Motor;
        Motor = new MarinersSparkBase("FunnelingMotor", FunnelingConstents.LeadingMotor.ControllerLocation,
         FunnelingConstents.LeadingMotor.Lead_ID, FunnelingConstents.LeadingMotor.Is_Brushless,
         FunnelingConstents.LeadingMotor.MOTOR_TYPE);

         Motor.setMotorInverted(true);
         return Motor;
    }

    private MarinersSparkBase configureCenterHighMotor(){
        MarinersSparkBase Motor;
        Motor = new MarinersSparkBase("CenteringHighMotor", FunnelingConstents.CenteringHIGHMotor.CONTROLLER_LOCATION,
         FunnelingConstents.CenteringHIGHMotor.CenterHIGH_ID,FunnelingConstents.CenteringHIGHMotor.Is_Brushless,
         FunnelingConstents.CenteringHIGHMotor.MOTOR_TYPE, FunnelingConstents.CenteringHIGHMotor.GAINS,
         FunnelingConstents.CenteringHIGHMotor.GearRatio);

          return Motor;
    }

    private MarinersSparkBase configureCenterLowMotor(){
        MarinersSparkBase Motor;
        Motor = new MarinersSparkBase("CenteringLowMotor", FunnelingConstents.CenteringLOWMotor.CONTROLLER_LOCATION,
         FunnelingConstents.CenteringLOWMotor.CenterLOW_ID,FunnelingConstents.CenteringLOWMotor.Is_Brushless,
          FunnelingConstents.CenteringLOWMotor.MOTOR_TYPE);
        Motor.setMotorInverted(false);
        
          return Motor;
    
    }

    // public void setDutyCycleCenterLow(double LowDutyCycle){
    //     centerLowMotor.setDutyCycle(LowDutyCycle);
    // }
    public void SetDutyCycleCenterHigh(double HighDutyCycle){
        centerHighMotor.setDutyCycle(HighDutyCycle);
    }
    public void SetDutyCycleLead(double LeadDutyCycle){
        funnelMotor.setDutyCycle(LeadDutyCycle);
    }
    public void update(FunnelIOInputs inputs){

        }
}

    