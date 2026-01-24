// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Funnel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.util.MarinersController.MarinersSimMotor;

/** Add your docs here. */
public class FunnelIOSim implements FunnelIO{
    MarinersSimMotor LeadMotor;
    MarinersSimMotor CenterHighMotor;
    MarinersSimMotor CenterLowMotor;

    public FunnelIOSim(){
        LeadMotor= new MarinersSimMotor(
            "Lead Motor",
            DCMotor.getNeoVortex(1),
            FunnelingConstents.LeadingMotor.GearRatio,
            FunnelingConstents.LeadingMotor.MOMENT_OF_INERTIA_SIM);
        
        CenterHighMotor= new MarinersSimMotor(
            "Center High Motor",
            DCMotor.getNeoVortex(1),
            FunnelingConstents.CenteringHIGHMotor.GearRatio,
            FunnelingConstents.CenteringHIGHMotor.MOMENT_OF_INERTIA_SIM);
        CenterHighMotor= new MarinersSimMotor(
            "Center Low Motor",
            DCMotor.getNeoVortex(1),
            FunnelingConstents.CenteringLOWMotor.GearRatio,
            FunnelingConstents.CenteringLOWMotor.MOMENT_OF_INERTIA_SIM);
    }

    public void setDutyCycleCenterLow(double DutyCycleLOW){
        CenterLowMotor.setDutyCycle(DutyCycleLOW);
    }   
    public void SetDutyCycleCenterHigh(double DutyCycleHIGH){
        CenterHighMotor.setDutyCycle(DutyCycleHIGH);
    } 
    public void SetDutyCycleLead(double DutyCycleLead){
        LeadMotor.setDutyCycle(DutyCycleLead);
    }
    public void update(FunnelIOInputs inputs){

    }

}
