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

    public FunnelIOSim(){
        LeadMotor= new MarinersSimMotor(
            "Lead Motor",
            DCMotor.getNeoVortex(1),
            FunnelConstants.FunnelMotor.GEAR_RATIO,
            FunnelConstants.FunnelMotor.MOMENT_OF_INERTIA_SIM);

        CenterHighMotor= new MarinersSimMotor(
            "Center Motor",
            DCMotor.getKrakenX60(1),
            FunnelConstants.CenteringMotor.GEAR_RATIO,
            FunnelConstants.CenteringMotor.MOMENT_OF_INERTIA_SIM);
    }

    public void setDutyCycleCenter(double DutyCycleCenter){
        CenterHighMotor.setDutyCycle(DutyCycleCenter);
    }
    public void setDutyCycleFunnel(double DutyCycleLead){
        LeadMotor.setDutyCycle(DutyCycleLead);
    }

    public double getFunnelVelocity(){
        return LeadMotor.getVelocity();
    }
    public double getCenterringVelocity(){
        return CenterHighMotor.getVelocity();
    }

    public double getFunnelSetpoint(){
        return LeadMotor.getSetpoint();
    }
    public double getCenteringSetpoint(){
        return CenterHighMotor.getSetpoint();
    }
}
