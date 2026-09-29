package frc.robot.subsystems.Liran;

import frc.robot.subsystems.Funnel.FunnelIO;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase;
import frc.robot.subsystems.Liran.kickerconstants;

public class kickerIOReal implements KickerIO{
    private static MarinersSparkBase leadMotor;
    private static MarinersSparkBase followMotor;
    public kickerIOReal(){
        leadMotor=new MarinersSparkBase(kickerconstants.LEADMOTOR.NAME, kickerconstants.LEADMOTOR.LOCATION, kickerconstants.LEADMOTOR.ID, kickerconstants.LEADMOTOR.ISBRUSHLESS, kickerconstants.LEADMOTOR.MOTOR_TYPE);
        followMotor=new MarinersSparkBase(kickerconstants.FUNNELMOTOR.NAME,kickerconstants.FUNNELMOTOR.LOCATION, kickerconstants.FUNNELMOTOR.ID, kickerconstants.FUNNELMOTOR.ISBRUSHLESS);
    }
    @Override
    public void setspeedrollers(double v) {
        leadMotor.setDutyCycle(v);
    }
 
    @Override
    public double getspeedrollers() {
       return (leadMotor.getVelocity());
    }
 
    @Override
    public void setspeedfunneling(double v) {
        followMotor.setDutyCycle(v);
    }

    @Override
    public double getspeedfunneling() {
        return (followMotor.getVelocity());
    }

}
