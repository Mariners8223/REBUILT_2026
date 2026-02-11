package frc.robot.subsystems.Funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
    @AutoLog
    public class FunnelIOInputs{

    }

    void SetDutyCycleLead(double DutyCycleLead);
    void SetDutyCycleCenter(double DutyCycleCenter);
    // void setDutyCycleCenterLow(double DutyCycleCenterLow);
    void update(FunnelIOInputs inputs);
    
}
