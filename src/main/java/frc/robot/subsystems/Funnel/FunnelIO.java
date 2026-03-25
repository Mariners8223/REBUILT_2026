package frc.robot.subsystems.Funnel;

public interface FunnelIO {
    void setDutyCycleFunnel(double funnelDutyCycle);
    void setDutyCycleCenter(double centeringDutyCycle);

    double getFunnelVelocity();
    double getCenterringVelocity();

    double getFunnelSetpoint();
    double getCenteringSetpoint();
}
