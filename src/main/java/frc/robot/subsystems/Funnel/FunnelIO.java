package frc.robot.subsystems.Funnel;

public interface FunnelIO {
    void setDutyCycleFunnel(double DutyCycleLead);
    void setDutyCycleCenter(double DutyCycleCenter);

    double getFunnelVelocity();
    double getCenterringVelocity();

    double getFunnelSetpoint();
    double getCenterringSetpoint();
}
