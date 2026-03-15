package frc.robot.subsystems.Funnel;

public interface FunnelIO {
    void SetDutyCycleLead(double DutyCycleLead);
    void SetDutyCycleCenter(double DutyCycleCenter);

    double getFunnelVelocity();
    double getCenterringVelocity();

    double getFunnelSetpoint();
    double getCenterringSetpoint();
}
