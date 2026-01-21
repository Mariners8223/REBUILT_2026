package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface IntakeIO 
{
    @AutoLog
    class IntakeInputs 
    {
        Pose3d pose;
        double currentPosition;
        double positionMotorSpeed;
        double TopMotorSpeed;
        double BottomMotorSpeed;
    }

    void setPositionMotorVoltage (double v);

    void setTopMotorVoltage(double v);

    void setBottomMotorVoltage(double v);

    double getCurrentPosition();

    boolean isAtDesiredPosition();

    void resetAllMotorsEncoder();

    void Update(IntakeInputs inputs);
}
