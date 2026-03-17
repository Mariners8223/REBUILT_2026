package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ClimbIO {
  @AutoLog
    class ClimbInputs{
        double height;
        Pose3d pose;
    }

    void setDutyCycle(double dutyCycle);
    void stopMotors();
    
    void setPosition(double position);
    double getPosition();
    void resetPosition();

    double getCurrent();

    void update(ClimbInputs inputs);
}
