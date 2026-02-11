package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ClimbIO {
  @AutoLog
    class ClimbInputs{
        double height;
        Pose3d pose;
    }

    void setPower(double power);

    void resetPosition();

    double getPosition();

    void setPosition(double position);

    void stopClimbMotor();

    void setBrakeMode(boolean isBrake);

    void Update(ClimbInputs inputs);
}