package frc.robot.subsystems.Climb;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  class ClimbInputs {
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
