public interface ClimbIO {
  @AutoLog
    class ClimbInputs{
        double height;
        Pose3d pose;
    }

    void setPower(double power);

    void resetPosition();
    double getPosition();

    void setBrakeMode(boolean isBrake);

    void Update(ClimbInputs inputs);
    void setServoAngle(double angle);
}