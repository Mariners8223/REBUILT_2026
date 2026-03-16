package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;

public interface IntakeIO
{
    @AutoLog
    class IntakeInputs
    {
        Pose3d pose;
        Angle currentPosition;
        double positionMotorSpeed;//RPM
        double rollersMotorSpeed;//RPM

        IntakePosition intakeState;
    }

    void setPositionMotorRotation (Angle rotation);

    void setRollersMotorDutyCycle(double dutyCycle);

    Angle getCurrentPosition();

    void resetPositionMotorEncoder();
    void resetPositionMotorEncoder(double angle);

    void startPIDTuning();

    double getRollersSetpoint();
    double getRollersVelocity();

    void Update(IntakeInputs inputs);
}
