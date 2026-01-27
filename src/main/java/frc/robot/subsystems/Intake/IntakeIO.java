package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO 
{
    @AutoLog
    class IntakeInputs 
    {
        Pose3d pose;
        Angle currentPosition; 
        AngularVelocity positionMotorSpeed;//RPM
        AngularVelocity TopMotorSpeed;//RPM
        AngularVelocity BottomMotorSpeed;//RPM
    }

    void setPositionMotorVoltage (double voltage);

    void setTopMotorVoltage(double voltage);

    void setBottomMotorVoltage(double voltage);

    double getCurrentPosition();

    void resetAllMotorsEncoder();

    void Update(IntakeInputs inputs);
}
