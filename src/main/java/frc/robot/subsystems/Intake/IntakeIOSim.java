package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotations;

import frc.util.MarinersController.MarinersSimMotor;

public class IntakeIOSim implements IntakeIO 
{
    MarinersSimMotor positionMotor;
    MarinersSimMotor topMotor;
    MarinersSimMotor bottomMotor;

    public IntakeIOSim()
    {
        positionMotor = new MarinersSimMotor("Position motor", null,IntakeConstants.PositionMotor.GEAR_RATIO,
        IntakeConstants.PositionMotor.ROTATIONS_PER_METERS.in(Rotations.per(Meter)), IntakeConstants.MOMENT_OF_INERTIA);

        positionMotor.enableSoftLimits(IntakeConstants.SOFT_MINIMUM, IntakeConstants.SOFT_MAXIMUM);
        positionMotor.setMotorInverted(IntakeConstants.PositionMotor.IS_INVERTED);

        topMotor = new MarinersSimMotor("Top motor", null, IntakeConstants.TopMotor.GEAR_RATIO,
        IntakeConstants.TopMotor.ROTATIONS_PER_METERS.in(Rotations.per(Meter)),IntakeConstants.MOMENT_OF_INERTIA);

        bottomMotor = new MarinersSimMotor("Bottom motor", null, IntakeConstants.BottomMotor.GEAR_RATIO,
        IntakeConstants.BottomMotor.ROTATIONS_PER_METERS.in(Rotations.per(Meter)),IntakeConstants.MOMENT_OF_INERTIA);
    }
    public void setPositionMotorVoltage(double voltage)
    {
        positionMotor.setVoltage(voltage);
    }

    public void setTopMotorVoltage(double voltage)
    {
        topMotor.setVoltage(voltage);
    }

    public void setBottomMotorVoltage(double voltage)
    {
        bottomMotor.setVoltage(voltage);
    }
}
