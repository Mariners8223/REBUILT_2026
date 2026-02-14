package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersSimMotor;
import static edu.wpi.first.units.Units.RPM;

public class IntakeIOSim implements IntakeIO
{
    MarinersSimMotor positionMotor;
    MarinersSimMotor rollersMotor;

    public IntakeIOSim()
    {
        positionMotor = new MarinersSimMotor("Position motor", null,IntakeConstants.PositionMotor.GEAR_RATIO,
        1 , IntakeConstants.MOMENT_OF_INERTIA);

        positionMotor.enableSoftLimits(IntakeConstants.PositionMotor.SOFT_MINIMUM, IntakeConstants.PositionMotor.SOFT_MAXIMUM);
        positionMotor.setMotorInverted(IntakeConstants.PositionMotor.IS_INVERTED);

        rollersMotor = new MarinersSimMotor("Rollers motor", null, IntakeConstants.RollersMotor.GEAR_RATIO,
        1 ,IntakeConstants.MOMENT_OF_INERTIA);

    }
    public void setPositionMotorRotation(Angle rotation)
    {
        positionMotor.setReference(rotation.in(Rotation), ControlMode.ProfiledPosition);
    }

    public void setRollersMotorDutyCycle(double dutyCycle)
    {
        rollersMotor.setDutyCycle(dutyCycle);
    }

    public Angle getCurrentPosition()
    {
        return Rotation.of(positionMotor.getPosition());
    }

    public void resetPositionMotorEncoder()
    {
        positionMotor.setMotorEncoderPosition(IntakeConstants.PositionMotor.IntakePosition.Closed.getAngle().in(Rotation));
    }

    public void startPIDTuning(){
        positionMotor.startPIDTuning();
    }

    public void Update(IntakeInputs inputs)
    {
        inputs.currentPosition = getCurrentPosition();
        inputs.positionMotorSpeed = RotationsPerSecond.of(positionMotor.getVelocity()).in(RPM);
        inputs.rollersMotorSpeed = RotationsPerSecond.of(rollersMotor.getVelocity()).in(RPM);
    }
}
