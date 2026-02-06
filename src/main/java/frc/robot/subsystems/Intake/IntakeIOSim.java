package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersSimMotor;
import frc.robot.subsystems.Intake.IntakeConstants;

public class IntakeIOSim implements IntakeIO 
{
    MarinersSimMotor positionMotor;
    MarinersSimMotor topMotor;
    MarinersSimMotor bottomMotor;

    public IntakeIOSim()
    {
        positionMotor = new MarinersSimMotor("Position motor", null,IntakeConstants.PositionMotor.GEAR_RATIO,
        1 , IntakeConstants.MOMENT_OF_INERTIA);

        positionMotor.enableSoftLimits(IntakeConstants.PositionMotor.SOFT_MINIMUM, IntakeConstants.PositionMotor.SOFT_MAXIMUM);
        positionMotor.setMotorInverted(IntakeConstants.PositionMotor.IS_INVERTED);

        topMotor = new MarinersSimMotor("Top motor", null, IntakeConstants.TopMotor.GEAR_RATIO,
        1 ,IntakeConstants.MOMENT_OF_INERTIA);

        bottomMotor = new MarinersSimMotor("Bottom motor", null, IntakeConstants.BottomMotor.GEAR_RATIO,
        1 ,IntakeConstants.MOMENT_OF_INERTIA);
    }
    public void setPositionMotorRotation(Angle rotation)
    {
        positionMotor.setReference(rotation.in(Rotation), ControlMode.ProfiledPosition);
    }

    public void setRollersDutyCycle(double dutyCycle)
    {
        topMotor.setDutyCycle(dutyCycle);
        bottomMotor.setDutyCycle(dutyCycle);
    }

    public Angle getCurrentPosition()
    {
        return positionMotor.getPosition();
    }

    public void setTopMotorDutyCycle(double dutyCycle)
    {
        topMotor.setDutyCycle(dutyCycle);
    }

    public void setBottomMotorDutyCycle(double dutyCycle)
    {
        bottomMotor.setDutyCycle(dutyCycle);
    }

    public void resetPositionMotorEncoder()
    {
        positionMotor.setMotorEncoderPosition(IntakeConstants.PositionMotor.BOTTOM_POSITION.in(Rotation));
    }

    public void Update(IntakeInputs inputs)
    {
        //inputs.currentPosition = getCurrentPosition();
    }
}
