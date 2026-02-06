package frc.robot.subsystems.Intake;

import frc.util.MarinersController.MarinersTalonFX;

import static edu.wpi.first.units.Units.Rotation;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import frc.util.MarinersController.MarinersController.ControlMode;

public class IntakeIOReal implements IntakeIO{
    private final MarinersTalonFX positionMotor;
    private final MarinersTalonFX rollersMotor;

    public IntakeIOReal()
    {
        this.positionMotor = configurePositionMotor();
        this.rollersMotor = configureRollersMotor();
    }
    private MarinersTalonFX configurePositionMotor()
    {
        MarinersTalonFX motor;
        motor = new MarinersTalonFX("position motor", IntakeConstants.PositionMotor.CONTROLLER_LOCATION, 
        IntakeConstants.PositionMotor.MOTOR_ID, IntakeConstants.PositionMotor.PID_GAINS);
        motor.setMotorInverted(IntakeConstants.PositionMotor.IS_INVERTED);
        motor.enableSoftLimits(IntakeConstants.PositionMotor.SOFT_MINIMUM, IntakeConstants.PositionMotor.SOFT_MAXIMUM);

        motor.setProfile(IntakeConstants.PositionMotor.PROFILE);

        // PID and Profile

        return motor;
    }
    private MarinersTalonFX configureRollersMotor()
    {
        MarinersTalonFX motor;
        motor = new MarinersTalonFX("rolllers motor", IntakeConstants.RollersMotor.CONTROLLER_LOCATION, 
        IntakeConstants.RollersMotor.MOTOR_ID);

        motor.setMotorInverted(IntakeConstants.RollersMotor.IS_INVERTED);

        return motor;
    }
    
    public void setPositionMotorRotation(Angle rotation)
    {
        positionMotor.setReference(rotation.in(Rotation), ControlMode.ProfiledVelocity);
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
        positionMotor.setMotorEncoderPosition(IntakeConstants.PositionMotor.IntakePosition.Bottom.getAngle().in(Rotation));
    }

    public void Update(IntakeInputs inputs)
    {
        inputs.currentPosition = getCurrentPosition();
        inputs.positionMotorSpeed = RPM.of(positionMotor.getVelocity());
        inputs.rollersMotorSpeed = RPM.of(rollersMotor.getVelocity());
    }
}
