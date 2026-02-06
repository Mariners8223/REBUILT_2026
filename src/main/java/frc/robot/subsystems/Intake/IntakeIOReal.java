package frc.robot.subsystems.Intake;

import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor;
import frc.util.MarinersController.MarinersController.ControlMode;

public class IntakeIOReal implements IntakeIO{
    private final MarinersSparkBase positionMotor;
    private final MarinersSparkBase topMotor;
    private final MarinersSparkBase bottomMotor;

    public IntakeIOReal()
    {
        this.positionMotor = configurePositionMotor();
        this.topMotor = configureTopMotor();
        this.bottomMotor = configureBottomMotor();
    }
    private MarinersSparkBase configurePositionMotor()
    {
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Position motor",IntakeConstants.PositionMotor.CONTROLLER_LOCATION, IntakeConstants.PositionMotor.MOTOR_ID,
        IntakeConstants.PositionMotor.IS_BRUSHLESS, IntakeConstants.PositionMotor.MOTOR_TYPE);
        
        motor.setMotorInverted(IntakeConstants.PositionMotor.IS_INVERTED);
        motor.enableSoftLimits(IntakeConstants.PositionMotor.SOFT_MINIMUM, IntakeConstants.PositionMotor.SOFT_MAXIMUM);

        // PID and Profile

        return motor;
    }
    private MarinersSparkBase configureTopMotor()
    {
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Top motor", IntakeConstants.TopMotor.CONTROLLER_LOCATION, IntakeConstants.TopMotor.MOTOR_ID,
        IntakeConstants.TopMotor.IS_BRUSHLESS, IntakeConstants.TopMotor.MOTOR_TYPE);

        motor.setMotorInverted(IntakeConstants.TopMotor.IS_INVERTED);

        return motor;
    }
    private MarinersSparkBase configureBottomMotor()
    {
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Bottom motor", IntakeConstants.BottomMotor.CONTROLLER_LOCATION, IntakeConstants.BottomMotor.MOTOR_ID,
        IntakeConstants.BottomMotor.IS_BRUSHLESS, IntakeConstants.BottomMotor.MOTOR_TYPE);

        motor.setMotorInverted(IntakeConstants.TopMotor.IS_INVERTED);

        motor.setMotorAsFollower(this.topMotor, IntakeConstants.BottomMotor.IS_INVERTED);

        return motor;
    }

    public void setPositionMotorRotation(Angle rotation)
    {
        positionMotor.setReference(rotation.in(Rotation), ControlMode.ProfiledVelocity);
    }

    public void setRollersDutyCycle(double dutyCycle)
    {
        topMotor.setDutyCycle(dutyCycle);
        bottomMotor.setDutyCycle(dutyCycle);
    }

    public Angle getCurrentPosition()
    {
        return positionMotor.getPosition().in(Angle);
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
        inputs.currentPosition = getCurrentPosition();
    }
}
