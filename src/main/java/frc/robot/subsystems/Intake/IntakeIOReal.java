package frc.robot.subsystems.Intake;

import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
import frc.util.MarinersController.MarinersTalonFX;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        IntakeConstants.PositionMotor.MOTOR_ID, IntakeConstants.PositionMotor.PID_GAINS, IntakeConstants.PositionMotor.GEAR_RATIO);
        motor.setMotorInverted(IntakeConstants.PositionMotor.IS_INVERTED);
        // motor.enableSoftLimits(IntakeConstants.PositionMotor.SOFT_MINIMUM, IntakeConstants.PositionMotor.SOFT_MAXIMUM);
        motor.setMotorIdleMode(true);
        // PID and Profile
        // motor.setMaxMinOutput(4, -2);
        motor.setProfile(IntakeConstants.PositionMotor.PROFILE);
        // motor.startPIDTuning();

        SmartDashboard.putString("Intake PID", motor.getPIDF().toString());
        return motor;
    }
    private MarinersTalonFX configureRollersMotor()
    {
        MarinersTalonFX motor;
        motor = new MarinersTalonFX("rollers motor", IntakeConstants.RollersMotor.CONTROLLER_LOCATION,
        IntakeConstants.RollersMotor.MOTOR_ID);

        motor.setMotorInverted(IntakeConstants.RollersMotor.IS_INVERTED);

        return motor;
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
        positionMotor.setMotorEncoderPosition(IntakeConstants.PositionMotor.IntakePosition.Open.getAngle().in(Rotation));
    }

    public void startPIDTuning(){
        positionMotor.startPIDTuning();
    }

    public void Update(IntakeInputs inputs)
    {
        inputs.currentPosition = getCurrentPosition();
        inputs.positionMotorSpeed = RotationsPerSecond.of(positionMotor.getVelocity()).in(RPM);
        inputs.rollersMotorSpeed = RotationsPerSecond.of(rollersMotor.getVelocity()).in(RPM);
        inputs.intakeState = IntakePosition.findNearestPosition(inputs.currentPosition);

        Logger.recordOutput("Setpoint", positionMotor.getSetpoint());
    }
}
