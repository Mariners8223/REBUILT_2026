package frc.robot.subsystems.Climb;


import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;

public class ClimbIOReal implements ClimbIO
{
    MarinersTalonFX motor;

    public ClimbIOReal()
    {
        motor = configureMotor();
    }

    public MarinersTalonFX configureMotor()
    {
        MarinersTalonFX motor = new MarinersTalonFX("Climb motor", ClimbConstants.CONTROLLER_LOCATION,
                                                    ClimbConstants.MOTOR_ID, ClimbConstants.PID);
        motor.getMeasurements().setGearRatio(ClimbConstants.ROTATIONS_TO_METERS * ClimbConstants.GEAR_RATIO);
        motor.setMotorInverted(ClimbConstants.IS_INVERTED);

        // motor.enableSoftLimits(ClimbConstants.SOFT_MINIMUM, ClimbConstants.SOFT_MAXIMUM); TODO: Add softlimits
        motor.setMotorIdleMode(true);
        motor.setPIDF(ClimbConstants.PID);
        return motor;
    }

    public void setDutyCycle(double dutyCycle)
    {
        motor.setDutyCycle(dutyCycle);
    }

    public void stopMotors()
    {
        motor.stopMotor();
    }

    public void resetPosition()
    {
        motor.setMotorEncoderPosition(ClimbConstants.START_POSITION);
    }

    public double getPosition()
    {
        return motor.getPosition();
    }

    public double getCurrent(){
        return motor.getMotor().getSupplyCurrent().getValueAsDouble();
    }

    public void setPosition(double position){
        motor.setReference(position, ControlMode.Position);
    }

    public void update(ClimbInputs inputs)
    {
        inputs.height = getPosition();
    }
}
