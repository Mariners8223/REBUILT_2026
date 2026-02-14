package frc.robot.subsystems.Climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

        motor.enableSoftLimits(ClimbConstants.SOFT_MINIMUM, ClimbConstants.SOFT_MAXIMUM);
        motor.startPIDTuning();
        return motor;
    }

    public void setPower(double power)
    {
        motor.setDutyCycle(power);
    }

    public void stopClimbMotor()
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
        return motor.getMotor().getTorqueCurrent().getValueAsDouble();
    }

    public void setPosition(double refrence){
        motor.setReference(refrence, ControlMode.Position);
    }

    @Override
    public void setBrakeMode(boolean isBrake)
    {
        motor.setMotorIdleMode(isBrake);
    }

    public void Update(ClimbInputs inputs)
    {
        inputs.height = getPosition();
        inputs.pose = new Pose3d(ClimbConstants.X_ON_ROBOT, ClimbConstants.Y_ON_ROBOT, getPosition(), new Rotation3d());
    }
}
