package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbConstants.ClimbStates;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    public Climb()
    {
        io = new ClimbIOReal();
        io.resetPosition();
    }

    public void setDutyCycle(double power)
    {
        io.setDutyCycle(power);
    }
    public void stopMotors()
    {
        io.stopMotors();
    }

    public void setState(ClimbStates height){
        io.setPosition(height.getHeight());
    }
    public void resetPosition(){
        io.resetPosition();
    }
    public double getPosition()
    {
        return io.getPosition();
    }
    public boolean isAtPosition(double position){
        return Math.abs(getPosition() - position) < ClimbConstants.CLIMB_TOLERANCE;
    }

    public double getCurrent(){
        return io.getCurrent();
    }

    public Command toStateCommand(ClimbStates desiredState){
        return this.runOnce(
            () -> setState(desiredState)
        );
    }

    public Command dutyCycleCommand(double dutyCycle){
        return this.startEnd(
            () -> this.setDutyCycle(dutyCycle),
            () -> this.stopMotors()
        );
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        io.update(inputs);
        Logger.processInputs(getName(), inputs);

        String currentCommandName = getCurrentCommand() == null ? "Null" : getCurrentCommand().getName();
        Logger.recordOutput("Climb/Current Command", currentCommandName);
    }
}
