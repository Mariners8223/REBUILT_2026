package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    public Climb()
    {
        io = new ClimbIOReal();
        io.resetPosition();

        io.setBrakeMode(true);
    }

    public void resetPosition(){
        io.resetPosition();
    }

    public void setMotorPower(double power)
    {
        io.setPower(power);
    }

    public void setMotorHeight(Heights height){
        io.setPosition(height.getHeight());
    }

    public double getPosition()
    {
        return io.getPosition();
    }

    public void stopClimbMotor()
    {
        io.stopClimbMotor();
    }

    public double getCurrent(){
        return io.getCurrent();
    }

    public boolean isAtPosition(double position){
        return Math.abs(getPosition() - position) < ClimbConstants.CLIMB_TOLERANCE;
    }

    public Command toPositionCommand(Heights desiredHeight){
        return this.runOnce(
            () -> setMotorHeight(desiredHeight)
        );
    }

    public Command dutyCycleCommand(double dutyCycle){
        return this.startEnd(
            () -> this.setMotorPower(dutyCycle),
            () -> this.stopClimbMotor()
        );
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        double percent = (inputs.height / ClimbConstants.SOFT_MINIMUM) * 100;

        SmartDashboard.putNumber("Climb/Climb Percent", percent);

        String currentCommandName = getCurrentCommand() == null ? "Null" : getCurrentCommand().getName();
        Logger.recordOutput("Climb/Current Command", currentCommandName);
    }
}
