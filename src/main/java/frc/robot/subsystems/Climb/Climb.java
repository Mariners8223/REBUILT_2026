package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    public Climb() 
    {
        io = new ClimbIOReal();
        io.resetPosition();

        io.setBrakeMode(false);
    }

    public void resetPosition(){
        io.resetPosition();
    }

    public void setMotorPower(double power) 
    {
        io.setPower(power);
    }
    
    public double getPosition()
    {
        return io.getPosition();
    }

    public void stopClimbMotor()
    {
        io.stopClimbMotor();
    }
    
    public boolean isAtPosition(double position){
        return Math.abs(getPosition() - position) < ClimbConstants.CLIMB_TOLERANCE;
    }
    public void setPosition(double position){
        io.setPosition(position);
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
         io.Update(inputs);
         Logger.processInputs(getName(), inputs);

         double climbPrecnet = (inputs.height / ClimbConstants.SOFT_MINIMUM) * 100;

         SmartDashboard.putNumber("climb precent", climbPrecnet);

        String currentCommandName = getCurrentCommand() == null ? "Null" : getCurrentCommand().getName();
        Logger.recordOutput("Climb/Current Command", currentCommandName);
    }
}