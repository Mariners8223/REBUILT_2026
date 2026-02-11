package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
import edu.wpi.first.units.measure.Angle;


public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake()
    {
        io = Robot.isReal() ? new IntakeIOReal() : new IntakeIOSim();
        this.resetPositionMotorEncoder();
    }

    public void setPositionMotorRotation(Angle rotations)
    {
        io.setPositionMotorRotation(rotations);
    }

    public void setPositionMotorState(IntakePosition position){
        setPositionMotorRotation(position.getAngle());
    }

    public void setRollersMotorDutyCycle(double dutyCycle)
    {
        io.setRollersMotorDutyCycle(dutyCycle);
    }
    
    public Angle getCurrentPosition()
    {
        return io.getCurrentPosition();
    }

    public void resetPositionMotorEncoder()
    {
        io.resetPositionMotorEncoder();
    }

    public void startPIDTuning(){
        io.startPIDTuning();
    }

    public Command moveToPositionCommand(IntakePosition position){
        return this.runOnce(() -> setPositionMotorState(position));
    }

    public Command spinRollersCommand(){
        return this.startEnd(
            () -> setRollersMotorDutyCycle(IntakeConstants.RollersMotor.DUTY_CYCLE),
            () -> setRollersMotorDutyCycle(0)
        );
    }

    @Override
    public void periodic() 
    {
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        Logger.recordOutput("Intake/Command", (getCurrentCommand() != null ? getCurrentCommand().toString() : "None"));
    }
}
