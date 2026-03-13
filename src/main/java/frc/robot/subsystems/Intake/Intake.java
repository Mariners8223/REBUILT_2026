package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
import edu.wpi.first.units.measure.Angle;


public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private IntakePosition state;

    public Intake()
    {
        io = Robot.isReal() ? new IntakeIOReal() : new IntakeIOSim();
        this.resetPositionMotorEncoder();
        state = IntakePosition.Closed;

        setDefaultCommand(this.moveToPositionCommand(this.state));
    }

    public void setPositionMotorRotation(Angle rotations)
    {
        io.setPositionMotorRotation(rotations);
    }

    public void setPositionMotorState(IntakePosition position){
        setPositionMotorRotation(position.getAngle());
        this.state = position;
    }

    public void setRollersMotorDutyCycle(double dutyCycle)
    {
        io.setRollersMotorDutyCycle(dutyCycle);
    }

    public Angle getCurrentPosition()
    {
        return io.getCurrentPosition();
    }

    public IntakePosition getCurrentState(){
        return IntakePosition.findNearestPosition(getCurrentPosition());
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

    public Command switchIntakePositionCommand(){
        IntakePosition switchedPosition =
            state == IntakePosition.Closed ? IntakePosition.Open : IntakePosition.Closed;

        return this.runOnce(
            () -> setPositionMotorState(switchedPosition)
        );
    }

    public Command bumpFuelCommand(){
        return Commands.sequence(
            this.moveToPositionCommand(IntakePosition.Middle),
            new WaitCommand(IntakeConstants.BUMP_WAIT_TIME),
            this.moveToPositionCommand(IntakePosition.Open)
        );
    }

    @Override
    public void periodic()
    {
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        Logger.recordOutput("Intake/State", getCurrentState() == null ? "Unkown" : getCurrentState().toString());
        Logger.recordOutput("Intake/Command", (getCurrentCommand() != null ? getCurrentCommand().toString() : "None"));
    }
}
