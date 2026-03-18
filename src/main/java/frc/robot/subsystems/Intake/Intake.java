package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;


public class Intake extends SubsystemBase{
    public static Alert rollersStall = new Alert("Stall", "Intake Rollers in stall", AlertType.kWarning);
    public static Alert pivotStall = new Alert("Stall", "Intake Pivot in stall", AlertType.kWarning);

    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private IntakePosition state;

    public Intake()
    {
        io = Robot.isReal() ? new IntakeIOReal() : new IntakeIOSim();
        this.resetPivotPosition();
        state = IntakePosition.Open; // TODO: Swap to Closed

        // setDefaultCommand(this.moveToPositionCommand());
    }

    public void setPivotRotation(Angle rotations)
    {
        io.setPositionMotorRotation(rotations);
    }

    public void setPivotState(IntakePosition position){
        setPivotRotation(position.getAngle());
        this.state = position;
    }

    public void setRollersDutyCycle(double dutyCycle)
    {
        io.setRollersMotorDutyCycle(dutyCycle);
    }

    public Angle getCurrentPosition()
    {
        return io.getCurrentPosition();
    }

    public IntakePosition getCurrentState(){
        return this.state;
    }

    public void resetPivotPosition()
    {
        io.resetPositionMotorEncoder();
    }

    public void resetPivotPosition(IntakePosition state){
        io.resetPositionMotorEncoder(state.getAngle().in(Rotation));
    }

    public boolean rollersInStall(){
        return (RobotState.isEnabled()) && (io.getRollersSetpoint() != 0 && Math.abs(io.getRollersVelocity()) < 1);
    }
    public boolean pivotInStall(){
        return (RobotState.isEnabled()) && (io.getPivotStallCurrent() > IntakeConstants.PositionMotor.STALL_CURRENT);
    }

    public void startPIDTuning(){
        io.startPIDTuning();
    }

    public Command moveToPositionCommand(IntakePosition position){
        return this.runOnce(() -> setPivotState(position));
    }

    public Command moveToPositionCommand(){
        return this.run(() -> this.setPivotState(this.state));
    }

    public Command spinRollersCommand(){
        return this.startEnd(
            () -> setRollersDutyCycle(IntakeConstants.RollersMotor.DUTY_CYCLE),
            () -> setRollersDutyCycle(0)
        );
    }

    public Command bumpFuelCommand(){
        return Commands.sequence(
            new InstantCommand(() -> this.setPivotState(IntakePosition.Middle)),
            new WaitCommand(IntakeConstants.BUMP_WAIT_TIME),
            new InstantCommand(() -> this.setPivotState(IntakePosition.Open))
        );
    }

    @Override
    public void periodic()
    {
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        Logger.recordOutput("Intake/State", state);
        Logger.recordOutput("Intake/Command", (getCurrentCommand() != null ? getCurrentCommand().toString() : "None"));

        Logger.recordOutput("Intake/Rollers in Stall", rollersInStall());
        Logger.recordOutput("Intake/Pivot in Stall", pivotInStall());

        rollersStall.set(rollersInStall());
        pivotStall.set(pivotInStall());
    }
}
