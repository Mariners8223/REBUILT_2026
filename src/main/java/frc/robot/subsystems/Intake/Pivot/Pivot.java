// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Pivot;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Pivot.PivotConstants.PivotStates;

public class Pivot extends SubsystemBase {
    public static Alert stallAlert = new Alert("Pivot in stall", AlertType.kWarning);

    private final PivotIO io;
    private final PivotInputsAutoLogged inputs = new PivotInputsAutoLogged();

    private PivotStates state;

    /** Creates a new Pivot. */
    public Pivot() {
        io = new PivotIOReal();
        state = PivotConstants.RESET;
        resetPosition();
    }

    public void setRotation(Angle rotations)
    {
        io.setRotation(rotations.in(Rotations));
    }
    public void setState(PivotStates position){
        setRotation(position.getAngle());
        this.state = position;
    }

    public Angle getPosition()
    {
        return Rotations.of(io.getRotation());
    }

    public PivotStates getState(){
        return this.state;
    }

    public void resetPosition()
    {
        this.resetPosition(PivotConstants.RESET);
    }
    public void resetPosition(PivotStates state){
        io.resetPositionMotorEncoder(state.getAngle().in(Rotations));
    }

    public boolean inStall(){
        return (RobotState.isEnabled()) && (io.getSupplyCurrent() > PivotConstants.STALL_CURRENT);
    }

    public Command moveToStateCommand(PivotStates state){
        return this.runOnce(() -> setState(state));
    }

    public Command raisePivot(PivotStates state){
        return this.startEnd(
            () -> setState(state),
            () -> setState(PivotStates.Open)
        );
    }

    public Command resetPivot(){
        return this.startEnd(
            () -> resetPosition(),
            () -> resetPosition()
        );
    }

    @Override
    public void periodic() {
        io.update(inputs);
        Logger.processInputs(getName(), inputs);

        Logger.recordOutput("Intake/State", getState());
        Logger.recordOutput("Intake/Pivot Command", (getCurrentCommand() != null ? getCurrentCommand().toString() : "None"));
        Logger.recordOutput("Intake/Pivot in Stall", inStall());

        stallAlert.set(inStall());
        // This method will be called once per scheduler run
    }
}
