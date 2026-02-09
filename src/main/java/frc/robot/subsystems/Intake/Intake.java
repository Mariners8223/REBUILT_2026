package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor.IntakePosition;
import frc.robot.subsystems.Intake.IntakeIO.IntakeInputs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;


public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    private double currentPosition;
    private double desiredPosition;


    public Intake()
    {
        io = new IntakeIOReal();
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

    @Override
    public void periodic() 
    {
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        Logger.recordOutput("Intake/Command", (getCurrentCommand() != null ? getCurrentCommand().toString() : "None"));
    }
}
