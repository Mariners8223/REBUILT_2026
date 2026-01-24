package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.Intake.IntakeIO.IntakeInputs;
import edu.wpi.first.wpilibj.RobotBase;


public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    private double currentPosition;
    private double desiredPosition;


    public Intake()
    {
        io = new IntakeIOReal();
        this.resetAllMotorsEncoder();
    }

    public void setPositionMotorVoltage (double v)
    {
        io.setPositionMotorVoltage(v);
    }

    public void setTopMotorVoltage(double v)
    {
        io.setTopMotorVoltage(v);
    }

    public void setBottomMotorVoltage(double v)
    {
        io.setBottomMotorVoltage(v);
    }

    public double getCurrentPosition()
    {
        return io.getCurrentPosition();
    }

    public void resetAllMotorsEncoder()
    {
        io.resetAllMotorsEncoder();
    }

    public double getDesiredPosition()
    {
        return this.desiredPosition;
    }

    @Override
    public void periodic() {
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
