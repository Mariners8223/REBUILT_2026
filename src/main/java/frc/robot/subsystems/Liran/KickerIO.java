package frc.robot.subsystems.Liran;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface KickerIO {
    @AutoLog
    public class KickerInputs{
    }
    void setspeedrollers(double v);
    double getspeedrollers();
    void setspeedfunneling(double v);
    double getspeedfunneling();



}