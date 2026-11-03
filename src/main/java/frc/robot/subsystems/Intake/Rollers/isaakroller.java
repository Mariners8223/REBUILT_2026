package frc.robot.subsystems.Intake.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface isaakroller {
    @AutoLog
    class isaakrollerInputs{
     double power;
    }
       public void setDutyCycle(double dutyCycle);
       public void update(isaakrollerInputs inputs);


}