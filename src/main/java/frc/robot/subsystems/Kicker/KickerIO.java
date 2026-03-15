// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface KickerIO {
    @AutoLog
    public class KickerInputs{
        double velocity; // RPM
        double dutyCycle;
    }

    void setDutyCycle(double targetDutyCycle);

    double getLeadVelocity();
    double getLeadSetpoint();

    double getFollowVelocity();
    double getFollowSetpoint();

    void update(KickerInputs inputs);
}
