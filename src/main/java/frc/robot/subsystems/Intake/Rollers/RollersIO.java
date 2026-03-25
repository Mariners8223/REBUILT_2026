// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Rollers;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface RollersIO {
    @AutoLog
    class RollersInputs {}

    public void setDutyCycle(double dutyCycle);

    public double getSetpoint();
    public double getVelocity();

    public void update(RollersInputs inputs);
}
