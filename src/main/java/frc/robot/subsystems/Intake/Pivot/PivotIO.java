// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Pivot;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface PivotIO {
    @AutoLog
    class PivotInputs
    {
    }

    public void setRotation(double rotation);
    public double getRotation();

    public double getSupplyCurrent();
    public void resetPositionMotorEncoder(double angle);

    public void update(PivotInputs inputs);
}
