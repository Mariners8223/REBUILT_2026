// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    class ShooterInputs{
        AngularVelocity shooterVelocity; // RPM
        LinearVelocity shooterLinearVelocity; // m/s

        AngularVelocity kickerVelocity; // RPM
        LinearVelocity kickerLinearVelocity; // m/s

        Pose3d pose;
    }

    double getShooterVelocity();
    void setShooterVelocity(double targetVelocity);

    double getKickerVelocity();
    void setKickerVelocity(double targetVelocity);
    void setKickerDutyCycle(double targetDutyCycle);

    void update(ShooterInputs inputs);
}
