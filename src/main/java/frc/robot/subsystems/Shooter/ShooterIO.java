// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    class ShooterInputs{
        double shooterVelocity; // RPM
        double shooterLinearVelocity; // m/s

        double triggerVelocity; // RPM
        double triggerLinearVelocity; // m/s

        Pose3d pose;
    }

    double getShooterVelocity();
    void setShooterVelocity(double targetVelocity);

    double getTriggerVelocity();
    void setTriggerVelocity(double targetVelocity);
    void setTriggerDutyCycle(double targetDutyCycle);

    void update(ShooterInputs inputs);
}
