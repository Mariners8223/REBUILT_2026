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
        double shooterSetPoint; // RPM
        double shooterLinearVelocity; // m/s
        double feedForward;

        Pose3d pose;
    }

    double getVelocity();
    double getAcceleration();
    void setVelocity(double targetVelocity);
    void setVoltage(double voltage);
    void setDutyCycle(double targetDutyCycle);
    void feedForwardBoost(double feedForwardBoost);
    void resetFeedForward();

    void update(ShooterInputs inputs);
    void setVelocityWithFeedforward(double in, double ff);
}
