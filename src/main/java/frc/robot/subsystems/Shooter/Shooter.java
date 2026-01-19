// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO io; 
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    /** Creates a new Shooter. */
    public Shooter() {}

    double getShooterVelocity(){ return inputs.shooterVelocity; }
    void setShooterVelocity(double targetVelocity){ io.setShooterVelocity(targetVelocity); }
    void setShooterLinearVelocity(double targetVelocity){

    }

    // double getTriggerVelocity(){ return inputs.triggerVelocity }
    // void setTriggerVelocity(double targetVelocity);
    // void setTriggerDutyCycle(double targetDutyCycle);

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
