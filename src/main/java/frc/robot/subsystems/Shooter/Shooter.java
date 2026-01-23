// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    private final ShooterIO io; 
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    /** Creates a new Shooter. */
    public Shooter() {
        io = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
    }

    AngularVelocity getShooterVelocity(){ 
        return inputs.shooterVelocity; 
    }
    LinearVelocity getShooterLinearVelocity(){
        return inputs.shooterLinearVelocity;
    }

    void setShooterVelocity(AngularVelocity targetVelocity){ 
        io.setShooterVelocity(targetVelocity.in(RPM)); 
    }
    void setShooterLinearVelocity(LinearVelocity targetVelocity){
        double targetAngularVelocity = targetVelocity.in(Meters.per(Minute)) / ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE.in(Meters);
        io.setShooterVelocity(targetAngularVelocity);
    }

    
    AngularVelocity getKickerVelocity(){ 
        return inputs.kickerVelocity;
    }
    LinearVelocity getKickerLinearVelocity(){
        return inputs.kickerLinearVelocity;
    }

    void setKickerVelocity(AngularVelocity targetVelocity){ 
        io.setKickerVelocity(targetVelocity.in(RPM)); 
    }
    void setKickerLinearVelocity(LinearVelocity targetVelocity){
        double targetAngularVelocity = targetVelocity.in(Meters.per(Minute)) / ShooterConstants.KICKER_WHEEL_CIRCUMFERENCE.in(Meters);
        io.setShooterVelocity(targetAngularVelocity);
    }


    @Override
    public void periodic() {
        io.update(inputs);
    }
}
