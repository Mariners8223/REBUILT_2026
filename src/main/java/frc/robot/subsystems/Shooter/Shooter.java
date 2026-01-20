// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    private final ShooterIO io; 
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    /** Creates a new Shooter. */
    public Shooter() {
        // io = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
    }

    AngularVelocity getShooterVelocity(){ 
        return getShooterVelocity(); 
    }
    LinearVelocity getShooterLinearVelocity(){
        // LinearVelocity shooterLinearVelocity = LinearVelocity.ofRelativeUnits(
        //     getShooterVelocity().in(RotationsPerSecond) * ShooterConstants.shooterWheelCircumference.in(Meters),
        //     MetersPerSecond
        // );
        // return shooterLinearVelocity;

        return getShooterLinearVelocity();
    }

    void setShooterVelocity(AngularVelocity targetVelocity){ 
        io.setShooterVelocity(targetVelocity.in(RPM)); 
    }
    void setShooterLinearVelocity(LinearVelocity targetVelocity){
        double targetAngularVelocity = targetVelocity.in(Meters.per(Minute)) / ShooterConstants.shooterWheelCircumference.in(Meters);
        io.setShooterVelocity(targetAngularVelocity);
    }

    
    AngularVelocity getTriggerVelocity(){ 
        return getTriggerVelocity(); 
    }
    LinearVelocity getTriggerLinearVelocity(){
        return getTriggerLinearVelocity();
    }

    void setTriggerVelocity(AngularVelocity targetVelocity){ 
        io.setTriggerVelocity(targetVelocity.in(RPM)); 
    }
    void setTriggerLinearVelocity(LinearVelocity targetVelocity){
        double targetAngularVelocity = targetVelocity.in(Meters.per(Minute)) / ShooterConstants.triggerWheelRadius.in(Meters);
        io.setShooterVelocity(targetAngularVelocity);
    }



    @Override
    public void periodic() {
        io.update(inputs);
    }
}
