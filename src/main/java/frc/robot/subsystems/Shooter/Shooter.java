// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    /** Creates a new Shooter. */
    public Shooter() {
        io = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
    }

    public AngularVelocity getShooterVelocity(){
        return RotationsPerSecond.of(io.getVelocity());
    }
    public AngularAcceleration getShooterAcceleration(){
        return RotationsPerSecondPerSecond.of(io.getAcceleration());
    }

    public void setVelocity(AngularVelocity targetVelocity){
        io.setVelocity(targetVelocity.in(RotationsPerSecond));
    }
    public void setVelocity(Distance distance){
        setVelocity(ShooterConstants.requiredAngularVelocity(distance));
    }
    public void setDutyCycle(double dutyCycle){
        io.setDutyCycle(dutyCycle);
    }
    public void setVoltage(Voltage voltage){
        io.setVoltage(voltage.in(Volts));
    }

    public void stopShooter(){
        io.setDutyCycle(0);
    }

    public boolean isAtRequiredVelocity(Distance distance){
        return isAtRequiredVelocity(ShooterConstants.requiredAngularVelocity(distance));
    }
    public boolean isAtRequiredVelocity(AngularVelocity velocity){
        return Constants.epsilonEquals(
            getShooterVelocity(),
            velocity,
            ShooterConstants.SHOOTER_ANGULAR_VELOCITY_TOLERANCE);
    }


    @Override
    public void periodic() {
        io.update(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
