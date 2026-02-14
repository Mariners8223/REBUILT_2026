// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    public LinearVelocity getShooterLinearVelocity(){
        return MetersPerSecond.of(inputs.shooterLinearVelocity);
    }

    public void stopShooter(){
        io.setDutyCycle(0);
    }
    public void setVelocity(AngularVelocity targetVelocity){
        io.setVelocity(targetVelocity.in(RotationsPerSecond));
    }
    public void setLinearVelocity(LinearVelocity targetVelocity){
        double targetAngularVelocity = targetVelocity.in(Meters.per(Minute)) / ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE.in(Meters);
        io.setVelocity(targetAngularVelocity);
    }
    public void setVoltage(Voltage voltage){
        io.setVoltage(voltage.in(Volts));
    }
    public void setDutyCycle(double dutyCycle){
        io.setDutyCycle(dutyCycle);
    }
    public void boostFeedForward(double boost){
        io.feedForwardBoost(boost);
    }
    public void resetFeedForward(){
        io.resetFeedForward();
    }

    public Command setDutyCycleCommand(double dutyCycle){
        return this.startEnd(
            () -> setDutyCycle(dutyCycle),
            () -> stopShooter()
        );
    }


    public void setVelocityByDistance(Distance distance){
        setVelocity(ShooterConstants.Calculations.requiredAngularVelocity(distance));
    }
    public boolean isAtRequiredVelocity(Distance distance){
        return ShooterConstants.Calculations.epsilonEquals(
            getShooterVelocity(),
            ShooterConstants.Calculations.requiredAngularVelocity(distance),
            ShooterConstants.SHOOTER_ANGULAR_VELOCITY_TOLERANCE
        );
    }
    public boolean isAtRequiredVelocity(AngularVelocity velocity){
        return ShooterConstants.Calculations.epsilonEquals(
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
